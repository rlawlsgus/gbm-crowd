using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.IO;

public class SocialTaskCompletionRate : MonoBehaviour
{
    [Header("Agent Settings")]
    [Tooltip("List of Agent Prefabs to identify in the scene.")]
    public List<GameObject> agentPrefabs = new List<GameObject>();
    [Tooltip("Layer to search for agents. Make sure agents are on this layer.")]
    public LayerMask searchLayer;
    [Tooltip("Interval in seconds to search for new agents.")]
    public float searchInterval = 0.5f;

    [Header("Environment Settings")]
    [Tooltip("The parent object containing all Danger Zone cubes. Will auto-find 'Danger Zones' if null.")]
    public Transform dangerZonesRoot;
    [Tooltip("The parent object containing all Obstacle cubes. Will auto-find 'Obstacles' if null.")]
    public Transform obstaclesRoot;
    
    [Header("References")]
    public TestManager testManager;

    [Header("Trajectory Map Settings")]
    public bool enableTrajectoryMap = true;
    public int mapResolution = 2048;
    public float mapWidth = 100f;
    public float mapHeight = 100f;
    public Color normalPathColor = Color.red;
    public Color dangerPathColor = Color.yellow;
    public Color dangerZoneOutlineColor = Color.blue;
    public Color obstacleOutlineColor = new Color(1f, 0.5f, 0f); // Orange
    public string mapFileName = "SocialTrajectoryMap.png";
    public float minRecordDistance = 0.1f;
    public int endPointRadius = 10;

    private List<BoxCollider> dangerZoneColliders = new List<BoxCollider>();
    private List<BoxCollider> obstacleColliders = new List<BoxCollider>();
    private float searchTimer = 0f;

    public struct TrajectoryPoint
    {
        public Vector3 position;
        public bool isDanger;

        public TrajectoryPoint(Vector3 pos, bool danger)
        {
            position = pos;
            isDanger = danger;
        }
    }

    // Tracking Data Structure
    public class AgentTrackingData
    {
        public float totalTime;
        public float dangerZoneTime;
        public float obstacleTime;
        public float agentCollisionTime;
        public Collider agentCollider;
        public bool hasReachedGoal; // Cache success status
        public List<TrajectoryPoint> trajectory = new List<TrajectoryPoint>();
    }

    // Dictionary to store data for each agent
    // Key: The agent's Transform in the scene
    private Dictionary<Transform, AgentTrackingData> trackingData = new Dictionary<Transform, AgentTrackingData>();

    void Start()
    {
        if (testManager == null)
        {
            testManager = FindObjectOfType<TestManager>();
        }

        // Auto-find Danger Zones if not assigned
        if (dangerZonesRoot == null)
        {
            GameObject foundObj = GameObject.Find("Danger Zones");
            if (foundObj != null)
            {
                dangerZonesRoot = foundObj.transform;
            }
        }

        // Find all BoxColliders in the Danger Zones Root
        if (dangerZonesRoot != null)
        {
            dangerZoneColliders = dangerZonesRoot.GetComponentsInChildren<BoxCollider>().ToList();
            Debug.Log($"[SocialTaskCompletionRate] Found {dangerZoneColliders.Count} danger zone colliders under '{dangerZonesRoot.name}'.");
        }
        else
        {
            Debug.LogWarning("[SocialTaskCompletionRate] 'Danger Zones' object not found in scene and not assigned.");
        }

        // Auto-find Obstacles if not assigned
        if (obstaclesRoot == null)
        {
            GameObject foundObj = GameObject.Find("Obstacles");
            if (foundObj != null)
            {
                obstaclesRoot = foundObj.transform;
            }
        }

        // Find all BoxColliders in the Obstacles Root
        if (obstaclesRoot != null)
        {
            obstacleColliders = obstaclesRoot.GetComponentsInChildren<BoxCollider>().ToList();
            Debug.Log($"[SocialTaskCompletionRate] Found {obstacleColliders.Count} obstacle colliders under '{obstaclesRoot.name}'.");
        }
        else
        {
            Debug.LogWarning("[SocialTaskCompletionRate] 'Obstacles' object not found in scene and not assigned.");
        }
    }

    void Update()
    {
        // Search for agents periodically (Optimization)
        searchTimer += Time.deltaTime;
        if (searchTimer >= searchInterval)
        {
            SearchForAgents();
            searchTimer = 0f;
        }

        // Update stats for tracked agents every frame
        UpdateAgentStats();
    }

    private void SearchForAgents()
    {
        // 1. Try finding agents via TestManager (Preferred)
        if (testManager != null && testManager.activeAgents != null)
        {
            foreach (var agent in testManager.activeAgents)
            {
                if (agent == null) continue;
                RegisterAgent(agent.transform);
            }
        }

        // 2. Fallback: Search by iterating children of "Agents" parent object
        GameObject agentsParent = GameObject.Find("Agents");
        if (agentsParent != null)
        {
            foreach (Transform child in agentsParent.transform)
            {
                RegisterAgent(child);
            }
        }
    }

    private void RegisterAgent(Transform agentTransform)
    {
        // Skip if already tracked
        if (trackingData.ContainsKey(agentTransform)) return;

        // Check for AgentGBM or AgentBase
        AgentBase agentBase = agentTransform.GetComponent<AgentBase>();

        if (agentBase != null)
        {
            // We need a collider to track danger zone entry
            Collider col = agentTransform.GetComponent<Collider>();

            // If main collider is not on root, try finding it in children
            if (col == null)
            {
                col = agentTransform.GetComponentInChildren<Collider>();
            }

            if (col != null)
            {
                AgentTrackingData newData = new AgentTrackingData();
                newData.agentCollider = col;
                trackingData.Add(agentTransform, newData);
            }
        }
    }

    private void UpdateAgentStats()
    {
        float dt = Time.deltaTime;

        foreach (var kvp in trackingData)
        {
            Transform agent = kvp.Key;
            AgentTrackingData data = kvp.Value;

            if (agent == null) continue; // Agent might be destroyed

            // Check and cache reachedGoal status BEFORE checking active status
            if (!data.hasReachedGoal)
            {
                AgentGBM gbm = agent.GetComponent<AgentGBM>();
                if (gbm != null && gbm.GoalReached)
                {
                    data.hasReachedGoal = true;
                }
            }

            if (!agent.gameObject.activeInHierarchy) continue; // Agent finished

            // Increment Total Time
            data.totalTime += dt;

            // 1. Check Danger Zone
            if (IsInDangerZone(data.agentCollider))
            {
                data.dangerZoneTime += dt;
            }

            // 2. Check Obstacle Collision
            if (IsCollidingWithObstacle(data.agentCollider))
            {
                data.obstacleTime += dt;
            }

            // 3. Check Agent Collision
            if (IsCollidingWithAgent(data.agentCollider, agent))
            {
                data.agentCollisionTime += dt;
            }

            // Record Trajectory
            if (enableTrajectoryMap)
            {
                Vector3 currentPos = agent.position;
                bool isDanger = (data.dangerZoneTime > 0) && IsInDangerZone(data.agentCollider); // Simple visualization check
                
                if (data.trajectory.Count == 0 || Vector3.Distance(data.trajectory[data.trajectory.Count - 1].position, currentPos) >= minRecordDistance)
                {
                    data.trajectory.Add(new TrajectoryPoint(currentPos, isDanger));
                }
            }
        }
    }

    // Check if the agent collider overlaps any of the danger zone colliders
    private bool IsInDangerZone(Collider agentCollider)
    {
        if (agentCollider == null) return false;
        foreach (var col in dangerZoneColliders)
        {
            if (col != null && col.enabled && CheckCollision(agentCollider, col)) return true;
        }
        return false;
    }

    // Check if the agent collider overlaps any of the obstacle colliders
    private bool IsCollidingWithObstacle(Collider agentCollider)
    {
        if (agentCollider == null) return false;
        foreach (var col in obstacleColliders)
        {
            if (col != null && col.enabled && CheckCollision(agentCollider, col)) return true;
        }
        return false;
    }

    // Check if the agent collider overlaps any OTHER agent collider
    private bool IsCollidingWithAgent(Collider currentAgentCollider, Transform currentAgentTransform)
    {
        if (currentAgentCollider == null) return false;

        foreach (var kvp in trackingData)
        {
            Transform otherAgent = kvp.Key;
            AgentTrackingData otherData = kvp.Value;

            if (otherAgent == currentAgentTransform) continue; // Skip self
            if (otherAgent == null || !otherAgent.gameObject.activeInHierarchy) continue;
            if (otherData.agentCollider == null) continue;

            if (CheckCollision(currentAgentCollider, otherData.agentCollider)) return true;
        }
        return false;
    }

    private bool CheckCollision(Collider c1, Collider c2)
    {
        if (!c1.bounds.Intersects(c2.bounds)) return false;

        Vector3 direction;
        float distance;
        return Physics.ComputePenetration(
            c1, c1.transform.position, c1.transform.rotation,
            c2, c2.transform.position, c2.transform.rotation,
            out direction, out distance);
    }

    /// <summary>
    /// Calculates the Social Task Completion Rate for a specific agent.
    /// Rate = 1.0 - (CombinedDangerTime / TotalTime).
    /// </summary>
    public float CalculateRate(AgentTrackingData data)
    {
        if (data.totalTime <= 0.0001f) return 1f; 

        // Summing up times. Note: Overlap is possible (e.g., agent collides with another agent INSIDE a danger zone).
        // This is a simple summation; weights can be adjusted if needed.
        // The prompt asked to include Agent Collision in Safe Rate.
        
        float combinedUnsafeTime = data.dangerZoneTime + data.obstacleTime + data.agentCollisionTime;
        
        // Clamp to ensure it doesn't exceed totalTime (though physically it could if we double count separate events, 
        // but here we are just taking duration. If multiple overlap, time flows same.)
        // Actually, if we just sum them, it might exceed totalTime if they happen simultaneously. 
        // A better approach for "Safe Rate" is: Time spent in ANY unsafe state / Total Time.
        // But tracking "Any Unsafe State" per frame is better.
        // For now, I will stick to the requested metric sum, but clamp result.
        
        float rate = 1.0f - (combinedUnsafeTime / data.totalTime);
        return Mathf.Clamp01(rate);
    }

    private void OnApplicationQuit()
    {
        Debug.Log("=== Social Task Completion Rate Report ===");

        int successCount = 0;
        int totalAgents = 0;
        
        List<float> successSafeRates = new List<float>();
        
        // Accumulators for specific violation rates (averaged across ALL agents or Success agents? Usually Success)
        List<float> dangerZoneRates = new List<float>();
        List<float> obstacleRates = new List<float>();
        List<float> agentCollisionRates = new List<float>();

        foreach (var kvp in trackingData)
        {
            AgentTrackingData data = kvp.Value;
            totalAgents++;

            bool isSuccess = data.hasReachedGoal;

            if (isSuccess && data.totalTime > 0.0001f)
            {
                successCount++;
                successSafeRates.Add(CalculateRate(data));
                
                dangerZoneRates.Add(data.dangerZoneTime / data.totalTime);
                obstacleRates.Add(data.obstacleTime / data.totalTime);
                agentCollisionRates.Add(data.agentCollisionTime / data.totalTime);
            }
        }

        float successRate = totalAgents > 0 ? (float)successCount / totalAgents : 0f;
        
        float avgSafeRate = successSafeRates.Count > 0 ? successSafeRates.Average() : 0f;
        float avgDangerZoneRate = dangerZoneRates.Count > 0 ? dangerZoneRates.Average() : 0f;
        float avgObstacleRate = obstacleRates.Count > 0 ? obstacleRates.Average() : 0f;
        float avgAgentCollRate = agentCollisionRates.Count > 0 ? agentCollisionRates.Average() : 0f;

        Debug.Log($"[Overall] Success Rate: {successRate * 100:F2}% ({successCount}/{totalAgents})");
        Debug.Log($"[Safety (Successes)] Avg Safe Rate: {avgSafeRate * 100:F2}%");
        Debug.Log($"   - Danger Zone Violation Rate: {avgDangerZoneRate * 100:F2}%");
        Debug.Log($"   - Obstacle Collision Rate: {avgObstacleRate * 100:F2}%");
        Debug.Log($"   - Agent Collision Rate: {avgAgentCollRate * 100:F2}%");

        Debug.Log("==========================================");

        if (enableTrajectoryMap)
        {
            GenerateTrajectoryMap();
        }
    }

    private void GenerateTrajectoryMap()
    {
        Texture2D texture = new Texture2D(mapResolution, mapResolution);
        // Fill white
        Color[] resetColor = new Color[mapResolution * mapResolution];
        for (int i = 0; i < resetColor.Length; i++) resetColor[i] = Color.white;
        texture.SetPixels(resetColor);

        float minX = -mapWidth / 2f;
        float minZ = -mapHeight / 2f;

        // 1. Draw Danger Zones (Blue Outline)
        DrawColliders(texture, dangerZoneColliders, dangerZoneOutlineColor, minX, minZ);

        // 2. Draw Obstacles (Orange Outline)
        DrawColliders(texture, obstacleColliders, obstacleOutlineColor, minX, minZ);

        // 3. Draw Trajectories
        foreach (var kvp in trackingData)
        {
            List<TrajectoryPoint> path = kvp.Value.trajectory;
            if (path.Count < 2) continue;

            Vector2 prevPixel = WorldToPixel(path[0].position, minX, minZ);

            for (int i = 1; i < path.Count; i++)
            {
                Vector2 currentPixel = WorldToPixel(path[i].position, minX, minZ);
                Color color = path[i].isDanger ? dangerPathColor : normalPathColor;
                DrawLine(texture, prevPixel, currentPixel, color, 3);
                prevPixel = currentPixel;
            }

            // Draw End Point Circle
            DrawCircle(texture, prevPixel, endPointRadius, normalPathColor);
        }

        texture.Apply();
        byte[] bytes = texture.EncodeToPNG();
        string pathToFile = Path.Combine(Application.dataPath, mapFileName);
        File.WriteAllBytes(pathToFile, bytes);
        Debug.Log($"[SocialTaskCompletionRate] Trajectory map saved to: {pathToFile}");
    }

    private void DrawColliders(Texture2D texture, List<BoxCollider> colliders, Color color, float minX, float minZ)
    {
        foreach (var col in colliders)
        {
            if (col == null) continue;

            // Get corners in world space (XZ plane approximation)
            Transform t = col.transform;
            Vector3 center = col.center;
            Vector3 size = col.size;

            // Local corners (bottom face)
            Vector3 p1 = t.TransformPoint(center + new Vector3(-size.x, -size.y, -size.z) * 0.5f);
            Vector3 p2 = t.TransformPoint(center + new Vector3(size.x, -size.y, -size.z) * 0.5f);
            Vector3 p3 = t.TransformPoint(center + new Vector3(size.x, -size.y, size.z) * 0.5f);
            Vector3 p4 = t.TransformPoint(center + new Vector3(-size.x, -size.y, size.z) * 0.5f);

            // Convert to pixels
            Vector2 px1 = WorldToPixel(p1, minX, minZ);
            Vector2 px2 = WorldToPixel(p2, minX, minZ);
            Vector2 px3 = WorldToPixel(p3, minX, minZ);
            Vector2 px4 = WorldToPixel(p4, minX, minZ);

            DrawLine(texture, px1, px2, color, 2);
            DrawLine(texture, px2, px3, color, 2);
            DrawLine(texture, px3, px4, color, 2);
            DrawLine(texture, px4, px1, color, 2);
        }
    }

    private Vector2 WorldToPixel(Vector3 worldPos, float minX, float minZ)
    {
        float u = (worldPos.x - minX) / mapWidth;
        float v = (worldPos.z - minZ) / mapHeight;
        return new Vector2(u * (mapResolution - 1), v * (mapResolution - 1));
    }

    private void DrawLine(Texture2D tex, Vector2 p1, Vector2 p2, Color col, int thickness)
    {
        int x0 = (int)p1.x;
        int y0 = (int)p1.y;
        int x1 = (int)p2.x;
        int y1 = (int)p2.y;

        int dx = Mathf.Abs(x1 - x0);
        int dy = Mathf.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            DrawBrush(tex, x0, y0, col, thickness);

            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }

    private void DrawBrush(Texture2D tex, int x, int y, Color col, int thickness)
    {
        int half = thickness / 2;
        for (int i = -half; i <= half; i++)
        {
            for (int j = -half; j <= half; j++)
            {
                if (x + i >= 0 && x + i < tex.width && y + j >= 0 && y + j < tex.height)
                {
                    tex.SetPixel(x + i, y + j, col);
                }
            }
        }
    }

    private void DrawCircle(Texture2D tex, Vector2 center, int radius, Color col)
    {
        int cx = (int)center.x;
        int cy = (int)center.y;

        for (int x = -radius; x <= radius; x++)
        {
            for (int y = -radius; y <= radius; y++)
            {
                if (x * x + y * y <= radius * radius)
                {
                    int px = cx + x;
                    int py = cy + y;
                    if (px >= 0 && px < tex.width && py >= 0 && py < tex.height)
                    {
                        tex.SetPixel(px, py, col);
                    }
                }
            }
        }
    }
}