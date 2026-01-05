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

    [Header("Trajectory Map Settings")]
    public bool enableTrajectoryMap = true;
    public int mapResolution = 2048;
    public float mapWidth = 100f;
    public float mapHeight = 100f;
    public Color normalPathColor = Color.red;
    public Color dangerPathColor = Color.yellow;
    public Color dangerZoneOutlineColor = Color.blue;
    public string mapFileName = "SocialTrajectoryMap.png";
    public float minRecordDistance = 0.1f;
    public int endPointRadius = 10;

    private List<BoxCollider> dangerZoneColliders = new List<BoxCollider>();
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
        public float dangerTime;
        public bool hasReachedGoal;
        public Collider agentCollider;
        public List<TrajectoryPoint> trajectory = new List<TrajectoryPoint>();
    }

    // Dictionary to store data for each agent
    // Key: The agent's Transform in the scene
    private Dictionary<Transform, AgentTrackingData> trackingData = new Dictionary<Transform, AgentTrackingData>();

    void Start()
    {
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
        if (agentPrefabs == null || agentPrefabs.Count == 0) return;

        // Global search using a massive box on the specified layer
        // Center at (0,0,0) with a huge extent.
        Collider[] hits = Physics.OverlapBox(Vector3.zero, Vector3.one * 100000f, Quaternion.identity, searchLayer);

        foreach (var hit in hits)
        {
            Transform agentTransform = hit.transform;

            // Skip if already tracked
            if (trackingData.ContainsKey(agentTransform)) continue;

            // Check if this object matches any of the prefabs
            if (IsMatchingPrefab(agentTransform.name))
            {
                AgentTrackingData newData = new AgentTrackingData();
                newData.agentCollider = hit; // Store the collider found
                trackingData.Add(agentTransform, newData);
            }
        }
    }

    private bool IsMatchingPrefab(string objectName)
    {
        foreach (var prefab in agentPrefabs)
        {
            if (prefab == null) continue;
            // Check if object name starts with prefab name (handles "Name" and "Name(Clone)")
            if (objectName.Contains(prefab.name))
            {
                return true;
            }
        }
        return false;
    }

    private void UpdateAgentStats()
    {
        foreach (var kvp in trackingData)
        {
            Transform agent = kvp.Key;
            AgentTrackingData data = kvp.Value;

            if (agent == null) continue; // Agent might be destroyed

            // Check if goal reached (only if not already reached)
            if (!data.hasReachedGoal)
            {
                // Try to get the component on the agent or its parents, including inactive ones
                AgentGBM gbmAgent = agent.GetComponent<AgentGBM>();
                if (gbmAgent == null)
                {
                    gbmAgent = agent.GetComponentsInParent<AgentGBM>(true).FirstOrDefault();
                }

                if (gbmAgent != null && gbmAgent.GoalReached)
                {
                    data.hasReachedGoal = true;
                    // Debug.Log($"[SocialTaskCompletionRate] Agent '{agent.name}' reached goal.");
                }
            }

            // If goal reached, stop tracking stats for this agent
            if (data.hasReachedGoal) continue;

            // Increment Total Time
            data.totalTime += Time.deltaTime;

            bool inDanger = IsInDangerZone(data.agentCollider);

            // Check if Agent is inside any Danger Zone
            if (inDanger)
            {
                data.dangerTime += Time.deltaTime;
            }

            // Record Trajectory
            if (enableTrajectoryMap)
            {
                Vector3 currentPos = agent.position;
                if (data.trajectory.Count == 0 || Vector3.Distance(data.trajectory[data.trajectory.Count - 1].position, currentPos) >= minRecordDistance)
                {
                    data.trajectory.Add(new TrajectoryPoint(currentPos, inDanger));
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
            if (col != null && col.enabled)
            {
                // 1. Fast AABB Check
                if (agentCollider.bounds.Intersects(col.bounds))
                {
                    // 2. Precise Collision Check
                    Vector3 direction;
                    float distance;
                    if (Physics.ComputePenetration(
                        agentCollider, agentCollider.transform.position, agentCollider.transform.rotation,
                        col, col.transform.position, col.transform.rotation,
                        out direction, out distance))
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /// <summary>
    /// Calculates the Social Task Completion Rate for a specific agent.
    /// Rate = 1.0 - (TimeInDanger / TotalTime).
    /// </summary>
    public float CalculateRate(AgentTrackingData data)
    {
        if (data.totalTime <= 0.0001f) return 1f; // Start with 100%

        float rate = 1.0f - (data.dangerTime / data.totalTime);
        return Mathf.Clamp01(rate);
    }

    private void OnApplicationQuit()
    {
        Debug.Log("=== Social Task Completion Rate Report ===");

        float totalRateSum = 0f;
        int count = 0;
        int successCount = 0;
        float successfulAgentsSafeRateSum = 0f;

        foreach (var kvp in trackingData)
        {
            Transform agent = kvp.Key;
            AgentTrackingData data = kvp.Value;
            string agentName = agent != null ? agent.name : "Destroyed Agent";

            float rate = CalculateRate(data);
            totalRateSum += rate;
            count++;

            Debug.Log($"Agent: {agentName} | Rate: {rate * 100:F2}% (Danger: {data.dangerTime:F2}s / Total: {data.totalTime:F2}s) | Goal Reached: {data.hasReachedGoal}");

            if (data.hasReachedGoal)
            {
                successCount++;
                successfulAgentsSafeRateSum += rate;
            }
        }

        if (count > 0)
        {
            Debug.Log($"Average Social Task Completion Rate: {(totalRateSum / count) * 100:F2}%");
            
            float successRate = (float)successCount / count;
            Debug.Log($"Success Rate: {successRate * 100:F2}% ({successCount}/{count})");

            if (successCount > 0)
            {
                Debug.Log($"Safe Rate (of successful agents): {(successfulAgentsSafeRateSum / successCount) * 100:F2}%");
            }
            else
            {
                Debug.Log("Safe Rate: N/A (No agents reached goal)");
            }
        }
        else
        {
            Debug.Log("No agents tracked.");
        }

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
        foreach (var col in dangerZoneColliders)
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

            DrawLine(texture, px1, px2, dangerZoneOutlineColor, 2);
            DrawLine(texture, px2, px3, dangerZoneOutlineColor, 2);
            DrawLine(texture, px3, px4, dangerZoneOutlineColor, 2);
            DrawLine(texture, px4, px1, dangerZoneOutlineColor, 2);
        }

        // 2. Draw Trajectories
        foreach (var kvp in trackingData)
        {
            List<TrajectoryPoint> path = kvp.Value.trajectory;
            if (path.Count < 2) continue;

            Vector2 prevPixel = WorldToPixel(path[0].position, minX, minZ);

            for (int i = 1; i < path.Count; i++)
            {
                Vector2 currentPixel = WorldToPixel(path[i].position, minX, minZ);

                // Color based on the status of the current point (or previous)
                // If the segment is "in danger", draw yellow.
                // We use the status of the point we are going TO, or FROM. 
                // Let's use logic: If I am standing in danger, the path I tread is danger.
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

