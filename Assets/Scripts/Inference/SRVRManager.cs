using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.IO;
using UnityEngine.SceneManagement;

[Serializable]
public class SRVRMetricsReport
{
    public string sceneName;
    public string date;
    public string time;
    public int totalAgents;
    public int successCount;
    public float goalSuccessRate;
    public float averageSafeRate;
    public float avgDangerZoneViolationRate;
    public float avgStaticObstacleCollisionRate;
    public float avgVehicleCollisionRate;
    public float avgDoorCollisionRate;
    public float avgAgentCollisionRate;
    public float avgTravelTime;
    public float stdDevTravelTime;
    public int totalGroupsDetected;
    public float avgInterGroupDistance;
}

public class SRVRManager : MonoBehaviour
{
    public static SRVRManager Instance { get; private set; }

    [Header("Agent Settings")]
    [Tooltip("Interval in seconds to search for new agents.")]
    public float searchInterval = 0.5f;
    public float agentRadius = 0.4f;

    [Header("Environment Settings")]
    public Transform dangerZonesRoot;
    public Transform obstaclesRoot;

    [Header("Logging Settings")]
    public string saveSubFolder = "MetricsLogs";
    public bool verbose = true;
    public bool logHitTags = false;
    private bool _hasExported = false;

    [Header("Trajectory Map Settings")]
    public bool enableTrajectoryMap = true;
    [Tooltip("If true, only draws the last N seconds of the trajectory.")]
    public bool drawOnlyTail = false;
    [Tooltip("Seconds of history to draw if 'drawOnlyTail' is true.")]
    public float trajectoryTailSeconds = 5.0f;
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

    public struct TrajectoryPoint
    {
        public Vector3 position;
        public bool isDanger;
        public float timestamp;

        public TrajectoryPoint(Vector3 pos, bool danger, float time)
        {
            position = pos;
            isDanger = danger;
            timestamp = time;
        }
    }

    public class AgentTrackingData
    {
        public float totalTime;
        public float dangerZoneTime;
        public float staticObstacleTime;
        public float vehicleCollisionTime;
        public float doorCollisionTime;
        public float agentCollisionTime;

        public Collider agentCollider;
        public AgentGBM agentComponent;
        public bool hasReachedGoal;
        public bool isFinished;

        public List<TrajectoryPoint> trajectory = new List<TrajectoryPoint>();

        // Group Stats
        public float groupDistanceSum;
        public int groupDistanceSamples;
    }

    private Dictionary<Transform, AgentTrackingData> trackingData = new Dictionary<Transform, AgentTrackingData>();
    private HashSet<Collider> dangerZoneColliders = new HashSet<Collider>();
    private HashSet<Collider> obstacleColliders = new HashSet<Collider>();
    private float searchTimer = 0f;

    [Header("Detection Tags")]
    public string agentTag = "AgentGBM";
    public string obstacleTag = "Obstacle";
    public string buildingTag = "Building";
    public string vehicleTag = "Vehicle";
    public string doorTag = "Door";

    void Awake()
    {
        if (Instance == null) Instance = this;
        else Destroy(gameObject);
    }

    void Start()
    {
        if (dangerZonesRoot == null) dangerZonesRoot = GameObject.Find("Danger Zones")?.transform;
        if (dangerZonesRoot != null)
            dangerZoneColliders = new HashSet<Collider>(dangerZonesRoot.GetComponentsInChildren<Collider>());

        if (obstaclesRoot == null) obstaclesRoot = GameObject.Find("Obstacles")?.transform;
        if (obstaclesRoot != null)
            obstacleColliders = new HashSet<Collider>(obstaclesRoot.GetComponentsInChildren<Collider>());

        if (verbose) Debug.Log($"[SRVR] Initialized: {dangerZoneColliders.Count} DangerZones, {obstacleColliders.Count} Obstacles collected.");
    }

    void Update()
    {
        searchTimer += Time.unscaledDeltaTime;
        if (searchTimer >= searchInterval)
        {
            SearchForAgents();
            searchTimer = 0f;
        }

        Physics.SyncTransforms();
        UpdateAgentStats();
    }

    private void SearchForAgents()
    {
        AgentGBM[] foundAgents = FindObjectsOfType<AgentGBM>(true);
        foreach (var agent in foundAgents)
        {
            if (agent == null) continue;
            Transform t = agent.transform;
            if (trackingData.ContainsKey(t)) continue;

            // 자식 오브젝트에서 Collider와 Rigidbody를 우선적으로 찾습니다.
            Collider col = t.GetComponentInChildren<Collider>();
            Rigidbody rb = t.GetComponentInChildren<Rigidbody>();

            if (col == null)
            {
                SphereCollider sc = agent.gameObject.AddComponent<SphereCollider>();
                sc.radius = agentRadius;
                sc.isTrigger = true;
                col = sc;
            }

            if (rb == null)
            {
                // Rigidbody가 없으면 충돌 감지 대상 오브젝트(col이 있는 곳)에 추가합니다.
                rb = col.gameObject.AddComponent<Rigidbody>();
                rb.isKinematic = true;
                rb.useGravity = false;
            }

            AgentTrackingData newData = new AgentTrackingData();
            newData.agentComponent = agent;
            newData.agentCollider = col;
            trackingData.Add(t, newData);
        }
    }

    private void UpdateAgentStats()
    {
        float dt = Time.deltaTime;
        if (dt <= 0) return;

        foreach (var kvp in trackingData)
        {
            Transform agentTransform = kvp.Key;
            AgentTrackingData data = kvp.Value;

            if (agentTransform == null || data.isFinished) continue;

            if (data.agentComponent != null && data.agentComponent.GoalReached)
            {
                data.hasReachedGoal = true;
                data.isFinished = true;
                continue;
            }

            if (!agentTransform.gameObject.activeInHierarchy) continue;

            data.totalTime += dt;

            // 에이전트 콜라이더의 실제 월드 위치
            Vector3 agentColPos = data.agentCollider.transform.position;

            // 1. Danger Zone Check
            bool hitDanger = false;
            foreach (var col in dangerZoneColliders)
            {
                if (col != null && col.enabled && IsOverlapping(data.agentCollider, col))
                {
                    hitDanger = true;
                    break;
                }
            }
            if (hitDanger) data.dangerZoneTime += dt;

            // 2. Collision Check (Nearby Objects)
            Collider[] nearby = Physics.OverlapSphere(agentColPos, agentRadius + 0.2f);
            foreach (var col in nearby)
            {
                if (col == null || col == data.agentCollider || col.transform.IsChildOf(agentTransform)) continue;

                if (IsOverlapping(data.agentCollider, col))
                {
                    string tag = col.tag;
                    if (logHitTags) Debug.Log($"Agent {data.agentComponent.agentIndex} hitting: {tag} on {col.name}");

                    if (tag == agentTag) data.agentCollisionTime += dt;
                    else if (tag == vehicleTag) data.vehicleCollisionTime += dt;
                    else if (tag == doorTag) data.doorCollisionTime += dt;
                    else if (tag == obstacleTag || tag == buildingTag || obstacleColliders.Contains(col))
                        data.staticObstacleTime += dt;
                }
            }

            // Group Stats
            if (data.agentComponent != null && data.agentComponent.groupMembers != null && data.agentComponent.groupMembers.Count > 0)
            {
                float distSum = 0f;
                int count = 0;
                foreach (var member in data.agentComponent.groupMembers)
                {
                    if (member != null && member.gameObject.activeInHierarchy && member != agentTransform)
                    {
                        distSum += Vector3.Distance(agentTransform.position, member.position);
                        count++;
                    }
                }

                if (count > 0)
                {
                    data.groupDistanceSum += (distSum / count);
                    data.groupDistanceSamples++;
                }
            }

            if (enableTrajectoryMap)
            {
                Vector3 currentPos = agentTransform.position;
                if (data.trajectory.Count == 0 ||
                    Vector3.Distance(data.trajectory[data.trajectory.Count - 1].position, currentPos) >= minRecordDistance)
                {
                    data.trajectory.Add(new TrajectoryPoint(currentPos, hitDanger, Time.time));
                }
            }
        }
    }

    private bool IsOverlapping(Collider agentCol, Collider otherCol)
    {
        if (agentCol == null || otherCol == null) return false;
        if (!agentCol.bounds.Intersects(otherCol.bounds)) return false;

        // 콜라이더의 실제 월드 위치와 회전
        Vector3 agentPos = agentCol.transform.position;
        Quaternion agentRot = agentCol.transform.rotation;

        if (otherCol is MeshCollider && !((MeshCollider)otherCol).convex)
        {
            Vector3 closestPoint = otherCol.ClosestPoint(agentPos);
            float dist = Vector3.Distance(agentPos, closestPoint);
            return dist <= agentRadius;
        }

        Vector3 dir; float distPen;
        return Physics.ComputePenetration(
            agentCol, agentPos, agentRot,
            otherCol, otherCol.transform.position, otherCol.transform.rotation,
            out dir, out distPen
        );
    }

    public float CalculateSafeRate(AgentTrackingData data)
    {
        if (data.totalTime <= 0.0001f) return 1f;
        float combinedUnsafeTime = data.dangerZoneTime + data.staticObstacleTime + data.vehicleCollisionTime + data.doorCollisionTime + data.agentCollisionTime;
        return Mathf.Clamp01(1.0f - (combinedUnsafeTime / data.totalTime));
    }

    private void OnApplicationQuit()
    {
        ExportResults();
        if (enableTrajectoryMap) GenerateTrajectoryMap();
    }

    private void ExportResults()
    {
        if (trackingData.Count == 0 || _hasExported) return;
        _hasExported = true;

        int totalAgents = trackingData.Count;
        int successCount = 0;

        List<float> successSafeRates = new List<float>();
        List<float> dangerZoneRates = new List<float>();
        List<float> staticObstacleRates = new List<float>();
        List<float> vehicleCollisionRates = new List<float>();
        List<float> doorCollisionRates = new List<float>();
        List<float> agentCollisionRates = new List<float>();

        foreach (var data in trackingData.Values)
        {
            if (data.hasReachedGoal)
            {
                successCount++;
                if (data.totalTime > 0.0001f)
                {
                    successSafeRates.Add(CalculateSafeRate(data));
                    dangerZoneRates.Add(data.dangerZoneTime / data.totalTime);
                    staticObstacleRates.Add(data.staticObstacleTime / data.totalTime);
                    vehicleCollisionRates.Add(data.vehicleCollisionTime / data.totalTime);
                    doorCollisionRates.Add(data.doorCollisionTime / data.totalTime);
                    agentCollisionRates.Add(data.agentCollisionTime / data.totalTime);
                }
            }
        }

        float gsr = totalAgents > 0 ? (float)successCount / totalAgents : 0f;
        float avgSafeRate = successSafeRates.Count > 0 ? successSafeRates.Average() : 0f;
        float avgDangerZoneRate = dangerZoneRates.Count > 0 ? dangerZoneRates.Average() : 0f;
        float avgStaticRate = staticObstacleRates.Count > 0 ? staticObstacleRates.Average() : 0f;
        float avgVehicleRate = vehicleCollisionRates.Count > 0 ? vehicleCollisionRates.Average() : 0f;
        float avgDoorRate = doorCollisionRates.Count > 0 ? doorCollisionRates.Average() : 0f;
        float avgAgentCollRate = agentCollisionRates.Count > 0 ? agentCollisionRates.Average() : 0f;

        float avgTime = 0f;
        float stdDevTime = 0f;
        var successAgents = trackingData.Values.Where(d => d.hasReachedGoal).ToList();
        if (successAgents.Count > 0)
        {
            avgTime = successAgents.Average(d => d.totalTime);
            double sumSqDiff = successAgents.Sum(d => Mathf.Pow(d.totalTime - avgTime, 2));
            stdDevTime = (float)((successAgents.Count > 1) ? System.Math.Sqrt(sumSqDiff / (successAgents.Count - 1)) : 0.0);
        }

        int groupCount = 0;
        HashSet<Transform> visitedForGroupCount = new HashSet<Transform>();
        List<float> allGroupAvgDistances = new List<float>();

        foreach (var kvp in trackingData)
        {
            Transform t = kvp.Key;
            AgentTrackingData d = kvp.Value;
            if (d.groupDistanceSamples > 0)
                allGroupAvgDistances.Add(d.groupDistanceSum / d.groupDistanceSamples);

            if (d.agentComponent != null && d.agentComponent.groupMembers != null && d.agentComponent.groupMembers.Count > 0)
            {
                if (!visitedForGroupCount.Contains(t))
                {
                    groupCount++;
                    visitedForGroupCount.Add(t);
                    foreach (var m in d.agentComponent.groupMembers)
                        if (m != null) visitedForGroupCount.Add(m);
                }
            }
        }
        float finalAvgGroupDist = (allGroupAvgDistances.Count > 0) ? allGroupAvgDistances.Average() : 0f;

        SRVRMetricsReport report = new SRVRMetricsReport
        {
            sceneName = SceneManager.GetActiveScene().name,
            date = DateTime.Now.ToString("yyyy-MM-dd"),
            time = DateTime.Now.ToString("HH-mm-ss"),
            totalAgents = totalAgents,
            successCount = successCount,
            goalSuccessRate = gsr,
            averageSafeRate = avgSafeRate,
            avgDangerZoneViolationRate = avgDangerZoneRate,
            avgStaticObstacleCollisionRate = avgStaticRate,
            avgVehicleCollisionRate = avgVehicleRate,
            avgDoorCollisionRate = avgDoorRate,
            avgAgentCollisionRate = avgAgentCollRate,
            avgTravelTime = avgTime,
            stdDevTravelTime = stdDevTime,
            totalGroupsDetected = groupCount,
            avgInterGroupDistance = finalAvgGroupDist
        };

        if (verbose)
        {
            Debug.Log("<color=yellow><b>=== SRVR (Social Region Violation Rate) Final Report ===</b></color>");
            Debug.Log($"<color=white><b>[Summary]</b></color> GSR: {gsr * 100:F2}% ({successCount}/{totalAgents})");
            Debug.Log($"<color=white><b>[Safety (Successes Only)]</b></color> Avg Safe Rate: {avgSafeRate * 100:F2}%");
            Debug.Log($"   - Danger Zone Violation Rate: {avgDangerZoneRate * 100:F2}%");
            Debug.Log($"   - Static Obstacle Collision Rate: {avgStaticRate * 100:F2}%");
            Debug.Log($"   - Vehicle/Train Collision Rate: {avgVehicleRate * 100:F2}%");
            Debug.Log($"   - Door Collision Rate: {avgDoorRate * 100:F2}%");
            Debug.Log($"   - Agent Collision Rate: {avgAgentCollRate * 100:F2}%");

            if (successCount > 0)
            {
                Debug.Log($"<color=white><b>[Time]</b></color> Avg Travel Time: {avgTime:F2}s (StdDev: {stdDevTime:F2}s)");
            }
            else
            {
                Debug.Log("<color=white><b>[Time]</b></color> No agents reached the goal.");
            }

            Debug.Log($"<color=white><b>[Groups]</b></color> Total Groups Detected: {groupCount}");
            Debug.Log($"<color=white><b>[Groups]</b></color> Avg Inter-Group Distance: {finalAvgGroupDist:F2}m");
            Debug.Log("<color=yellow><b>================================================</b></color>");
        }

        string folderPath = Path.Combine(Application.dataPath, "..", saveSubFolder);
        if (!Directory.Exists(folderPath)) Directory.CreateDirectory(folderPath);

        string fileName = $"{report.sceneName}_{report.date}_{report.time}_SRVRMetrics.json";
        string fullPath = Path.Combine(folderPath, fileName);

        try
        {
            File.WriteAllText(fullPath, JsonUtility.ToJson(report, true));
            if (verbose) Debug.Log($"[SRVRManager] Exported SRVR Metrics to {fullPath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[SRVRManager] Failed to export metrics: {e.Message}");
        }
    }

    private void GenerateTrajectoryMap()
    {
        Texture2D texture = new Texture2D(mapResolution, mapResolution);
        Color[] resetColor = new Color[mapResolution * mapResolution];
        for (int i = 0; i < resetColor.Length; i++) resetColor[i] = Color.white;
        texture.SetPixels(resetColor);

        float minX = transform.position.x - mapWidth / 2f;
        float minZ = transform.position.z - mapHeight / 2f;
        float currentTime = Time.time;
        float tailThreshold = (drawOnlyTail && trajectoryTailSeconds > 0f) ? currentTime - trajectoryTailSeconds : -1f;

        foreach (var col in dangerZoneColliders)
        {
            if (col != null && col.enabled) DrawCollider(texture, col, dangerZoneOutlineColor, minX, minZ);
        }
        foreach (var col in obstacleColliders)
        {
            if (col != null && col.enabled) DrawCollider(texture, col, obstacleOutlineColor, minX, minZ);
        }

        foreach (var data in trackingData.Values)
        {
            List<TrajectoryPoint> path = data.trajectory;
            if (path.Count < 2) continue;

            int startIndex = 0;
            if (tailThreshold > 0)
            {
                for (int i = 0; i < path.Count; i++)
                {
                    if (path[i].timestamp >= tailThreshold)
                    {
                        startIndex = i;
                        break;
                    }
                }
            }

            if (startIndex >= path.Count - 1) continue;

            Vector2 prevPixel = WorldToPixel(path[startIndex].position, minX, minZ);
            for (int i = startIndex + 1; i < path.Count; i++)
            {
                Vector2 currentPixel = WorldToPixel(path[i].position, minX, minZ);
                Color color = path[i].isDanger ? dangerPathColor : normalPathColor;
                DrawLine(texture, prevPixel, currentPixel, color, 1);
                prevPixel = currentPixel;
            }

            DrawCircle(texture, prevPixel, endPointRadius, normalPathColor);
        }

        texture.Apply();
        byte[] bytes = texture.EncodeToPNG();
        string pathToFile = Path.Combine(Application.dataPath, mapFileName);
        File.WriteAllBytes(pathToFile, bytes);
        Debug.Log($"[SRVRManager] Map saved to: {pathToFile}");
    }

    private void DrawCollider(Texture2D texture, Collider col, Color color, float minX, float minZ)
    {
        if (col is BoxCollider boxCol)
        {
            Transform t = col.transform;
            Vector3 c = boxCol.center;
            Vector3 s = boxCol.size;
            Vector3 p1 = t.TransformPoint(c + new Vector3(-s.x, -s.y, -s.z) * 0.5f);
            Vector3 p2 = t.TransformPoint(c + new Vector3(s.x, -s.y, -s.z) * 0.5f);
            Vector3 p3 = t.TransformPoint(c + new Vector3(s.x, -s.y, s.z) * 0.5f);
            Vector3 p4 = t.TransformPoint(c + new Vector3(-s.x, -s.y, s.z) * 0.5f);
            Vector2 px1 = WorldToPixel(p1, minX, minZ);
            Vector2 px2 = WorldToPixel(p2, minX, minZ);
            Vector2 px3 = WorldToPixel(p3, minX, minZ);
            Vector2 px4 = WorldToPixel(p4, minX, minZ);
            DrawLine(texture, px1, px2, color, 2);
            DrawLine(texture, px2, px3, color, 2);
            DrawLine(texture, px3, px4, color, 2);
            DrawLine(texture, px4, px1, color, 2);
        }
        else
        {
            Bounds b = col.bounds;
            Vector3 p1 = new Vector3(b.min.x, b.center.y, b.min.z);
            Vector3 p2 = new Vector3(b.max.x, b.center.y, b.min.z);
            Vector3 p3 = new Vector3(b.max.x, b.center.y, b.max.z);
            Vector3 p4 = new Vector3(b.min.x, b.center.y, b.max.z);
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
        int x0 = (int)p1.x, y0 = (int)p1.y, x1 = (int)p2.x, y1 = (int)p2.y;
        int dx = Mathf.Abs(x1 - x0), dy = Mathf.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        while (true)
        {
            for (int i = -thickness / 2; i <= thickness / 2; i++)
            {
                for (int j = -thickness / 2; j <= thickness / 2; j++)
                {
                    int px = x0 + i, py = y0 + j;
                    if (px >= 0 && px < tex.width && py >= 0 && py < tex.height) tex.SetPixel(px, py, col);
                }
            }
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    }

    private void DrawCircle(Texture2D tex, Vector2 center, int radius, Color col)
    {
        int cx = (int)center.x, cy = (int)center.y;
        for (int x = -radius; x <= radius; x++)
        {
            for (int y = -radius; y <= radius; y++)
            {
                if (x * x + y * y <= radius * radius)
                {
                    int px = cx + x, py = cy + y;
                    if (px >= 0 && px < tex.width && py >= 0 && py < tex.height) tex.SetPixel(px, py, col);
                }
            }
        }
    }
}
