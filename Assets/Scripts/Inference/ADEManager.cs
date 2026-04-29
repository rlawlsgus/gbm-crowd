using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Globalization;
using UnityEngine;
using UnityEngine.SceneManagement;

public class ADEManager : MonoBehaviour
{
    public static ADEManager Instance;

    [Header("Agent Spawning")]
    [Tooltip("AgentGBM이 부착된 에이전트 프리펩")]
    public GameObject simAgentPrefab;
    public Transform simAgentsRoot;

    [Header("Ground Truth Files (Zara/ETH)")]
    public TextAsset trajectoriesTxt; // obsmat.txt
    public TextAsset homographyTxt;   // homography.txt
    public TextAsset groupsTxt;       // groups.txt

    [Header("World Mapping (Sync with ZaraSim)")]
    public bool useHomography = true;
    public bool flipX = true;
    public bool flipZ = false;
    public float worldScale = 1.0f;
    public Vector3 worldOffset = new Vector3(0, -0.45f, 0);

    [Header("Rotation (Sync with ZaraSim)")]
    public bool rotateInWorld = true;
    public float rotateDeg = 0f;
    public enum PivotMode { WorldOrigin, WorldOffset, DataCentroid, Custom }
    public PivotMode pivotMode = PivotMode.DataCentroid;
    public Vector3 customPivotWorld = Vector3.zero;

    [Header("Evaluation Settings")]
    [Tooltip("0.8초 기준 (25fps * 0.8s = 20 frames)")]
    public int evaluationInterval = 20;
    public string saveSubFolder = "MetricsLogs";
    public bool verbose = true;

    // 데이터 저장소
    private Dictionary<int, TrajectoryData> gtData = new Dictionary<int, TrajectoryData>();
    private Dictionary<int, List<int>> groupMap = new Dictionary<int, List<int>>();
    private Matrix4x4 homographyMatrix = Matrix4x4.identity;
    private Vector3 dataCentroidWorld = Vector3.zero;

    // 소환 관리
    private Dictionary<int, int> agentStartFrames = new Dictionary<int, int>();
    private Dictionary<int, AgentGBM> spawnedAgents = new Dictionary<int, AgentGBM>();

    // 지표 관리
    private Dictionary<int, AgentMetrics> agentMetricsMap = new Dictionary<int, AgentMetrics>();
    private List<float> allSegmentADEs = new List<float>();
    private List<float> allSegmentFDEs = new List<float>();

    private int lastProcessedFrame = -1;
    private bool hasExported = false;

    public class TrajectoryData { public Dictionary<int, Vector2> posByFrame = new Dictionary<int, Vector2>(); }

    [Serializable]
    public class FinalMetricsReport
    {
        public string sceneName;
        public string date;
        public string time;
        public int totalSegments;
        public float meanADE;
        public float stdDevADE;
        public float meanFDE;
        public float stdDevFDE;
    }

    public class AgentMetrics
    {
        public int id;
        public List<Vector3> gtPath = new List<Vector3>();
        public List<Vector3> simPath = new List<Vector3>();
        public int currentFrameCount = 0;

        public void Record(Vector3 gtPos, Vector3 simPos)
        {
            gtPath.Add(new Vector3(gtPos.x, 0, gtPos.z));
            simPath.Add(new Vector3(simPos.x, 0, simPos.z));
            currentFrameCount++;
        }

        public void Calculate()
        {
            if (gtPath.Count > 0)
            {
                float ade = 0;
                for (int i = 0; i < gtPath.Count; i++) ade += Vector3.Distance(gtPath[i], simPath[i]);
                ade /= gtPath.Count;
                
                float fde = Vector3.Distance(gtPath.Last(), simPath.Last());
                
                Instance.AddGlobalRecord(ade, fde);
                
                if (Instance.verbose) 
                    Debug.Log($"<color=#00FF00>[ADEManager]</color> Agent {id} Segment Finished | ADE: {ade:F4}, FDE: {fde:F4}");
            }
            
            gtPath.Clear(); 
            simPath.Clear(); 
            currentFrameCount = 0;
        }
    }

    void Awake()
    {
        if (Instance == null) Instance = this;
        else { Destroy(gameObject); return; }
        ParseAll();
    }

    void ParseAll()
    {
        ParseHomography();
        ParseGroups();
        ParseTrajectories();
        ComputeDataCentroid();
    }

    void ParseHomography()
    {
        if (homographyTxt == null) return;
        string[] lines = homographyTxt.text.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
        var inv = CultureInfo.InvariantCulture;
        for (int i = 0; i < 3 && i < lines.Length; i++)
        {
            string[] parts = lines[i].Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            for (int j = 0; j < 3 && j < parts.Length; j++)
                homographyMatrix[i, j] = float.Parse(parts[j], inv);
        }
        homographyMatrix[3, 3] = 1f;
    }

    void ParseGroups()
    {
        if (groupsTxt == null) return;
        string[] lines = groupsTxt.text.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
        foreach (var line in lines)
        {
            var parts = line.Split(new[] { ' ', '\t', ',' }, StringSplitOptions.RemoveEmptyEntries);
            var ids = parts.Select(p => int.Parse(p)).ToList();
            foreach (var id in ids) groupMap[id] = ids;
        }
    }

    void ParseTrajectories()
    {
        if (trajectoriesTxt == null) return;
        string[] lines = trajectoriesTxt.text.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
        var inv = CultureInfo.InvariantCulture;
        foreach (string line in lines)
        {
            string[] parts = line.Split(new[] { ' ', '\t', ',' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length < 4) continue;
            if (int.TryParse(parts[0], out int id) && float.TryParse(parts[1], NumberStyles.Any, inv, out float x) &&
                float.TryParse(parts[2], NumberStyles.Any, inv, out float y) && int.TryParse(parts[3], out int frame))
            {
                if (!gtData.ContainsKey(id)) gtData[id] = new TrajectoryData();
                gtData[id].posByFrame[frame] = new Vector2(x, y);

                if (!agentStartFrames.ContainsKey(id) || frame < agentStartFrames[id])
                    agentStartFrames[id] = frame;
            }
        }
    }

    void ComputeDataCentroid()
    {
        double sx = 0, sz = 0; long cnt = 0;
        foreach (var traj in gtData.Values)
        {
            foreach (var rawXY in traj.posByFrame.Values)
            {
                Vector2 mapped = ApplyHomography(rawXY);
                Vector3 p = MapToUnityXZ(mapped) * worldScale + worldOffset;
                sx += p.x; sz += p.z; cnt++;
            }
        }
        if (cnt > 0) dataCentroidWorld = new Vector3((float)(sx / cnt), 0, (float)(sz / cnt));
    }

    private float _accumulatedSimTime = 0f;

    void LateUpdate()
    {
        if (Time.timeScale > 0)
        {
            _accumulatedSimTime += Time.deltaTime;
        }

        int currentFrame = Mathf.FloorToInt(_accumulatedSimTime / 0.04f + 0.001f);

        if (currentFrame > lastProcessedFrame)
        {
            for (int f = lastProcessedFrame + 1; f <= currentFrame; f++)
            {
                CheckAndSpawnAgents(f);
                SampleAndEvaluate(f);
            }
            lastProcessedFrame = currentFrame;
        }
    }

    void CheckAndSpawnAgents(int frame)
    {
        if (simAgentPrefab == null) return;
        if (simAgentsRoot == null) simAgentsRoot = new GameObject("SimulatedAgents_Root").transform;

        Vector3 pivot = GetPivot();

        var pendingIds = agentStartFrames.Keys.ToList();
        foreach (int id in pendingIds)
        {
            int startFrame = agentStartFrames[id];

            if (frame >= startFrame && !spawnedAgents.ContainsKey(id))
            {
                if (!gtData[id].posByFrame.ContainsKey(startFrame)) continue;

                Vector2 rawXY = gtData[id].posByFrame[startFrame];
                Vector3 spawnPos = RawToWorld(rawXY, pivot);

                GameObject go = Instantiate(simAgentPrefab, spawnPos, Quaternion.identity, simAgentsRoot);
                go.name = $"ped_{id}_Sim";

                AgentGBM client = go.GetComponent<AgentGBM>();
                if (client != null)
                {
                    client.agentIndex = id;
                    int lastF = gtData[id].posByFrame.Keys.Max();
                    Vector3 goalPos = RawToWorld(gtData[id].posByFrame[lastF], pivot);

                    client.GoalPosition = goalPos;
                    spawnedAgents[id] = client;

                    if (groupMap.TryGetValue(id, out List<int> memberIds))
                    {
                        foreach (int mId in memberIds)
                        {
                            if (mId == id) continue;
                            if (spawnedAgents.TryGetValue(mId, out AgentGBM memberClient))
                            {
                                if (!client.groupMembers.Contains(memberClient.transform))
                                    client.groupMembers.Add(memberClient.transform);
                                
                                if (!memberClient.groupMembers.Contains(client.transform))
                                    memberClient.groupMembers.Add(client.transform);
                            }
                        }
                    }
                    GradientBasedModel.AddAgent(client);
                }
            }
        }
    }

    void SampleAndEvaluate(int frame)
    {
        Vector3 pivot = GetPivot();

        var activeIds = spawnedAgents.Keys.ToList();
        foreach (int id in activeIds)
        {
            AgentGBM agent = spawnedAgents[id];
            if (agent == null) continue;

            if (gtData[id].posByFrame.TryGetValue(frame, out var rawXY))
            {
                if (!agentMetricsMap.ContainsKey(id)) agentMetricsMap[id] = new AgentMetrics { id = id };

                Vector3 gtPos = RawToWorld(rawXY, pivot);
                
                // 1. Record Sim position BEFORE correction
                agentMetricsMap[id].Record(gtPos, agent.transform.position);

                // 2. If segment finished, calculate and SNAPPING TO GT
                if (agentMetricsMap[id].currentFrameCount >= evaluationInterval)
                {
                    agentMetricsMap[id].Calculate();
                    
                    // --- FORCE RESET TO GT POSITION ---
                    agent.transform.position = gtPos;
                    agent.Velocity = Vector3.zero; // Start next segment from static GT position
                    
                    if (verbose) Debug.Log($"[ADEManager] Agent {id} reset to GT at frame {frame}");
                }
            }
            else
            {
                // Trajectory ended
                if (agentMetricsMap.ContainsKey(id)) 
                {
                    // Calculate remaining segment if any
                    if (agentMetricsMap[id].currentFrameCount > 0)
                        agentMetricsMap[id].Calculate();
                }
                spawnedAgents.Remove(id);
                Destroy(agent.gameObject);
            }
        }
    }

    public Vector3 RawToWorld(Vector2 rawXY, Vector3 pivot)
    {
        Vector2 mapped = useHomography ? ApplyHomography(rawXY) : rawXY;
        Vector3 pos = MapToUnityXZ(mapped) * worldScale + worldOffset;
        if (rotateInWorld) pos = pivot + Quaternion.Euler(0, rotateDeg, 0) * (pos - pivot);
        return pos;
    }

    private Vector2 ApplyHomography(Vector2 uv)
    {
        float x = homographyMatrix[0, 0] * uv.x + homographyMatrix[0, 1] * uv.y + homographyMatrix[0, 2];
        float y = homographyMatrix[1, 0] * uv.x + homographyMatrix[1, 1] * uv.y + homographyMatrix[1, 2];
        float w = homographyMatrix[2, 0] * uv.x + homographyMatrix[2, 1] * uv.y + homographyMatrix[2, 2];
        return (Mathf.Abs(w) > 1e-5f) ? new Vector2(x / w, y / w) : uv;
    }

    private Vector3 MapToUnityXZ(Vector2 mapped)
    {
        return new Vector3(flipX ? -mapped.x : mapped.x, 0, flipZ ? -mapped.y : mapped.y);
    }

    private Vector3 GetPivot()
    {
        switch (pivotMode)
        {
            case PivotMode.WorldOrigin: return Vector3.zero;
            case PivotMode.WorldOffset: return worldOffset;
            case PivotMode.DataCentroid: return dataCentroidWorld;
            case PivotMode.Custom: return customPivotWorld;
            default: return worldOffset;
        }
    }

    public void AddGlobalRecord(float ade, float fde) { allSegmentADEs.Add(ade); allSegmentFDEs.Add(fde); }

    public void ExportToJson()
    {
        if (allSegmentADEs.Count == 0 || hasExported) return;
        hasExported = true;
        float meanADE = allSegmentADEs.Average(); float meanFDE = allSegmentFDEs.Average();
        FinalMetricsReport report = new FinalMetricsReport
        {
            sceneName = SceneManager.GetActiveScene().name,
            date = DateTime.Now.ToString("yyyy-MM-dd"),
            time = DateTime.Now.ToString("HH-mm-ss"),
            totalSegments = allSegmentADEs.Count,
            meanADE = meanADE,
            meanFDE = meanFDE,
            stdDevADE = CalculateStdDev(allSegmentADEs, meanADE),
            stdDevFDE = CalculateStdDev(allSegmentFDEs, meanFDE)
        };
        string path = Path.Combine(Application.dataPath, "..", saveSubFolder);
        if (!Directory.Exists(path)) Directory.CreateDirectory(path);
        File.WriteAllText(Path.Combine(path, $"{report.sceneName}_{report.date}_{report.time}_ADEMetrics.json"), JsonUtility.ToJson(report, true));
        Debug.Log($"[ADEManager] Exported Final ADE/FDE Results to {path}");
    }

    float CalculateStdDev(List<float> values, float mean)
    {
        if (values.Count <= 1) return 0;
        return (float)Math.Sqrt(values.Sum(v => Math.Pow(v - mean, 2)) / values.Count);
    }

    void OnApplicationQuit() { ExportToJson(); }
}
