using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

public class AgentSpawnManager : MonoBehaviour
{
    public static AgentSpawnManager Instance;

    [Header("Agent Filter")]
    public int startIdx = 0;
    public int finalIdx = 10000;

    [Header("Agent Prefab")]
    public GameObject simAgentPrefab;
    public Transform agentsRoot;

    [Header("Ground Truth Files (Zara/ETH)")]
    public TextAsset trajectoriesTxt; // obsmat.txt
    public TextAsset groupsTxt;       // groups.txt
    public TextAsset homographyTxt;   // homography.txt (optional)

    [Header("World Mapping")]
    public bool useHomography = true;
    public bool flipX = true;
    public bool flipZ = false;
    public float worldScale = 1.0f;
    public Vector3 worldOffset = new Vector3(0, -0.45f, 0);

    [Header("Rotation")]
    public bool rotateInWorld = true;
    public float rotateDeg = 0f;
    public enum PivotMode { WorldOrigin, WorldOffset, DataCentroid, Custom }
    public PivotMode pivotMode = PivotMode.DataCentroid;
    public Vector3 customPivotWorld = Vector3.zero;

    [Header("Playback Settings")]
    [Tooltip("Sampling interval in seconds (default 0.04s = 25fps)")]
    public float samplingInterval = 0.04f;
    public float playbackSpeed = 1.0f;

    // Data Storage
    private Dictionary<int, TrajectoryData> gtData = new Dictionary<int, TrajectoryData>();
    private Dictionary<int, List<int>> groupMap = new Dictionary<int, List<int>>();
    private Matrix4x4 homographyMatrix = Matrix4x4.identity;
    private Vector3 dataCentroidWorld = Vector3.zero;

    // Spawn Management
    private Dictionary<int, int> agentStartFrames = new Dictionary<int, int>();
    private Dictionary<int, GameObject> spawnedAgents = new Dictionary<int, GameObject>();
    private HashSet<int> _hasSpawnedHistory = new HashSet<int>();

    private int lastProcessedFrame = -1;
    private float _accumulatedSimTime = 0f;

    public class TrajectoryData { public Dictionary<int, Vector2> posByFrame = new Dictionary<int, Vector2>(); }

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
                // Agent Filter Apply
                if (id < startIdx || id > finalIdx) continue;

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

    void LateUpdate()
    {
        if (Time.timeScale > 0)
        {
            _accumulatedSimTime += Time.deltaTime * playbackSpeed;
        }

        int currentFrame = Mathf.FloorToInt(_accumulatedSimTime / samplingInterval + 0.001f);

        if (currentFrame > lastProcessedFrame)
        {
            for (int f = lastProcessedFrame + 1; f <= currentFrame; f++)
            {
                CheckAndSpawnAgents(f);
            }
            lastProcessedFrame = currentFrame;
        }
    }

    void CheckAndSpawnAgents(int frame)
    {
        if (simAgentPrefab == null) return;
        if (agentsRoot == null) agentsRoot = new GameObject("SimulatedAgents_Root").transform;

        Vector3 pivot = GetPivot();

        foreach (var id in agentStartFrames.Keys.ToList())
        {
            if (frame >= agentStartFrames[id] && !spawnedAgents.ContainsKey(id) && !_hasSpawnedHistory.Contains(id))
            {
                if (!gtData[id].posByFrame.ContainsKey(agentStartFrames[id])) continue;

                Vector2 rawXY = gtData[id].posByFrame[agentStartFrames[id]];
                Vector3 spawnPos = RawToWorld(rawXY, pivot);

                // Calculate initial rotation towards the second point if available, otherwise towards goal
                Quaternion spawnRot = Quaternion.identity;
                int secondFrame = gtData[id].posByFrame.Keys.Where(f => f > agentStartFrames[id]).OrderBy(f => f).FirstOrDefault();
                if (secondFrame != 0)
                {
                    Vector3 nextPos = RawToWorld(gtData[id].posByFrame[secondFrame], pivot);
                    Vector3 diff = nextPos - spawnPos;
                    if (diff.sqrMagnitude > 0.001f) spawnRot = Quaternion.LookRotation(diff);
                }
                else
                {
                    int lastF = gtData[id].posByFrame.Keys.Max();
                    Vector3 goalPos = RawToWorld(gtData[id].posByFrame[lastF], pivot);
                    Vector3 diff = goalPos - spawnPos;
                    if (diff.sqrMagnitude > 0.001f) spawnRot = Quaternion.LookRotation(diff);
                }

                GameObject go = Instantiate(simAgentPrefab, spawnPos, spawnRot, agentsRoot);
                go.name = $"ped_{id}_Sim";

                AgentGBM agentGBM = go.GetComponent<AgentGBM>();

                if (agentGBM != null)
                {
                    int lastF = gtData[id].posByFrame.Keys.Max();
                    Vector3 goalPos = RawToWorld(gtData[id].posByFrame[lastF], pivot);

                    // Optional: Create Goal Object for visualization
                    GameObject goalObj = new GameObject($"Goal_{id}_Sim");
                    goalObj.transform.position = goalPos;
                    goalObj.transform.SetParent(agentsRoot);

                    agentGBM.agentIndex = id;
                    agentGBM.pdmMode = false; // So it deactivates on goal in AgentGBM.Update()
                    agentGBM.GoalPosition = goalPos;
                    agentGBM.GoalReached = false;
                    agentGBM.Velocity = Vector3.zero;

                    // Add to the simulation model
                    GradientBasedModel.AddAgent(agentGBM);

                    spawnedAgents[id] = go;
                    _hasSpawnedHistory.Add(id);

                    // Set Group Members
                    if (groupMap.TryGetValue(id, out List<int> memberIds))
                    {
                        foreach (int mId in memberIds)
                        {
                            if (mId == id) continue;
                            if (spawnedAgents.TryGetValue(mId, out GameObject memberGo))
                            {
                                AgentGBM mGbm = memberGo.GetComponent<AgentGBM>();

                                if (agentGBM != null) agentGBM.groupMembers.Add(memberGo.transform);
                                if (mGbm != null) mGbm.groupMembers.Add(go.transform);
                            }
                        }
                    }
                }
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
}
