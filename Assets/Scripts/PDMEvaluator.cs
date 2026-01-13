using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;

public class PDMEvaluator : MonoBehaviour
{
    [Header("Setup")]
    [Tooltip("Prefab for the Shadow Agent (Must have AgentGBM or AgentPRM component).")]
    public GameObject agentPrefab; 
    public ZaraGroupRotationSimulator zaraSimulator;
    public float resetInterval = 0.8f; // Reset every 0.8s as requested
    
    [Header("Map Generation")]
    public bool generateMap = true;
    public string mapFileName = "PDMTrajectoryMap.png";
    public int mapResolution = 2048;
    public float mapSize = 100f;
    public Color gtColor = Color.blue;
    public Color agentColor = Color.red;
    public Color obstacleColor = new Color(1f, 0.5f, 0f); // Orange

    // Obstacles
    private List<BoxCollider> obstacleColliders = new List<BoxCollider>();

    // Internal tracking
    private class AgentPair
    {
        public Transform gtTransform;
        public AgentBase shadowAgent;
        public GameObject shadowAgentObj;
        
        public List<Vector3> gtPath = new List<Vector3>();
        public List<Vector3> shadowPath = new List<Vector3>();
        public Vector3 prevGtPos; 
        
        // Per-interval tracking
        public float currentIntervalDiffSum;
        public int currentIntervalFrameCount;
    }

    private Dictionary<int, AgentPair> pairs = new Dictionary<int, AgentPair>();
    private float resetTimer = 0f;
    
    // Metrics
    private List<float> adeList = new List<float>(); // Average Displacement Error per interval
    private List<float> fdeList = new List<float>(); // Final Displacement Error per interval

    IEnumerator Start()
    {
        // 1. Find Obstacles (Static environment)
        GameObject obsObj = GameObject.Find("Obstacles");
        if (obsObj != null)
        {
            obstacleColliders = obsObj.GetComponentsInChildren<BoxCollider>().ToList();
        }
        else
        {
            var allObs = FindObjectsOfType<Obstacle>();
            foreach(var obs in allObs)
            {
                var col = obs.GetComponent<BoxCollider>();
                if (col != null) obstacleColliders.Add(col);
            }
        }

        // 2. Ensure Zara Simulator is linked
        if (zaraSimulator == null)
        {
            zaraSimulator = FindObjectOfType<ZaraGroupRotationSimulator>();
            if (zaraSimulator == null)
            {
                Debug.LogError("[PDMEvaluator] ZaraGroupRotationSimulator not found! Cannot fetch GT data.");
                yield break;
            }
        }
        
        // 3. Ensure a Vision Simulator (GBM/PRM) exists for Shadow Agents
        if (FindObjectOfType<SimulatorBase>() == null)
        {
            Debug.LogError("[PDMEvaluator] No SimulatorBase (GradientBasedModel or PureReactiveModel) found! Shadow Agents will not move correctly.");
        }

        yield return null; 
        
        Debug.Log($"[PDMEvaluator] Started. Reset Interval: {resetInterval}s");
    }

    // Sync in LateUpdate to match Zara's Update cycle (Zara moves agents in Update)
    void LateUpdate()
    {
        if (zaraSimulator != null)
        {
            SyncAgents();
        }
    }

    void SyncAgents()
    {
        // Direct access to Zara's agents
        var zaraAgents = zaraSimulator.GetAllAgents();
        
        foreach (var kv in zaraAgents)
        {
            int id = kv.Key;
            GameObject gtObj = kv.Value;
            bool gtActive = gtObj.activeSelf; 

            if (gtActive)
            {
                if (!pairs.ContainsKey(id))
                {
                    // Case 1: New Agent -> Create Shadow immediately
                    CreatePair(id, gtObj.transform);
                }
                else
                {
                    // Case 2: Existing Agent -> Ensure Shadow is active if GT is active
                    var pair = pairs[id];
                    if (!pair.shadowAgentObj.activeSelf)
                    {
                        pair.shadowAgentObj.SetActive(true);
                        // Force reset on re-activation to avoid jumps
                        ForceReset(pair.shadowAgent, gtObj.transform.position, gtObj.transform.rotation * Quaternion.Euler(0, -90, 0));
                        // Reset accumulators for the new active period
                        pair.currentIntervalDiffSum = 0f;
                        pair.currentIntervalFrameCount = 0;
                    }
                }
            }
            else
            {
                // GT Inactive
                if (pairs.ContainsKey(id))
                {
                    var pair = pairs[id];
                    if (pair.shadowAgentObj.activeSelf)
                    {
                        pair.shadowAgentObj.SetActive(false);
                    }
                }
            }
        }
    }

    void CreatePair(int id, Transform gtTransform)
    {
        // Spawn Shadow Agent at GT position with -90 degree adjustment
        Quaternion adjustedRot = gtTransform.rotation * Quaternion.Euler(0, -90, 0);
        GameObject shadowObj = Instantiate(agentPrefab, gtTransform.position, adjustedRot);
        shadowObj.name = $"Shadow_Agent_{id}";
        
        shadowObj.layer = LayerMask.NameToLayer("Obstacle");
        
        foreach(Transform child in shadowObj.transform)
        {
            child.gameObject.layer = shadowObj.layer;
        }

        AgentBase shadowAgent = shadowObj.GetComponent<AgentBase>();
        if (shadowAgent == null)
        {
            Debug.LogError("Agent Prefab missing AgentBase component!");
            return;
        }

        Vector3 finalGoal = zaraSimulator.GetFinalWorldPosition(id);
        shadowAgent.GoalPosition = finalGoal;
        shadowAgent.pdmMode = true; // Set PDM mode to prevent auto-disable
        
        if (shadowAgent is AgentGBM gbmAgent)
        {
            GradientBasedModel.AddAgent(gbmAgent);
        }
        else if (shadowAgent is AgentPRM prmAgent)
        {
            PureReactiveModel.AddAgent(prmAgent);
        }
        
        AgentPair pair = new AgentPair
        {
            gtTransform = gtTransform,
            shadowAgent = shadowAgent,
            shadowAgentObj = shadowObj,
            prevGtPos = gtTransform.position
        };

        ForceReset(shadowAgent, gtTransform.position, adjustedRot);
        
        pairs.Add(id, pair);
    }

    void FixedUpdate()
    {
        if (pairs.Count == 0) return;

        resetTimer += Time.fixedDeltaTime;
        bool isResetFrame = false;

        if (resetTimer >= resetInterval)
        {
            isResetFrame = true;
            resetTimer = 0f;
        }

        foreach (var pair in pairs.Values)
        {
            if (pair.gtTransform == null) continue;
            
            // Note: Sync is handled in LateUpdate, so we just check if it's currently active in hierarchy
            if (!pair.gtTransform.gameObject.activeInHierarchy) continue; 

            Vector3 currentGtPos = pair.gtTransform.position;
            
            // Only update prevPos here for delta calculations if needed
            pair.prevGtPos = currentGtPos;

            Vector3 gtPos = currentGtPos;
            Vector3 shadowPos = pair.shadowAgentObj.transform.position;
            float dist = Vector3.Distance(new Vector3(gtPos.x, 0, gtPos.z), new Vector3(shadowPos.x, 0, shadowPos.z));

            pair.currentIntervalDiffSum += dist;
            pair.currentIntervalFrameCount++;
            
            if (isResetFrame)
            {
                // --- Metrics Calculation ---
                fdeList.Add(dist);

                if (pair.currentIntervalFrameCount > 0)
                {
                    float avgIntervalDiff = pair.currentIntervalDiffSum / pair.currentIntervalFrameCount;
                    adeList.Add(avgIntervalDiff);
                }

                // Reset accumulators
                pair.currentIntervalDiffSum = 0f;
                pair.currentIntervalFrameCount = 0;

                // --- Reset Agent ---
                ForceReset(pair.shadowAgent, currentGtPos, pair.gtTransform.rotation * Quaternion.Euler(0, -90, 0));
                
                if (generateMap)
                {
                    pair.shadowPath.Add(Vector3.negativeInfinity);
                }
            }
            // If NOT reset frame, just record path
            else if (generateMap)
            {
                RecordPath(pair.gtPath, gtPos);
                RecordPath(pair.shadowPath, shadowPos);
            }
        }
    }

    void ForceReset(AgentBase agent, Vector3 pos, Quaternion rot)
    {
        agent.transform.position = pos;
        agent.transform.rotation = rot;
        agent.Velocity = Vector3.zero;
        
        // Reset specific flags
        if (agent is AgentGBM gbm)
        {
            gbm.GoalReached = false;
        }
        // Ensure active if it was disabled by logic
        if (!agent.gameObject.activeSelf) agent.gameObject.SetActive(true);
    }

    void RecordPath(List<Vector3> path, Vector3 currentPos)
    {
        Vector3 flatPos = new Vector3(currentPos.x, 0, currentPos.z);
        
        if (path.Count == 0)
        {
            path.Add(flatPos);
            return;
        }

        Vector3 lastPoint = path[path.Count - 1];
        
        if (float.IsNegativeInfinity(lastPoint.x))
        {
            path.Add(flatPos);
            return;
        }

        if (Vector3.Distance(lastPoint, flatPos) > 0.05f) // Slightly tighter threshold
        {
            path.Add(flatPos);
        }
    }

    void OnApplicationQuit()
    {
        if (adeList.Count > 0)
        {
            Debug.Log($"[PDM Report] ADE (Average Displacement Error): {adeList.Average():F4} meters (over {adeList.Count} intervals)");
        }
        
        if (fdeList.Count > 0)
        {
            Debug.Log($"[PDM Report] FDE (Final Displacement Error): {fdeList.Average():F4} meters (over {fdeList.Count} intervals)");
        }

        if (generateMap)
        {
            GenerateMap();
        }
    }

    void GenerateMap()
    {
        Texture2D texture = new Texture2D(mapResolution, mapResolution);
        Color[] resetColor = new Color[mapResolution * mapResolution];
        for (int i = 0; i < resetColor.Length; i++) resetColor[i] = Color.white;
        texture.SetPixels(resetColor);

        float minX = transform.position.x - mapSize / 2f;
        float minZ = transform.position.z - mapSize / 2f;

        DrawColliders(texture, obstacleColliders, obstacleColor, minX, minZ);

        foreach (var pair in pairs.Values)
        {
            DrawPath(texture, pair.gtPath, gtColor, minX, minZ);
            DrawPath(texture, pair.shadowPath, agentColor, minX, minZ);
            DrawEndPoint(texture, pair.gtPath, gtColor, minX, minZ);
            DrawEndPoint(texture, pair.shadowPath, agentColor, minX, minZ);
        }

        texture.Apply();
        byte[] bytes = texture.EncodeToPNG();
        string path = Path.Combine(Application.dataPath, mapFileName);
        File.WriteAllBytes(path, bytes);
        Debug.Log($"PDM Map saved to {path}");
    }

    void DrawColliders(Texture2D texture, List<BoxCollider> colliders, Color color, float minX, float minZ)
    {
        foreach (var col in colliders)
        {
            if (col == null) continue;
            Transform t = col.transform;
            Vector3 center = col.center;
            Vector3 size = col.size;
            Vector3 p1 = t.TransformPoint(center + new Vector3(-size.x, -size.y, -size.z) * 0.5f);
            Vector3 p2 = t.TransformPoint(center + new Vector3(size.x, -size.y, -size.z) * 0.5f);
            Vector3 p3 = t.TransformPoint(center + new Vector3(size.x, -size.y, size.z) * 0.5f);
            Vector3 p4 = t.TransformPoint(center + new Vector3(-size.x, -size.y, size.z) * 0.5f);
            Vector2 px1 = WorldToPixel(p1, minX, minZ);
            Vector2 px2 = WorldToPixel(p2, minX, minZ);
            Vector2 px3 = WorldToPixel(p3, minX, minZ);
            Vector2 px4 = WorldToPixel(p4, minX, minZ);
            DrawLine(texture, px1, px2, color);
            DrawLine(texture, px2, px3, color);
            DrawLine(texture, px3, px4, color);
            DrawLine(texture, px4, px1, color);
        }
    }

    void DrawEndPoint(Texture2D tex, List<Vector3> path, Color col, float minX, float minZ)
    {
        if (path == null || path.Count == 0) return;
        Vector3 last = path.LastOrDefault(p => !float.IsNegativeInfinity(p.x));
        if (!float.IsNegativeInfinity(last.x))
        {
            Vector2 pixel = WorldToPixel(last, minX, minZ);
            DrawCircle(tex, (int)pixel.x, (int)pixel.y, 5, col); 
        }
    }

    void DrawPath(Texture2D tex, List<Vector3> path, Color col, float minX, float minZ)
    {
        if (path.Count < 2) return;
        Vector2 prev = Vector2.zero;
        bool hasPrev = false;
        foreach(var pt in path)
        {
            if (float.IsNegativeInfinity(pt.x)) { hasPrev = false; continue; }
            Vector2 cur = WorldToPixel(pt, minX, minZ);
            if (hasPrev) DrawLine(tex, prev, cur, col);
            prev = cur; hasPrev = true;
        }
    }

    Vector2 WorldToPixel(Vector3 pos, float minX, float minZ)
    {
        float u = (pos.x - minX) / mapSize;
        float v = (pos.z - minZ) / mapSize;
        return new Vector2(u * (mapResolution - 1), v * (mapResolution - 1));
    }

    void DrawLine(Texture2D tex, Vector2 p1, Vector2 p2, Color col)
    {
        int x0 = (int)p1.x; int y0 = (int)p1.y;
        int x1 = (int)p2.x; int y1 = (int)p2.y;
        int dx = Mathf.Abs(x1 - x0), dy = Mathf.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        while(true)
        {
            DrawBrush(tex, x0, y0, col);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    }

    void DrawBrush(Texture2D tex, int x, int y, Color col)
    {
        int thickness = 3; // Line thickness
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

    void DrawCircle(Texture2D tex, int cx, int cy, int r, Color col)
    {
        for (int x = -r; x <= r; x++) for (int y = -r; y <= r; y++)
            if (x*x + y*y <= r*r && cx+x >=0 && cx+x < tex.width && cy+y >=0 && cy+y < tex.height) tex.SetPixel(cx + x, cy + y, col);
    }
}