using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.IO;
using System.Text;

public class DiversityEvaluator : MonoBehaviour
{
    [Header("Collection Settings")]
    [Tooltip("Output directory for path files.")]
    public string outputFolder = "DiversityData";
    [Tooltip("Unique suffix for this run (e.g., Run1, Run2). If empty, uses timestamp.")]
    public string runId = "";
    [Tooltip("Minimum Agent ID to record (inclusive).")]
    public int minAgentID = 0;
    [Tooltip("Maximum Agent ID to record (inclusive).")]
    public int maxAgentID = 100;
    [Tooltip("Minimum distance movement required to record a new path point.")]
    public float minRecordDistance = 0.1f;
    [Tooltip("Maximum duration to record paths in seconds. Set to 0 for unlimited.")]
    public float maxRecordingTime = 60.0f;

    // Internal State
    // Key: Agent ID, Value: List of points
    private Dictionary<int, List<Vector3>> agentPaths = new Dictionary<int, List<Vector3>>();
    // Cache map for Agent Transform -> ID
    private Dictionary<Transform, int> trackedAgents = new Dictionary<Transform, int>();
    
    private float searchTimer = 0f;
    private const float searchInterval = 1.0f;
    private float recordingTimer = 0f;

    void Start()
    {
        if (string.IsNullOrEmpty(runId))
        {
            runId = System.DateTime.Now.ToString("yyyyMMdd_HHmmss");
        }
        
        // Create output directory if it doesn't exist
        string dirPath = Path.Combine(Application.dataPath, "..", outputFolder);
        if (!Directory.Exists(dirPath))
        {
            Directory.CreateDirectory(dirPath);
        }
    }

    void Update()
    {
        // Check recording time limit
        if (maxRecordingTime > 0 && recordingTimer >= maxRecordingTime)
        {
            return;
        }
        
        recordingTimer += Time.deltaTime;

        // Periodic Search for new agents
        searchTimer += Time.deltaTime;
        if (searchTimer >= searchInterval)
        {
            FindAgents();
            searchTimer = 0f;
        }

        // Record Paths
        RecordCurrentPositions();
    }

    void FindAgents()
    {
        // Find all AgentGBM components in the scene
        AgentGBM[] allAgents = FindObjectsOfType<AgentGBM>();

        foreach (var agent in allAgents)
        {
            if (trackedAgents.ContainsKey(agent.transform)) continue;

            int id = ParseAgentID(agent.gameObject.name);
            
            // Check if ID is within range
            if (id >= minAgentID && id <= maxAgentID)
            {
                trackedAgents.Add(agent.transform, id);
                if (!agentPaths.ContainsKey(id))
                {
                    agentPaths[id] = new List<Vector3>();
                }
                // Debug.Log($"[DiversityEvaluator] Started tracking Agent {id} ({agent.name})");
            }
        }
    }

    // Try to parse ID from name (e.g., "GBM_Agent_10" -> 10, "ped_5" -> 5)
    int ParseAgentID(string name)
    {
        string digits = new string(name.Where(char.IsDigit).ToArray());
        if (int.TryParse(digits, out int id))
        {
            return id;
        }
        return -1;
    }

    void RecordCurrentPositions()
    {
        foreach (var kvp in trackedAgents)
        {
            Transform t = kvp.Key;
            int id = kvp.Value;

            if (t == null || !t.gameObject.activeInHierarchy) continue;

            Vector3 currentPos = t.position;
            List<Vector3> path = agentPaths[id];

            // Add point if path is empty or moved enough
            if (path.Count == 0 || Vector3.Distance(path.Last(), currentPos) >= minRecordDistance)
            {
                path.Add(currentPos);
            }
        }
    }

    void OnApplicationQuit()
    {
        SavePathsToFile();
    }

    public void SavePathsToFile()
    {
        if (agentPaths.Count == 0) return;

        string fileName = $"Paths_{runId}.txt";
        string fullPath = Path.Combine(Application.dataPath, "..", outputFolder, fileName);

        StringBuilder sb = new StringBuilder();
        
        // File Format:
        // ID:x,y,z;x,y,z;...
        
        foreach (var kvp in agentPaths)
        {
            int id = kvp.Key;
            List<Vector3> path = kvp.Value;
            
            if (path.Count < 2) continue; // Skip practically empty paths

            sb.Append($"{id}:");
            for (int i = 0; i < path.Count; i++)
            {
                Vector3 p = path[i];
                sb.Append($"{p.x:F4},{p.y:F4},{p.z:F4}");
                if (i < path.Count - 1) sb.Append(";");
            }
            sb.AppendLine();
        }

        try
        {
            File.WriteAllText(fullPath, sb.ToString());
            Debug.Log($"[DiversityEvaluator] Saved {agentPaths.Count} agent paths to {fullPath}");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"[DiversityEvaluator] Failed to save paths: {ex.Message}");
        }
    }
}
