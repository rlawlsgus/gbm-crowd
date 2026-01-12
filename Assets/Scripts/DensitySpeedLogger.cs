using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Text;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class DensitySpeedLogger : MonoBehaviour
{
    [Header("References")]
    public TestManager testManager;

    [Header("Settings")]
    public bool stopOnFirstGoal = false;
    
    [Tooltip("Radius to calculate local density around each agent (meters).")]
    public float measurementRadius = 3.0f; 
    
    // Data Structure: 
    // Key: Neighbor Count (N) - Discrete values
    private SortedDictionary<int, SpeedAccumulator> densityData = new SortedDictionary<int, SpeedAccumulator>();

    private class SpeedAccumulator
    {
        public double totalSpeed;
        public long count;
    }

    // Internal
    private Dictionary<int, Vector3> prevPositions = new Dictionary<int, Vector3>();
    private bool isFinished = false;
    private float measurementArea; // Pi * r^2

    void Start()
    {
        if (testManager == null)
            testManager = FindObjectOfType<TestManager>();

        if (testManager == null)
            Debug.LogWarning("[DensitySpeedLogger] TestManager not found. Please assign it manually.");
            
        measurementArea = Mathf.PI * measurementRadius * measurementRadius;
        if (measurementArea < 0.001f) measurementArea = 1f; 
    }

    void Update()
    {
        if (isFinished) return;
        if (testManager == null || testManager.activeAgents == null) return;

        float dt = Time.deltaTime;
        if (dt <= 1e-5f) return;

        List<GameObject> activeAgents = testManager.activeAgents;
        int count = activeAgents.Count;

        for (int i = 0; i < count; i++)
        {
            var agent = activeAgents[i];
            if (agent == null) continue;

            if (stopOnFirstGoal && CheckGoalReached(agent)) return;

            if (!agent.activeInHierarchy)
            {
                prevPositions.Remove(agent.GetInstanceID());
                continue;
            }

            int id = agent.GetInstanceID();
            Vector3 currentPos = agent.transform.position;
            Vector3 currentPosFlat = new Vector3(currentPos.x, 0, currentPos.z);
            
            float speed = 0f;
            bool speedValid = false;

            if (prevPositions.TryGetValue(id, out Vector3 prevPosFlat))
            {
                float dist = Vector3.Distance(currentPosFlat, prevPosFlat);
                speed = dist / dt;
                
                if (speed < 20.0f) speedValid = true;
            }
            prevPositions[id] = currentPosFlat;

            if (!speedValid) continue; 

            // Measure Local Density (Discrete Neighbor Count)
            int neighborCount = 1; // Include self

            for (int j = 0; j < count; j++)
            {
                if (i == j) continue;
                var neighbor = activeAgents[j];
                if (neighbor == null || !neighbor.activeInHierarchy) continue;

                if (Vector3.Distance(agent.transform.position, neighbor.transform.position) <= measurementRadius)
                {
                    neighborCount++;
                }
            }

            // Store directly by neighbor count
            if (!densityData.ContainsKey(neighborCount))
            {
                densityData[neighborCount] = new SpeedAccumulator();
            }
            densityData[neighborCount].totalSpeed += speed;
            densityData[neighborCount].count++;
        }
    }

    bool CheckGoalReached(GameObject agent)
    {
        if (agent == null) return false;

        bool reached = false;
        
        // Check AgentGBM specific flag
        AgentGBM agentGBM = agent.GetComponent<AgentGBM>();
        if (agentGBM != null && agentGBM.GoalReached) reached = true;
        
        // Also check if deactivated (standard behavior for reaching goal in this project)
        if (!agent.activeInHierarchy) reached = true; 

        if (reached)
        {
            Debug.Log($"[DensitySpeedLogger] Agent {agent.name} reached goal. Stopping.");
            FinishAndStop();
            return true;
        }
        return false;
    }

    void FinishAndStop()
    {
        isFinished = true;
        LogFinalResult();
#if UNITY_EDITOR
        EditorApplication.isPlaying = false;
#else
        Application.Quit();
#endif
    }

    private void LogFinalResult()
    {
        StringBuilder sb = new StringBuilder();
        sb.AppendLine("<b>[Final Density-Speed Report (Discrete)]</b>");
        sb.AppendLine("Neighbors(N) | Density(ag/m^2) | Avg_Speed(m/s) | Sample_Count");
        
        foreach (var kvp in densityData)
        {
            int N = kvp.Key;
            float density = N / measurementArea;
            double avgSpeed = kvp.Value.totalSpeed / kvp.Value.count;

            sb.AppendLine($"{N} | {density:F4} | {avgSpeed:F4} | {kvp.Value.count}");
        }
        Debug.Log(sb.ToString());
    }

    private void OnApplicationQuit() { if (!isFinished) LogFinalResult(); }

    private void OnDestroy() { if (!isFinished) LogFinalResult(); }

    public void ResetMetrics()
    {
        if (!isFinished) LogFinalResult();
        densityData.Clear();
        prevPositions.Clear();
        isFinished = false;
        Debug.Log("[DensitySpeedLogger] Metrics Reset.");
    }

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 400), GUI.skin.box);
        GUILayout.Label($"<b>Density Speed Logger</b>");
        GUILayout.Label($"Radius: {measurementRadius}m (Area: {measurementArea:F1}m^2)");
        
        GUILayout.Space(10);
        GUILayout.Label("<b>Live Data (N -> Speed)</b>");
        
        int shown = 0;
        foreach(var kvp in densityData)
        {
            if (shown++ > 15) break;
            int N = kvp.Key;
            float density = N / measurementArea;
            double avg = kvp.Value.totalSpeed / kvp.Value.count;
            GUILayout.Label($"N={N} ({density:F2}): <b>{avg:F2} m/s</b> ({kvp.Value.count})");
        }

        GUILayout.Space(10);
        stopOnFirstGoal = GUILayout.Toggle(stopOnFirstGoal, "Stop on First Goal");
        if (GUILayout.Button("Reset Metrics")) ResetMetrics();
        GUILayout.EndArea();
    }
}
