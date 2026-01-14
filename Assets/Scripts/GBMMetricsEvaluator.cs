using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;
using System;

public class GBMMetricsEvaluator : MonoBehaviour
{
    [Header("Configuration")]
    public GameObject agentPrefab;
    public ZaraGroupRotationSimulator simulator;
    public Transform obstaclesParent;
    public float sampleInterval = 0.8f;
    public float agentTimeout = 60.0f; // Agents older than this will be removed

    [Header("Metrics Settings")]
    public float maxScanDistance = 5.0f;
    public string outputFolder = "Assets/Analysis_Output_GBM";
    public string fileNamePrefix = "GBM_Metrics_Report";

    // Internal State
    private Dictionary<int, IGBMAgent> activeAgents = new Dictionary<int, IGBMAgent>();
    private Dictionary<int, float> agentSpawnTimes = new Dictionary<int, float>();
    private HashSet<int> spawnedAgentIDs = new HashSet<int>();
    private Collider[] obstacleColliders;
    
    // --- Metrics Data ---
    // For raw data export
    private struct MetricDataPoint
    {
        public float TimeSinceSpawn;
        public float DistanceToGoal;
        public float MinNeighborDist;
        public float MinObstacleDist;
        public int ActiveAgentCount;
    }
    private List<MetricDataPoint> allMetrics = new List<MetricDataPoint>();

    // For aggregated report
    private List<float> stepAvgAgentDists = new List<float>();
    private List<float> stepAvgObstacleDists = new List<float>();
    private Dictionary<int, List<float>> agentDistByCount = new Dictionary<int, List<float>>();
    private Dictionary<int, List<float>> obsDistByCount = new Dictionary<int, List<float>>();

    // --- Agent Abstraction ---
    private interface IGBMAgent
    {
        int ID { get; }
        Transform transform { get; }
        GameObject gameObject { get; }
        Vector3 GoalPosition { set; }
        bool pdmMode { set; }
        bool GoalReached { get; }
        float GetDistanceToGoal();
    }

    private class GBMAgentWrapper : IGBMAgent
    {
        private AgentGBM agent;
        public int ID { get; }
        public GBMAgentWrapper(AgentGBM a, int id) { agent = a; ID = id; }
        public Transform transform => agent.transform;
        public GameObject gameObject => agent.gameObject;
        public Vector3 GoalPosition { set => agent.GoalPosition = value; }
        public bool pdmMode { set => agent.pdmMode = value; }
        public bool GoalReached => agent.GoalReached;
        public float GetDistanceToGoal() => agent.GetDistanceToGoal();
    }

    IEnumerator Start()
    {
        // 0. Reset State
        GradientBasedModel.ClearAgents();
        spawnedAgentIDs.Clear();
        allMetrics.Clear();

        // 1. Dependencies Check
        if (simulator == null) simulator = FindObjectOfType<ZaraGroupRotationSimulator>();
        if (simulator == null)
        {
            Debug.LogError("[GBMMetricsEvaluator] Simulator not found.");
            yield break;
        }

        if (obstaclesParent == null)
        {
            GameObject obsObj = GameObject.Find("Obstacles");
            if (obsObj != null) obstaclesParent = obsObj.transform;
        }

        if (obstaclesParent != null)
        {
            obstacleColliders = obstaclesParent.GetComponentsInChildren<Collider>();
        }
        else
        {
            obstacleColliders = new Collider[0];
        }

        // 2. Initialize Simulator
        // Ensure simulator is playing
        simulator.Play(); 
        
        // 3. Start Evaluation Loop
        StartCoroutine(EvaluationLoop());
    }

    // Update is called once per frame
    void Update()
    {
        CheckAndSpawnAgents();
    }

    void CheckAndSpawnAgents()
    {
        if (simulator == null) return;

        // Check for active GT agents that don't have a corresponding GBM agent yet
        foreach (Transform child in simulator.transform)
        {
            // Only interested in active "ped_X" objects
            if (!child.gameObject.activeSelf) continue;
            if (!child.name.StartsWith("ped_")) continue;

            int id;
            if (!int.TryParse(child.name.Replace("ped_", ""), out id)) continue;

            // If we haven't spawned a GBM agent for this ID yet, and it's not an ID we've spawned before
            if (!activeAgents.ContainsKey(id) && !spawnedAgentIDs.Contains(id))
            {
                SpawnGBMAgent(id, child);
            }
            
            // Hide GT agent to avoid visual clutter/z-fighting, 
            // but keep it active so Simulator logic continues.
            // Note: If Simulator resets active state, this might flicker.
            // Renderer disabling is safer than SetActive(false) if Simulator checks activeSelf.
            var renderers = child.GetComponentsInChildren<Renderer>();
            foreach (var r in renderers) r.enabled = false;
        }
    }

    void SpawnGBMAgent(int id, Transform gtTransform)
    {
        Vector3 startPos = gtTransform.position;
        Quaternion startRot = gtTransform.rotation;
        
        GameObject agentObj = Instantiate(agentPrefab, startPos, startRot);
        agentObj.name = $"GBM_Agent_{id}";

        var agentComponent = agentObj.GetComponent<AgentGBM>();
        if (agentComponent == null)
        {
            Debug.LogError($"[GBMMetricsEvaluator] Prefab missing AgentGBM component: {agentObj.name}");
            Destroy(agentObj);
            return;
        }

        // Register the agent with the simulator
        GradientBasedModel.AddAgent(agentComponent);

        IGBMAgent gbmAgent = new GBMAgentWrapper(agentComponent, id);

        // Configure Agent
        gbmAgent.pdmMode = true; // Enable standalone mode (bypassing Manager)
        
        Vector3 finalGoal = simulator.GetFinalWorldPosition(id);
        gbmAgent.GoalPosition = finalGoal;

        activeAgents.Add(id, gbmAgent);
        agentSpawnTimes.Add(id, Time.time);
        spawnedAgentIDs.Add(id);
        // Debug.Log($"[GBMMetricsEvaluator] Spawned Agent {id} at {startPos}");
    }

    IEnumerator EvaluationLoop()
    {
        // Initial wait
        yield return null;

        while (true)
        {
            // Wait for interval
            yield return new WaitForSeconds(sampleInterval);

            // Filter active agents and check timeouts
            List<int> agentsToRemove = new List<int>();
            List<IGBMAgent> validAgents = new List<IGBMAgent>();

            foreach (var kvp in activeAgents)
            {
                int id = kvp.Key;
                IGBMAgent agent = kvp.Value;

                // 1. Check if null (destroyed externally)
                if (agent == null || agent.gameObject == null)
                {
                    agentsToRemove.Add(id);
                    continue;
                }

                // 2. Check if timed out
                if (agentSpawnTimes.ContainsKey(id))
                {
                    float age = Time.time - agentSpawnTimes[id];
                    if (age > agentTimeout)
                    {
                        // Timeout! Destroy and mark for removal
                        // Debug.Log($"[GBMMetricsEvaluator] Agent {id} timed out (Age: {age:F1}s). Removing.");
                        Destroy(agent.gameObject);
                        agentsToRemove.Add(id);
                        continue;
                    }
                }
                
                // 3. Check if goal reached
                if (agent.GoalReached)
                {
                    // Debug.Log($"[GBMMetricsEvaluator] Agent {id} reached its goal. Removing.");
                    // The agent handles its own destruction/deactivation, but we must remove it from metrics.
                    // Destroying here might be redundant if agent deactivates, but is safe.
                    Destroy(agent.gameObject); 
                    agentsToRemove.Add(id);
                    continue;
                }

                // 4. Check if active (might be disabled if finished goal)
                if (agent.gameObject.activeInHierarchy)
                {
                    validAgents.Add(agent);
                }
            }

            // Clean up removed agents
            foreach (int id in agentsToRemove)
            {
                activeAgents.Remove(id);
                agentSpawnTimes.Remove(id);
            }

            if (validAgents.Count > 0)
            {
                RecordMetrics(validAgents);
            }
        }
    }

    void RecordMetrics(List<IGBMAgent> agents)
    {
        int count = agents.Count;
        if (count == 0) return;

        float stepSumNeighbor = 0f;
        int stepValidNeighbor = 0;
        float stepSumObs = 0f;
        int stepValidObs = 0;

        for (int i = 0; i < count; i++)
        {
            IGBMAgent agent = agents[i];
            Vector3 p1 = agent.transform.position;

            // 1. Per-Agent Neighbor Distance
            float minNeighborDist = float.PositiveInfinity;
            for (int j = 0; j < count; j++)
            {
                if (i == j) continue;
                float d = Vector3.Distance(p1, agents[j].transform.position);
                if (d < minNeighborDist)
                {
                    minNeighborDist = d;
                }
            }

            // 2. Per-Agent Obstacle Distance
            float minObstacleDist = float.PositiveInfinity;
            if (obstacleColliders != null && obstacleColliders.Length > 0)
            {
                foreach (var col in obstacleColliders)
                {
                    if (col == null) continue;
                    Vector3 cp = col.ClosestPoint(p1);
                    float d = Vector3.Distance(p1, cp);
                    if (d < minObstacleDist)
                    {
                        minObstacleDist = d;
                    }
                }
            }

            // 3. Store Raw Data Point
            allMetrics.Add(new MetricDataPoint
            {
                TimeSinceSpawn = Time.time - agentSpawnTimes[agent.ID],
                DistanceToGoal = agent.GetDistanceToGoal(),
                MinNeighborDist = (minNeighborDist <= maxScanDistance) ? minNeighborDist : float.PositiveInfinity,
                MinObstacleDist = (minObstacleDist <= maxScanDistance) ? minObstacleDist : float.PositiveInfinity,
                ActiveAgentCount = count
            });

            // 4. Accumulate for step-wide average (maintaining old report style)
            if (minNeighborDist <= maxScanDistance)
            {
                stepSumNeighbor += minNeighborDist;
                stepValidNeighbor++;
            }
            if (minObstacleDist <= maxScanDistance)
            {
                stepSumObs += minObstacleDist;
                stepValidObs++;
            }
        }

        // 5. Update aggregated lists for summary report
        float avgNeighbor = (stepValidNeighbor > 0) ? stepSumNeighbor / stepValidNeighbor : 0f;
        if (stepValidNeighbor > 0)
        {
            stepAvgAgentDists.Add(avgNeighbor);
            if (!agentDistByCount.ContainsKey(count)) agentDistByCount[count] = new List<float>();
            agentDistByCount[count].Add(avgNeighbor);
        }

        float avgObs = (stepValidObs > 0) ? stepSumObs / stepValidObs : 0f;
        if (stepValidObs > 0)
        {
            stepAvgObstacleDists.Add(avgObs);
            if (!obsDistByCount.ContainsKey(count)) obsDistByCount[count] = new List<float>();
            obsDistByCount[count].Add(avgObs);
        }
    }

    void SaveReport()
    {
        if (stepAvgAgentDists.Count == 0 && stepAvgObstacleDists.Count == 0) return;

        float totalAvgAgent = (stepAvgAgentDists.Count > 0) ? stepAvgAgentDists.Average() : 0f;
        float totalAvgObs = (stepAvgObstacleDists.Count > 0) ? stepAvgObstacleDists.Average() : 0f;

        System.Text.StringBuilder sb = new System.Text.StringBuilder();

        sb.AppendLine("============================================");
        sb.AppendLine("       GBM CROWD EVALUATION REPORT          ");
        sb.AppendLine("============================================");
        sb.AppendLine($" Date                 : {DateTime.Now}");
        sb.AppendLine($" Agent Prefab         : {(agentPrefab != null ? agentPrefab.name : "None")}");
        sb.AppendLine($" Max Scan Distance    : {maxScanDistance} m");
        sb.AppendLine($" Interval             : {sampleInterval} sec");
        sb.AppendLine($" Total Steps Analyzed : {Mathf.Max(stepAvgAgentDists.Count, stepAvgObstacleDists.Count)}");
        sb.AppendLine($" ------------------------------------------");
        sb.AppendLine($" OVERALL AVG NEIGHBOR DIST : {totalAvgAgent:F4}");
        sb.AppendLine($" OVERALL AVG OBSTACLE DIST : {totalAvgObs:F4}");
        sb.AppendLine("============================================");
        sb.AppendLine("       METRICS BY ACTIVE AGENT COUNT        ");
        sb.AppendLine("============================================");

        var allKeys = new HashSet<int>(agentDistByCount.Keys);
        allKeys.UnionWith(obsDistByCount.Keys);
        var sortedKeys = allKeys.OrderBy(k => k).ToList();

        foreach (int c in sortedKeys)
        {
            string aStr = agentDistByCount.ContainsKey(c) ? $"{agentDistByCount[c].Average():F4}m" : "N/A";
            string oStr = obsDistByCount.ContainsKey(c) ? $"{obsDistByCount[c].Average():F4}m" : "N/A";
            sb.AppendLine($" Agents: {c,2} | Neighbor: {aStr,8} | Obstacle: {oStr,8}");
        }
        sb.AppendLine("============================================");

        string content = sb.ToString();
        Debug.Log(content);

        try
        {
            if (!Directory.Exists(outputFolder)) Directory.CreateDirectory(outputFolder);
            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string path = Path.Combine(outputFolder, $"{fileNamePrefix}_{timestamp}.txt");
            File.WriteAllText(path, content);
            Debug.Log($"[GBMMetricsEvaluator] Report saved: {path}");
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to save report: {ex.Message}");
        }
    }

    void SaveRawData()
    {
        if (allMetrics.Count == 0) return;

        System.Text.StringBuilder sb = new System.Text.StringBuilder();
        sb.AppendLine("TimeSinceSpawn,DistanceToGoal,MinNeighborDist,MinObstacleDist,ActiveAgentCount");

        foreach (var dp in allMetrics)
        {
            string neighborDist = float.IsPositiveInfinity(dp.MinNeighborDist) ? "" : dp.MinNeighborDist.ToString("F4");
            string obstacleDist = float.IsPositiveInfinity(dp.MinObstacleDist) ? "" : dp.MinObstacleDist.ToString("F4");

            sb.AppendLine($"{dp.TimeSinceSpawn:F4},{dp.DistanceToGoal:F4},{neighborDist},{obstacleDist},{dp.ActiveAgentCount}");
        }

        try
        {
            if (!Directory.Exists(outputFolder)) Directory.CreateDirectory(outputFolder);
            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string path = Path.Combine(outputFolder, $"{fileNamePrefix}_RawData_{timestamp}.csv");
            File.WriteAllText(path, sb.ToString());
            Debug.Log($"[GBMMetricsEvaluator] Raw data saved: {path}");
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to save raw data: {ex.Message}");
        }
    }

    void OnApplicationQuit()
    {
        SaveReport();
        SaveRawData();
        GradientBasedModel.ClearAgents();
    }
}
