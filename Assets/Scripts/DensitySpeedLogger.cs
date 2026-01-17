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
    [Header("Settings")]
    public string agentTag = "Agent";
    public bool stopOnFirstGoal = false;

    [Tooltip("Radius (Rs) for Gaussian Kernel Density Estimation (meters).")]
    public float measurementRadius = 5.0f;

    [Tooltip("Resolution for density binning in the report (e.g., 100 = 0.01 step).")]
    public int densityBinMultiplier = 50; // 0.02 step

    [Header("Sampling Settings")]
    [Tooltip("Time interval between measurements (seconds). Higher values smooth out jitter.")]
    public float sampleInterval = 0.5f;
    private float _sampleAccumulator = 0f;

    [Header("Data Filtering")]
    [Tooltip("Speeds below this value (m/s) are ignored (e.g. stationary).")]
    public float minValidSpeed = 0.05f;
    [Tooltip("Speeds above this value (m/s) are ignored (e.g. teleportation).")]
    public float maxValidSpeed = 10.0f; // Relaxed due to smoothing

    // Data Structure: 
    // Key: Quantized Density Bin (int) -> Density * densityBinMultiplier
    private SortedDictionary<int, SpeedAccumulator> densityData = new SortedDictionary<int, SpeedAccumulator>();

    private class SpeedAccumulator
    {
        public double totalSpeed;
        public double totalSpeedSq; // For StdDev: Sum(x^2)
        public double totalDensity; // To calculate exact average density in this bin
        public long count;
        public double minSpeed = double.MaxValue;
        public double maxSpeed = double.MinValue;
    }

    [Tooltip("Delay before starting measurement (seconds)")]
    public float startDelay = 2.0f;
    private float _timer = 0f;

    // Internal
    private Dictionary<int, Vector3> prevPositions = new Dictionary<int, Vector3>();
    private bool isFinished = false;

    // Cache for agents when TestManager is not used
    private List<GameObject> _cachedAgents = new List<GameObject>();
    private float _agentSearchTimer = 0f;
    private const float AGENT_SEARCH_INTERVAL = 0.5f;

    void Start()
    {
        _timer = 0f;
    }

    /// <summary>
    /// Calculates Gaussian Kernel Density for each agent.
    /// Formula: d_a^t = sum_{i in N(a^t)} (1 / (pi * Rs^2)) * exp(-|p_i^t - p_a^t|^2 / Rs^2)
    /// </summary>
    public List<float> CalculateGaussianDensities(List<Vector3> positions, float Rs)
    {
        int n = positions.Count;
        List<float> densities = new List<float>(new float[n]);

        float RsSq = Rs * Rs;
        // Normalization constant: 1 / (pi * Rs^2)
        float normalization = 1.0f / (Mathf.PI * RsSq);

        for (int i = 0; i < n; i++)
        {
            float d_a = 0f;
            Vector3 p_a = positions[i];

            for (int j = 0; j < n; j++)
            {
                Vector3 p_i = positions[j];

                // Distance squared on XZ plane
                float dx = p_i.x - p_a.x;
                float dz = p_i.z - p_a.z;
                float distSq = dx * dx + dz * dz;

                // Constraint: Only sum neighbors within distance Rs
                if (distSq <= RsSq)
                {
                    // Gaussian kernel
                    d_a += normalization * Mathf.Exp(-distSq / RsSq);
                }
            }
            densities[i] = d_a;
        }
        return densities;
    }

    void Update()
    {
        if (isFinished) return;

        float dt = Time.deltaTime;

        // Handle Start Delay
        if (_timer < startDelay)
        {
            _timer += dt;
            return;
        }

        // Sampling Logic
        _sampleAccumulator += dt;
        if (_sampleAccumulator < sampleInterval)
        {
            // Only update agent cache in background if needed, or just wait.
            // But checking agents every frame is redundant. Let's do it inside the sample block or separate timer.
            // We'll keep the agent search timer separate to ensure we have agents when the sample tick hits.
            _agentSearchTimer += dt;
            if (_agentSearchTimer >= AGENT_SEARCH_INTERVAL)
            {
                _agentSearchTimer = 0f;
                RefreshAgents();
            }
            return;
        }

        // --- MEASUREMENT TICK ---
        float timeStep = _sampleAccumulator;
        _sampleAccumulator = 0f;

        // If no agents cached yet, try finding them
        if (_cachedAgents.Count == 0) RefreshAgents();

        // Filter active only
        var activeAgents = _cachedAgents.Where(a => a != null && a.activeInHierarchy).ToList();
        int count = activeAgents.Count;
        if (count == 0) return;

        List<Vector3> positions = new List<Vector3>(count);
        List<int> agentIds = new List<int>(count);

        for (int i = 0; i < count; i++)
        {
            var agent = activeAgents[i];

            // Use "Root" for position tracking if available
            Transform rootObj = agent.transform.Find("Root");
            Vector3 pos = (rootObj != null) ? rootObj.position : agent.transform.position;

            positions.Add(pos);
            agentIds.Add(agent.GetInstanceID());

            // Check goal condition
            if (stopOnFirstGoal && CheckGoalReached(agent)) return;
        }

        // 2. Calculate Densities
        List<float> currentDensities = CalculateGaussianDensities(positions, measurementRadius);

        // 3. Calculate Speed and Bin Data
        for (int i = 0; i < count; i++)
        {
            int id = agentIds[i];
            Vector3 currentPosFlat = new Vector3(positions[i].x, 0, positions[i].z);
            float density = currentDensities[i];

            float speed = 0f;
            bool speedValid = false;

            if (prevPositions.TryGetValue(id, out Vector3 prevPosFlat))
            {
                float dist = Vector3.Distance(currentPosFlat, prevPosFlat);
                speed = dist / timeStep; // speed = distance / sampleInterval

                // Filter outliers with relaxed constraints due to smoothing
                if (speed >= minValidSpeed && speed <= maxValidSpeed) speedValid = true;
            }
            prevPositions[id] = currentPosFlat;

            if (!speedValid) continue;

            // Binning by Density
            int binKey = Mathf.FloorToInt(density * densityBinMultiplier);

            if (!densityData.ContainsKey(binKey))
            {
                densityData[binKey] = new SpeedAccumulator();
            }
            densityData[binKey].totalSpeed += speed;
            densityData[binKey].totalSpeedSq += (speed * speed);
            densityData[binKey].totalDensity += density;
            densityData[binKey].count++;

            if (speed < densityData[binKey].minSpeed) densityData[binKey].minSpeed = speed;
            if (speed > densityData[binKey].maxSpeed) densityData[binKey].maxSpeed = speed;
        }
    }

    void RefreshAgents()
    {
        _cachedAgents.Clear();
        var agentsFound = GameObject.FindGameObjectsWithTag(agentTag);
        _cachedAgents.AddRange(agentsFound);
    }

    bool CheckGoalReached(GameObject agent)
    {
        bool reached = false;

        if (!agent.activeInHierarchy) reached = true;

        if (!reached)
        {
            var agentGBM = agent.GetComponent<AgentGBM>();
            if (agentGBM != null && agentGBM.GoalReached) reached = true;
        }

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
        sb.AppendLine("<b>[Final Density-Speed Report (Gaussian KDE)]</b>");
        sb.AppendLine("Density_Bin_Center | Avg_Density(ag/m^2) | Avg_Speed(m/s) | Min_Speed | Max_Speed | Std_Dev | Samples");

        foreach (var kvp in densityData)
        {
            int bin = kvp.Key;
            double exactAvgDensity = kvp.Value.totalDensity / kvp.Value.count;
            double avgSpeed = kvp.Value.totalSpeed / kvp.Value.count;

            // Standard Deviation Calculation: sqrt(E[X^2] - (E[X])^2)
            double avgSpeedSq = kvp.Value.totalSpeedSq / kvp.Value.count;
            double variance = avgSpeedSq - (avgSpeed * avgSpeed);
            if (variance < 0) variance = 0; // Floating point safeguard
            double stdDev = System.Math.Sqrt(variance);

            // Bin Center calculation for reference
            float binCenter = (bin + 0.5f) / densityBinMultiplier;

            sb.AppendLine($"{binCenter:F3} | {exactAvgDensity:F4} | {avgSpeed:F4} | {kvp.Value.minSpeed:F4} | {kvp.Value.maxSpeed:F4} | {stdDev:F4} | {kvp.Value.count}");
        }

        Debug.Log(sb.ToString());

        // CSV Format for easy copy-paste
        StringBuilder csv = new StringBuilder();
        csv.AppendLine("\n[CSV Format]");
        csv.AppendLine("Density,Avg_Speed,Min_Speed,Max_Speed,Std_Dev");
        foreach (var kvp in densityData)
        {
            double exactAvgDensity = kvp.Value.totalDensity / kvp.Value.count;
            double avgSpeed = kvp.Value.totalSpeed / kvp.Value.count;

            double avgSpeedSq = kvp.Value.totalSpeedSq / kvp.Value.count;
            double variance = avgSpeedSq - (avgSpeed * avgSpeed);
            if (variance < 0) variance = 0;
            double stdDev = System.Math.Sqrt(variance);

            csv.AppendLine($"{exactAvgDensity:F4},{avgSpeed:F4},{kvp.Value.minSpeed:F4},{kvp.Value.maxSpeed:F4},{stdDev:F4}");
        }
        Debug.Log(csv.ToString());
    }

    private void OnApplicationQuit() { if (!isFinished) LogFinalResult(); }

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
        GUILayout.BeginArea(new Rect(10, 10, 350, 450), GUI.skin.box);
        GUILayout.Label($"<b>Density Speed Logger (Gaussian KDE)</b>");
        GUILayout.Label($"Radius (Rs): {measurementRadius}m");

        GUILayout.Space(10);
        GUILayout.Label("<b>Live Data (Top 15 Bins)</b>");

        int shown = 0;
        foreach (var kvp in densityData)
        {
            if (shown++ > 15) break;
            double avgD = kvp.Value.totalDensity / kvp.Value.count;
            double avgS = kvp.Value.totalSpeed / kvp.Value.count;
            GUILayout.Label($"D={avgD:F2} : <b>{avgS:F2} m/s</b> ({kvp.Value.count})");
        }

        GUILayout.Space(10);
        stopOnFirstGoal = GUILayout.Toggle(stopOnFirstGoal, "Stop on First Goal");
        if (GUILayout.Button("Reset Metrics")) ResetMetrics();
        GUILayout.EndArea();
    }
}