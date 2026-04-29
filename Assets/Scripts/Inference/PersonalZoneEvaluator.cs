using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

public class PersonalZoneEvaluator : MonoBehaviour
{
    public static PersonalZoneEvaluator Instance;

    [Header("Evaluation Settings")]
    [Tooltip("The radius of the personal zone (in meters).")]
    public float personalZoneRadius = 1.5f;
    
    [Tooltip("The root object where agents are located.")]
    public Transform agentsRoot;

    [Header("Logging Settings")]
    public string saveSubFolder = "MetricsLogs";
    public bool verbose = true;

    [Header("Sampling Settings")]
    [Tooltip("Sampling interval in seconds (default 0.04s = 25fps)")]
    public float samplingInterval = 0.04f;

    [Serializable]
    public class PersonalZoneMetricsReport
    {
        public string sceneName;
        public string date;
        public string time;
        public int totalSamples;
        public float personalZoneRadius;
        public float averageViolatorsPerAgent;
        public float violationRate; // Frames with at least one violation / Total frames
        public List<ViolatorCountEntry> histogramEntries;
    }

    [Serializable]
    public class ViolatorCountEntry
    {
        public int violatorCount;
        public int frameCount;
    }

    private Dictionary<int, int> _violatorsCountHistogram = new Dictionary<int, int>();
    private List<int> _allViolationCounts = new List<int>();
    private int _totalFramesSampled = 0;
    private int _framesWithViolations = 0;

    private int _lastProcessedFrame = -1;
    private float _accumulatedSimTime = 0f;
    private bool _hasExported = false;

    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
        }
        else
        {
            Destroy(gameObject);
            return;
        }
    }

    void LateUpdate()
    {
        if (Time.timeScale > 0)
        {
            _accumulatedSimTime += Time.deltaTime;
        }

        int currentFrame = Mathf.FloorToInt(_accumulatedSimTime / samplingInterval + 0.001f);
        
        if (currentFrame > _lastProcessedFrame)
        {
            for (int f = _lastProcessedFrame + 1; f <= currentFrame; f++)
            {
                SamplePersonalZoneViolations();
            }
            _lastProcessedFrame = currentFrame;
        }
    }

    void SamplePersonalZoneViolations()
    {
        AgentGBM[] allAgents;
        
        if (agentsRoot != null)
        {
            allAgents = agentsRoot.GetComponentsInChildren<AgentGBM>();
        }
        else
        {
            allAgents = FindObjectsOfType<AgentGBM>();
        }

        if (allAgents.Length == 0) return;

        int frameTotalViolators = 0;
        bool frameHasViolation = false;

        foreach (var agent in allAgents)
        {
            if (agent == null || !agent.gameObject.activeInHierarchy)
                continue;

            int agentViolators = 0;
            Vector3 agentPos = agent.transform.position;

            foreach (var other in allAgents)
            {
                if (other == null || other == agent || !other.gameObject.activeInHierarchy)
                    continue;

                // Check if they are in the same group
                if (IsSameGroup(agent, other))
                    continue;

                // Planar distance check
                float dist = Vector2.Distance(
                    new Vector2(agentPos.x, agentPos.z),
                    new Vector2(other.transform.position.x, other.transform.position.z)
                );

                if (dist < personalZoneRadius)
                {
                    agentViolators++;
                }
            }

            if (agentViolators > 0)
            {
                frameHasViolation = true;
                frameTotalViolators += agentViolators;
            }

            // Record histogram for each agent's personal zone state in this frame
            if (!_violatorsCountHistogram.ContainsKey(agentViolators))
                _violatorsCountHistogram[agentViolators] = 0;
            _violatorsCountHistogram[agentViolators]++;
            
            _allViolationCounts.Add(agentViolators);
        }

        _totalFramesSampled++;
        if (frameHasViolation)
        {
            _framesWithViolations++;
        }
    }

    private bool IsSameGroup(AgentGBM a, AgentGBM b)
    {
        if (a.groupMembers == null || a.groupMembers.Count == 0) return false;
        return a.groupMembers.Contains(b.transform);
    }

    public void ExportToJson()
    {
        if (_allViolationCounts.Count == 0 || _hasExported) return;
        _hasExported = true;

        float avgViolators = (float)_allViolationCounts.Average();
        float violationRate = _totalFramesSampled > 0 ? (float)_framesWithViolations / _totalFramesSampled : 0f;

        List<ViolatorCountEntry> histogram = _violatorsCountHistogram
            .Select(kvp => new ViolatorCountEntry { violatorCount = kvp.Key, frameCount = kvp.Value })
            .OrderBy(e => e.violatorCount)
            .ToList();

        PersonalZoneMetricsReport report = new PersonalZoneMetricsReport
        {
            sceneName = SceneManager.GetActiveScene().name,
            date = DateTime.Now.ToString("yyyy-MM-dd"),
            time = DateTime.Now.ToString("HH-mm-ss"),
            totalSamples = _allViolationCounts.Count,
            personalZoneRadius = personalZoneRadius,
            averageViolatorsPerAgent = avgViolators,
            violationRate = violationRate,
            histogramEntries = histogram
        };

        string folderPath = Path.Combine(Application.dataPath, "..", saveSubFolder);
        if (!Directory.Exists(folderPath))
        {
            Directory.CreateDirectory(folderPath);
        }

        string fileName = $"{report.sceneName}_{report.date}_{report.time}_PersonalZoneMetrics.json";
        string fullPath = Path.Combine(folderPath, fileName);
        
        try
        {
            File.WriteAllText(fullPath, JsonUtility.ToJson(report, true));
            
            if (verbose)
            {
                Debug.Log($"<color=#00FFFF>[PersonalZoneEvaluator]</color> Exported Personal Zone Metrics to {fullPath}");
                Debug.Log($"<color=#00FFFF>[PersonalZoneEvaluator]</color> Avg Violators: {avgViolators:F4}, Violation Rate: {violationRate:P2}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[PersonalZoneEvaluator] Failed to export metrics: {e.Message}");
        }
    }

    void OnApplicationQuit()
    {
        ExportToJson();
    }
}
