using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

public class SmoothnessManager : MonoBehaviour
{
    public static SmoothnessManager Instance;

    [Header("Sampling Settings")]
    [Tooltip("Sampling interval in seconds (default 0.04s = 25fps)")]
    public float samplingInterval = 0.04f;

    [Header("Logging Settings")]
    public string saveSubFolder = "MetricsLogs";
    public bool verbose = true;

    private class AgentPathData
    {
        public string name;
        public List<Vector3> positions = new List<Vector3>();
        public bool isFinished = false;
    }

    private Dictionary<Transform, AgentPathData> agentPaths = new Dictionary<Transform, AgentPathData>();
    private float _accumulatedSimTime = 0f;
    private int lastProcessedFrame = -1;
    private bool hasExported = false;

    [Serializable]
    public class SmoothnessReport
    {
        public string sceneName;
        public string date;
        public string time;
        public int totalAgents;
        public float meanRMSJerk;
        public float stdDevRMSJerk;
        public List<AgentSmoothnessInfo> agents;
    }

    [Serializable]
    public class AgentSmoothnessInfo
    {
        public string agentName;
        public float rmsJerk;
        public int totalSamples;
    }

    void Awake()
    {
        if (Instance == null) Instance = this;
        else { Destroy(gameObject); return; }
    }

    void LateUpdate()
    {
        // 25fps 고정 간격 샘플링 (시간 스케일이 0보다 클 때만 누적)
        if (Time.timeScale > 0)
        {
            _accumulatedSimTime += Time.deltaTime;
        }

        int currentFrame = Mathf.FloorToInt(_accumulatedSimTime / samplingInterval + 0.001f);
        
        if (currentFrame > lastProcessedFrame)
        {
            for (int f = lastProcessedFrame + 1; f <= currentFrame; f++)
            {
                SamplePositions();
            }
            lastProcessedFrame = currentFrame;
        }
    }

    void SamplePositions()
    {
        // GBM 에이전트들을 찾아 샘플링합니다.
        AgentGBM[] gbmAgents = FindObjectsOfType<AgentGBM>(true);
        foreach (var agent in gbmAgents)
        {
            if (agent == null) continue;
            
            // GoalReached이거나 비활성화된 경우 측정을 종료합니다.
            bool isFinished = agent.GoalReached || !agent.gameObject.activeInHierarchy;
            RecordPosition(agent.transform, isFinished);
        }
    }

    void RecordPosition(Transform target, bool isFinished)
    {
        if (!agentPaths.TryGetValue(target, out AgentPathData data))
        {
            data = new AgentPathData { name = target.name };
            agentPaths.Add(target, data);
        }

        if (data.isFinished) return;

        if (isFinished)
        {
            data.isFinished = true;
            return;
        }

        data.positions.Add(target.position);
    }

    /// <summary>
    /// SmoothnessManager.txt와 동일한 RMS Jerk 계산 로직입니다.
    /// </summary>
    public float CalculateRMSJerk(List<Vector3> p)
    {
        int K = p.Count;
        if (K < 4) return 0f;

        int N = K - 3;
        double sumSqJerk = 0;
        float dt = samplingInterval;
        float dt3 = dt * dt * dt;

        for (int i = 0; i < N; i++)
        {
            // j_i = (p_{i+3} - 3p_{i+2} + 3p_{i+1} - p_i) / dt^3
            Vector3 jerkVec = (p[i + 3] - 3f * p[i + 2] + 3f * p[i + 1] - p[i]) / dt3;
            sumSqJerk += jerkVec.sqrMagnitude;
        }

        return (float)Math.Sqrt(sumSqJerk / N);
    }

    public void ExportToJson()
    {
        if (agentPaths.Count == 0 || hasExported) return;
        hasExported = true;

        List<AgentSmoothnessInfo> agentInfos = new List<AgentSmoothnessInfo>();
        List<float> allRmsJerks = new List<float>();

        foreach (var data in agentPaths.Values)
        {
            float rmsJerk = CalculateRMSJerk(data.positions);
            // 최소 4개의 샘플이 있어야 Jerk 계산이 가능합니다.
            if (rmsJerk <= 0 && data.positions.Count < 4) continue;

            agentInfos.Add(new AgentSmoothnessInfo
            {
                agentName = data.name,
                rmsJerk = rmsJerk,
                totalSamples = data.positions.Count
            });
            allRmsJerks.Add(rmsJerk);
        }

        if (allRmsJerks.Count == 0) return;

        float meanJerk = allRmsJerks.Average();
        float stdDevJerk = CalculateStdDev(allRmsJerks, meanJerk);

        SmoothnessReport report = new SmoothnessReport
        {
            sceneName = SceneManager.GetActiveScene().name,
            date = DateTime.Now.ToString("yyyy-MM-dd"),
            time = DateTime.Now.ToString("HH-mm-ss"),
            totalAgents = allRmsJerks.Count,
            meanRMSJerk = meanJerk,
            stdDevRMSJerk = stdDevJerk,
            agents = agentInfos
        };

        string folderPath = Path.Combine(Application.dataPath, "..", saveSubFolder);
        if (!Directory.Exists(folderPath)) Directory.CreateDirectory(folderPath);

        string fileName = $"{report.sceneName}_{report.date}_{report.time}_SmoothnessMetrics.json";
        string fullPath = Path.Combine(folderPath, fileName);

        try
        {
            File.WriteAllText(fullPath, JsonUtility.ToJson(report, true));
            if (verbose)
            {
                Debug.Log($"<color=#00FF00>[SmoothnessManager]</color> Exported to {fullPath}");
                Debug.Log($"<color=#00FF00><b>[Summary]</b></color> Mean RMS Jerk: {meanJerk:F4} (n={allRmsJerks.Count})");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[SmoothnessManager] Failed to export: {e.Message}");
        }
    }

    private float CalculateStdDev(List<float> values, float mean)
    {
        if (values.Count <= 1) return 0f;
        double sumOfSquares = values.Sum(v => Math.Pow(v - mean, 2));
        return (float)Math.Sqrt(sumOfSquares / values.Count);
    }

    void OnApplicationQuit()
    {
        ExportToJson();
    }
}
