using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.IO;

public class DiversityEvaluator : MonoBehaviour
{
    [Header("Settings")]
    public List<AgentGBM> targetAgents; // Assign existing agents here
    public int trialsPerEpisode = 5;
    public string outputFileName = "DiversityScore.txt";

    private List<List<Vector3>> recordedPaths = new List<List<Vector3>>();
    private List<Vector3> currentPath = new List<Vector3>();
    
    // State
    private bool isRunning = false;

    void Start()
    {
        if (targetAgents == null || targetAgents.Count == 0)
        {
            Debug.LogError("[DiversityEvaluator] No agents assigned! Please assign agents in the Inspector.");
            return;
        }

        StartCoroutine(EvaluationLoop());
    }

    IEnumerator EvaluationLoop()
    {
        // Wait for Simulators to initialize (they often use Start() to find agents)
        yield return new WaitForSeconds(1.0f);

        isRunning = true;
        List<float> agentDiversityScores = new List<float>();

        foreach (var agent in targetAgents)
        {
            if (agent == null) continue;

            Debug.Log($"[DiversityEvaluator] Evaluating Agent: {agent.name}");

            // 1. Store Initial State
            Vector3 startPos = agent.transform.position;
            Quaternion startRot = agent.transform.rotation;
            
            // 2. Identify Goal (optional check, AgentGBM has GoalPosition)
            // We assume GoalPosition is set and static for the agent.

            recordedPaths.Clear();

            // 3. Run Trials
            for (int t = 0; t < trialsPerEpisode; t++)
            {
                Debug.Log($"   -> Trial {t + 1}/{trialsPerEpisode}");
                
                // Reset Agent
                agent.gameObject.SetActive(false); 
                agent.transform.position = startPos;
                agent.transform.rotation = startRot;
                agent.Velocity = Vector3.zero; // Reset physics/velocity
                agent.GoalReached = false;     // Reset goal flag
                
                yield return null; // Wait a frame to ensure systems update
                
                agent.gameObject.SetActive(true);

                currentPath.Clear();

                // Wait for Goal Reached
                bool goalReached = false;
                float timeout = 60.0f; // Max duration per trial
                float timer = 0f;

                while (!goalReached && timer < timeout)
                {
                    yield return null; // Wait frame
                    timer += Time.deltaTime;

                    if (agent == null) break;

                    // Record Path
                    Vector3 pos = agent.transform.position;
                    
                    if (currentPath.Count == 0 || Vector3.Distance(currentPath.Last(), pos) > 0.1f)
                    {
                        currentPath.Add(pos);
                    }

                    // Check Goal
                    if (agent.GoalReached) 
                    {
                        goalReached = true;
                    }
                }

                if (!goalReached)
                {
                    Debug.LogWarning($"[DiversityEvaluator] Trial {t+1} timed out for agent {agent.name}");
                }

                // Save path
                recordedPaths.Add(new List<Vector3>(currentPath));
                
                // Small delay between trials
                yield return new WaitForSeconds(0.5f);
            }

            // 4. Calculate Diversity for this Agent
            float diversity = CalculateAveragePairwiseDTW(recordedPaths);
            agentDiversityScores.Add(diversity);
            Debug.Log($"[DiversityEvaluator] Agent {agent.name} Diversity (Avg DTW): {diversity:F4}");
            
            // Ensure agent is deactivated if it wasn't already (to clean up before next agent?)
            // Actually, keep it as is. If we want to restore scene state, it's complex.
            // We leave the agent in its final state of the last trial (likely GoalReached=true, inactive).
        }

        // Final Report
        float finalAvgDiv = agentDiversityScores.Count > 0 ? agentDiversityScores.Average() : 0f;
        string report = $"Final Average Diversity Score (over {agentDiversityScores.Count} agents, {trialsPerEpisode} trials each): {finalAvgDiv:F4}";
        Debug.Log(report);
        
        // Write to file in project root or data path
        string path = Path.Combine(Application.dataPath, "..", outputFileName); // Project root
        try {
            File.WriteAllText(path, report);
            Debug.Log($"Report saved to {path}");
        } catch (System.Exception e) {
            Debug.LogError($"Failed to write report: {e.Message}");
        }

        isRunning = false;
        
#if UNITY_EDITOR
        UnityEditor.EditorApplication.isPlaying = false;
#else
        Application.Quit();
#endif
    }

    // --- DTW Calculation ---

    float CalculateAveragePairwiseDTW(List<List<Vector3>> paths)
    {
        if (paths.Count < 2) return 0f;

        float totalDist = 0f;
        int pairCount = 0;

        for (int i = 0; i < paths.Count; i++)
        {
            for (int j = i + 1; j < paths.Count; j++)
            {
                totalDist += CalculateDTW(paths[i], paths[j]);
                pairCount++;
            }
        }

        return pairCount > 0 ? totalDist / pairCount : 0f;
    }

    float CalculateDTW(List<Vector3> path1, List<Vector3> path2)
    {
        int n = path1.Count;
        int m = path2.Count;
        
        if (n == 0 || m == 0) return 0f;

        float[,] dtw = new float[n + 1, m + 1];

        for (int i = 0; i <= n; i++)
        {
            for (int j = 0; j <= m; j++)
            {
                dtw[i, j] = float.PositiveInfinity;
            }
        }
        dtw[0, 0] = 0f;

        for (int i = 1; i <= n; i++)
        {
            for (int j = 1; j <= m; j++)
            {
                float cost = Vector3.Distance(path1[i - 1], path2[j - 1]);
                dtw[i, j] = cost + Mathf.Min(dtw[i - 1, j],    // Insertion
                                             Mathf.Min(dtw[i, j - 1],    // Deletion
                                                       dtw[i - 1, j - 1] // Match
                                                      ));
            }
        }

        return dtw[n, m];
    }
}
