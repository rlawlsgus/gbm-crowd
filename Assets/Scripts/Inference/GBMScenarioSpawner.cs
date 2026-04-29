using UnityEngine;
using System.Collections.Generic;
using System;

public class GBMScenarioSpawner : MonoBehaviour
{
    [Serializable]
    public class AgentScenarioData
    {
        public string name = "Agent";
        public Vector2 startPos; // 2D planar position
        public float startRotation; // 2D planar rotation (degrees)
        public Vector2 goalPos;  // 2D planar position
        public int groupId = 0;
        public Color debugColor = Color.cyan;
    }

    [Serializable]
    public class ScenarioExportData
    {
        public float worldHeight;
        public List<AgentScenarioData> agents;
    }

    [Header("Settings")]
    public GameObject agentPrefab;
    public Transform agentsRoot;
    [Tooltip("Drag and drop the scenario JSON file here.")]
    public TextAsset scenarioJsonFile;
    public bool spawnOnStart = true;
    public bool disableAgentsOnGoal = true;

    [Header("Runtime Info")]
    public List<AgentGBM> spawnedAgents = new List<AgentGBM>();

    private ScenarioExportData lastGizmoData;
    private TextAsset lastJsonFile;

    private void Start()
    {
        if (spawnOnStart)
        {
            SpawnFromScenario();
        }
    }

    void OnDrawGizmos()
    {
        if (Application.isPlaying || scenarioJsonFile == null) return;

        // Cache parsing to avoid heavy processing every frame in edit mode
        if (lastGizmoData == null || lastJsonFile != scenarioJsonFile)
        {
            try
            {
                lastGizmoData = JsonUtility.FromJson<ScenarioExportData>(scenarioJsonFile.text);
                lastJsonFile = scenarioJsonFile;
            }
            catch
            {
                return; // Silently fail if JSON is invalid while typing/editing
            }
        }

        if (lastGizmoData == null || lastGizmoData.agents == null) return;

        float y = lastGizmoData.worldHeight;
        foreach (var agent in lastGizmoData.agents)
        {
            Vector3 s = new Vector3(agent.startPos.x, y, agent.startPos.y);
            Vector3 g = new Vector3(agent.goalPos.x, y, agent.goalPos.y);

            Gizmos.color = agent.debugColor;
            
            // Draw line between start and goal
            Gizmos.DrawLine(s, g);

            // Draw Start Point (Sphere + Vertical Line)
            Gizmos.DrawWireSphere(s, 0.3f);
            Gizmos.DrawLine(s, s + Vector3.up * 1.5f);

            // Draw Forward direction indicator
            Vector3 forward = Quaternion.Euler(0, agent.startRotation, 0) * Vector3.forward;
            Gizmos.DrawLine(s, s + forward * 0.8f);
            Gizmos.DrawWireSphere(s + forward * 0.8f, 0.05f);

            // Draw Goal Point (Wire Cube + Short Vertical Line)
            Color goalColor = agent.debugColor;
            goalColor.a = 0.5f;
            Gizmos.color = goalColor;
            Gizmos.DrawWireCube(g, new Vector3(0.5f, 0.1f, 0.5f));
            Gizmos.DrawLine(g, g + Vector3.up * 0.5f);
        }
    }

    [ContextMenu("Spawn From Scenario")]
    public void SpawnFromScenario()
    {
        if (scenarioJsonFile == null)
        {
            Debug.LogError("[GBMScenarioSpawner] Scenario JSON file is not assigned in the inspector!");
            return;
        }

        if (agentPrefab == null)
        {
            Debug.LogError("[GBMScenarioSpawner] Agent Prefab is not assigned!");
            return;
        }

        string json = scenarioJsonFile.text;
        ScenarioExportData data = JsonUtility.FromJson<ScenarioExportData>(json);

        if (data == null || data.agents == null)
        {
            Debug.LogError($"[GBMScenarioSpawner] Failed to parse scenario data from {scenarioJsonFile.name}");
            return;
        }

        ClearExistingAgents();

        float y = data.worldHeight;
        
        // Create Goal Root for clean hierarchy
        GameObject goalRoot = new GameObject("Scenario_Goals_Runtime");
        if (agentsRoot != null) goalRoot.transform.SetParent(agentsRoot);

        Dictionary<int, List<AgentGBM>> groupGroups = new Dictionary<int, List<AgentGBM>>();

        for (int i = 0; i < data.agents.Count; i++)
        {
            var agentData = data.agents[i];
            Vector3 spawnPos = new Vector3(agentData.startPos.x, y, agentData.startPos.y);
            Vector3 goalPos = new Vector3(agentData.goalPos.x, y, agentData.goalPos.y);

            GameObject go = Instantiate(agentPrefab, spawnPos, Quaternion.Euler(0, agentData.startRotation, 0));
            go.name = $"{agentData.name}_{i}_Sim";
            
            if (agentsRoot != null)
            {
                go.transform.SetParent(agentsRoot);
            }

            AgentGBM agentGBM = go.GetComponent<AgentGBM>();
            if (agentGBM != null)
            {
                agentGBM.agentIndex = i;
                InitializeAgent(agentGBM, spawnPos, goalPos, agentData.debugColor);
                spawnedAgents.Add(agentGBM);

                // Create Goal Object (optional but helpful for visualization)
                GameObject goalObj = new GameObject($"{agentData.name}_{i}_Goal");
                goalObj.transform.position = goalPos;
                goalObj.transform.SetParent(goalRoot.transform);

                // Grouping (0 is ignored as per requirement)
                if (agentData.groupId != 0)
                {
                    if (!groupGroups.ContainsKey(agentData.groupId)) 
                        groupGroups[agentData.groupId] = new List<AgentGBM>();
                    groupGroups[agentData.groupId].Add(agentGBM);
                }
            }
            else
            {
                Debug.LogWarning($"[GBMScenarioSpawner] Agent {go.name} is missing AgentGBM component!");
            }
        }

        // Mutual Group Member Registration
        foreach (var group in groupGroups.Values)
        {
            if (group.Count <= 1) continue;
            foreach (var agent in group)
            {
                foreach (var member in group)
                {
                    if (agent == member) continue;
                    if (!agent.groupMembers.Contains(member.transform))
                        agent.groupMembers.Add(member.transform);
                }
            }
        }

        Debug.Log($"[GBMScenarioSpawner] Successfully spawned {spawnedAgents.Count} agents from {scenarioJsonFile.name}. Groups: {groupGroups.Count}");
    }

    private void InitializeAgent(AgentGBM agent, Vector3 spawnPos, Vector3 goalPos, Color color)
    {
        agent.pdmMode = !disableAgentsOnGoal;
        agent.GoalPosition = goalPos;
        agent.transform.position = spawnPos;
        agent.Velocity = Vector3.zero;
        agent.GoalReached = false;
        agent.Color = color;

        // Add to the simulation model
        GradientBasedModel.AddAgent(agent);
    }

    [ContextMenu("Clear Agents")]
    public void ClearExistingAgents()
    {
        GradientBasedModel.ClearAgents();

        foreach (var agent in spawnedAgents)
        {
            if (agent != null)
            {
                if (Application.isPlaying) Destroy(agent.gameObject);
                else DestroyImmediate(agent.gameObject);
            }
        }
        spawnedAgents.Clear();

        // Also clear children of agentsRoot just in case
        if (agentsRoot != null)
        {
            List<GameObject> toDestroy = new List<GameObject>();
            foreach (Transform child in agentsRoot)
            {
                toDestroy.Add(child.gameObject);
            }
            foreach (var child in toDestroy)
            {
                if (Application.isPlaying) Destroy(child);
                else DestroyImmediate(child);
            }
        }
    }
}
