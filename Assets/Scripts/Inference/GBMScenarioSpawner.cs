using UnityEngine;
using System.Collections.Generic;
using System;
using System.IO;

#if UNITY_EDITOR
using UnityEditor;
#endif

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
        public List<AgentScenarioData> agents = new List<AgentScenarioData>();
    }

    [Header("Settings")]
    public GameObject agentPrefab;
    public Transform agentsRoot;
    [Tooltip("Drag and drop the scenario JSON file here.")]
    public TextAsset scenarioJsonFile;
    public bool spawnOnStart = true;
    public bool disableAgentsOnGoal = true;

    [Header("Editor / Runtime Data")]
    public ScenarioExportData currentScenarioData;

    [Header("Runtime Info")]
    public List<AgentGBM> spawnedAgents = new List<AgentGBM>();

    private void Start()
    {
        // If data is empty but we have a file, load it automatically at start
        if ((currentScenarioData == null || currentScenarioData.agents == null || currentScenarioData.agents.Count == 0) && scenarioJsonFile != null)
        {
            LoadScenario();
        }

        if (spawnOnStart)
        {
            SpawnFromScenario();
        }
    }

    [ContextMenu("Load Scenario from JSON")]
    public void LoadScenario()
    {
        if (scenarioJsonFile == null)
        {
            Debug.LogError("[GBMScenarioSpawner] No JSON file assigned!");
            return;
        }

        try
        {
            currentScenarioData = JsonUtility.FromJson<ScenarioExportData>(scenarioJsonFile.text);
            Debug.Log($"[GBMScenarioSpawner] Loaded {currentScenarioData.agents.Count} agents from {scenarioJsonFile.name}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[GBMScenarioSpawner] Failed to parse JSON: {e.Message}");
        }
    }

    [ContextMenu("Save Scenario to JSON")]
    public void SaveScenario()
    {
        if (currentScenarioData == null) return;
        string json = JsonUtility.ToJson(currentScenarioData, true);

#if UNITY_EDITOR
        if (scenarioJsonFile != null)
        {
            string path = AssetDatabase.GetAssetPath(scenarioJsonFile);
            File.WriteAllText(path, json);
            AssetDatabase.ImportAsset(path);
            Debug.Log($"[GBMScenarioSpawner] Scenario saved and updated: {path}");
        }
        else
        {
            string path = EditorUtility.SaveFilePanel("Save Scenario", "Assets", "NewScenario", "json");
            if (!string.IsNullOrEmpty(path))
            {
                File.WriteAllText(path, json);
                AssetDatabase.Refresh();
                Debug.Log($"[GBMScenarioSpawner] Scenario saved to new file: {path}");
            }
        }
#else
        Debug.LogWarning("Saving is only supported in Editor mode.");
#endif
    }

    [ContextMenu("Spawn From Scenario")]
    public void SpawnFromScenario()
    {
        if (currentScenarioData == null || currentScenarioData.agents == null || currentScenarioData.agents.Count == 0)
        {
            Debug.LogWarning("[GBMScenarioSpawner] No scenario data to spawn!");
            return;
        }

        if (agentPrefab == null)
        {
            Debug.LogError("[GBMScenarioSpawner] Agent Prefab is not assigned!");
            return;
        }

        ClearExistingAgents();

        float y = currentScenarioData.worldHeight;

        // Create Goal Root for clean hierarchy
        GameObject goalRoot = new GameObject("Scenario_Goals_Runtime");
        if (agentsRoot != null) goalRoot.transform.SetParent(agentsRoot);

        Dictionary<int, List<AgentGBM>> groupGroups = new Dictionary<int, List<AgentGBM>>();

        for (int i = 0; i < currentScenarioData.agents.Count; i++)
        {
            var agentData = currentScenarioData.agents[i];
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
                InitializeAgent(agentGBM, spawnPos, goalPos, Color.white);
                spawnedAgents.Add(agentGBM);

                // Create Goal Object
                GameObject goalObj = new GameObject($"{agentData.name}_{i}_Goal");
                goalObj.transform.position = goalPos;
                goalObj.transform.SetParent(goalRoot.transform);

                // Grouping
                if (agentData.groupId != 0)
                {
                    if (!groupGroups.ContainsKey(agentData.groupId))
                        groupGroups[agentData.groupId] = new List<AgentGBM>();
                    groupGroups[agentData.groupId].Add(agentGBM);
                }
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

        Debug.Log($"[GBMScenarioSpawner] Successfully spawned {spawnedAgents.Count} agents. Groups: {groupGroups.Count}");
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

        if (agentsRoot != null)
        {
            List<GameObject> toDestroy = new List<GameObject>();
            foreach (Transform child in agentsRoot) toDestroy.Add(child.gameObject);
            foreach (var child in toDestroy)
            {
                if (Application.isPlaying) Destroy(child);
                else DestroyImmediate(child);
            }
        }
    }

    void OnDrawGizmos()
    {
        if (currentScenarioData == null || currentScenarioData.agents == null) return;

        float y = currentScenarioData.worldHeight;
        foreach (var agent in currentScenarioData.agents)
        {
            Vector3 s = new Vector3(agent.startPos.x, y, agent.startPos.y);
            Vector3 g = new Vector3(agent.goalPos.x, y, agent.goalPos.y);

            Gizmos.color = agent.debugColor;
            Gizmos.DrawLine(s, g);
            Gizmos.DrawWireSphere(s, 0.3f);

            Vector3 forward = Quaternion.Euler(0, agent.startRotation, 0) * Vector3.forward;
            Gizmos.DrawLine(s, s + forward * 0.8f);

            Color goalColor = agent.debugColor;
            goalColor.a = 0.5f;
            Gizmos.color = goalColor;
            Gizmos.DrawWireCube(g, new Vector3(0.5f, 0.1f, 0.5f));
        }
    }
}

#if UNITY_EDITOR
[CustomEditor(typeof(GBMScenarioSpawner))]
public class GBMScenarioSpawnerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        GBMScenarioSpawner spawner = (GBMScenarioSpawner)target;

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Scenario Controls", EditorStyles.boldLabel);

        if (GUILayout.Button("Load from JSON File"))
        {
            Undo.RecordObject(spawner, "Load Scenario");
            spawner.LoadScenario();
        }

        if (GUILayout.Button("Save to JSON File"))
        {
            spawner.SaveScenario();
        }

        EditorGUILayout.Space();
        if (GUILayout.Button("Spawn Agents (Preview/Start)"))
        {
            spawner.SpawnFromScenario();
        }

        if (GUILayout.Button("Clear Agents"))
        {
            spawner.ClearExistingAgents();
        }

        if (GUI.changed)
        {
            EditorUtility.SetDirty(spawner);
        }
    }

    private void OnSceneGUI()
    {
        GBMScenarioSpawner spawner = (GBMScenarioSpawner)target;
        if (spawner.currentScenarioData == null || spawner.currentScenarioData.agents == null) return;

        float y = spawner.currentScenarioData.worldHeight;
        bool changed = false;

        for (int i = 0; i < spawner.currentScenarioData.agents.Count; i++)
        {
            var agent = spawner.currentScenarioData.agents[i];

            // 1. Start Position Handle
            Vector3 startPos = new Vector3(agent.startPos.x, y, agent.startPos.y);
            EditorGUI.BeginChangeCheck();
            Vector3 newStartPos = Handles.PositionHandle(startPos, Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(spawner, $"Move Agent {i} Start");
                agent.startPos = new Vector2(newStartPos.x, newStartPos.z);
                changed = true;
            }

            // 2. Goal Position Handle
            Vector3 goalPos = new Vector3(agent.goalPos.x, y, agent.goalPos.y);
            EditorGUI.BeginChangeCheck();
            Vector3 newGoalPos = Handles.PositionHandle(goalPos, Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(spawner, $"Move Agent {i} Goal");
                agent.goalPos = new Vector2(newGoalPos.x, newGoalPos.z);
                changed = true;
            }

            // 3. Rotation Handle
            Vector3 forward = Quaternion.Euler(0, agent.startRotation, 0) * Vector3.forward;
            EditorGUI.BeginChangeCheck();
            Quaternion newRot = Handles.RotationHandle(Quaternion.Euler(0, agent.startRotation, 0), startPos);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(spawner, $"Rotate Agent {i}");
                agent.startRotation = newRot.eulerAngles.y;
                changed = true;
            }

            // Labels
            Handles.Label(startPos + Vector3.up * 1.5f, $"Agent {i} ({agent.name})", EditorStyles.boldLabel);
        }

        if (changed)
        {
            EditorUtility.SetDirty(spawner);
        }
    }
}
#endif
