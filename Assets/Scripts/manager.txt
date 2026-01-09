using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CCPManager : MonoBehaviour
{
    [Header("Agent Settings")]
    public GameObject agentPrefab;
    public bool useGoalOnlyAgent = false;
    public bool disableAgentsOnGoal = true;

    [Header("Spawn Settings")]
    public List<Transform> spawnPoints;
    public List<Transform> goals;

    [Header("Visualization")]
    public bool showTrails = true;
    public float trailWidth = 0.1f;
    public Vector3 trailOffset = new Vector3(0f, 0.5f, 0f);
    public Material trailMaterial;
    public Color[] agentColors = new Color[] {
        Color.cyan, Color.red, Color.green, Color.yellow, Color.magenta, Color.white, new Color(1f, 0.5f, 0f) // Orange
    };

    private List<GameObject> spawnedAgents = new List<GameObject>();

    void Start()
    {
        SpawnAgents();
    }

    public void SpawnAgents()
    {
        // Clear existing agents if any
        foreach (var agent in spawnedAgents)
        {
            if (agent != null) Destroy(agent);
        }
        spawnedAgents.Clear();

        if (spawnPoints == null || goals == null)
        {
            Debug.LogWarning("Spawn Points or Goals list is null in CCPManager.");
            return;
        }

        int count = Mathf.Min(spawnPoints.Count, goals.Count);

        for (int i = 0; i < count; i++)
        {
            if (spawnPoints[i] == null || goals[i] == null) continue;

            // Instantiate
            GameObject agentObj = Instantiate(agentPrefab, spawnPoints[i].position, spawnPoints[i].rotation);

            // Setup Visualization
            if (showTrails)
            {
                SetupTrailRenderer(agentObj, i);
            }

            // Try to set up Agent_Training
            Agent_Training agentTraining = agentObj.GetComponent<Agent_Training>();
            if (agentTraining != null)
            {
                agentTraining.pdmMode = true; // IMPORTANT: Set this to avoid Manager dependency
                agentTraining.GoalTransform = goals[i];
                agentTraining.disableOnGoal = disableAgentsOnGoal;
                agentTraining.SetGoal(goals[i].position);
                agentTraining.ForceReset(spawnPoints[i].position, spawnPoints[i].rotation, Vector3.zero);
            }
            else
            {
                // Try to set up Agent_GoalOnly_Training
                Agent_GoalOnly_Training agentGoalOnly = agentObj.GetComponent<Agent_GoalOnly_Training>();
                if (agentGoalOnly != null)
                {
                    agentGoalOnly.pdmMode = true; // IMPORTANT: Set this to avoid Manager dependency
                    agentGoalOnly.GoalTransform = goals[i];
                    agentGoalOnly.disableOnGoal = disableAgentsOnGoal;
                    agentGoalOnly.SetGoal(goals[i].position);
                    agentGoalOnly.ForceReset(spawnPoints[i].position, spawnPoints[i].rotation, Vector3.zero);
                }
            }

            spawnedAgents.Add(agentObj);
        }
    }

    private void SetupTrailRenderer(GameObject agent, int index)
    {
        // Create a child object for the trail to offset it vertically
        GameObject trailObj = new GameObject("PathTrail");
        trailObj.transform.SetParent(agent.transform);
        trailObj.transform.localPosition = trailOffset;

        TrailRenderer tr = trailObj.AddComponent<TrailRenderer>();

        // Basic Settings
        tr.time = 9999f; // Infinite trail
        tr.startWidth = trailWidth;
        tr.endWidth = trailWidth;
        tr.minVertexDistance = 0.05f; // Smoothness

        // Material
        if (trailMaterial != null)
        {
            tr.material = trailMaterial;
        }
        else
        {
            // Create a default material if none provided to avoid pink texture
            Material defaultMat = new Material(Shader.Find("Sprites/Default"));
            tr.material = defaultMat;
        }

        // Color
        Color c = Color.white;
        if (agentColors != null && agentColors.Length > 0)
        {
            c = agentColors[index % agentColors.Length];
        }

        Gradient gradient = new Gradient();
        gradient.SetKeys(
            new GradientColorKey[] { new GradientColorKey(c, 0.0f), new GradientColorKey(c, 1.0f) },
            new GradientAlphaKey[] { new GradientAlphaKey(1.0f, 0.0f), new GradientAlphaKey(1.0f, 1.0f) }
        );
        tr.colorGradient = gradient;

        // Shadow casting (Optional, usually off for trails)
        tr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
    }
}