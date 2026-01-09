using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VisualizationManager : MonoBehaviour
{
    [Header("Visualization")]
    public bool showTrails = true;
    public float trailWidth = 0.1f;
    public Vector3 trailOffset = new Vector3(0f, 0.5f, 0f);
    public Material trailMaterial;
    public Color[] agentColors = new Color[] {
        Color.cyan, Color.red, Color.green, Color.yellow, Color.magenta, Color.white, new Color(1f, 0.5f, 0f) // Orange
    };

    void Start()
    {
        if (showTrails)
        {
            ApplyTrails();
        }
    }

    void ApplyTrails()
    {
        AgentGBM[] agents = FindObjectsOfType<AgentGBM>();
        for (int i = 0; i < agents.Length; i++)
        {
            SetupTrailRenderer(agents[i].gameObject, i);
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
            // Using a standard shader that supports vertex colors
            Shader shader = Shader.Find("Sprites/Default"); 
            if (shader == null) shader = Shader.Find("Particles/Standard Unlit");
            if (shader == null) shader = Shader.Find("Standard"); // Last resort
            
            Material defaultMat = new Material(shader);
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
