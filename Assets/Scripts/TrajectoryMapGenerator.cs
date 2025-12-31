using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class TrajectoryMapGenerator : MonoBehaviour
{
    [Header("Agent Identification")]
    [Tooltip("Layer where agents are placed.")]
    public LayerMask agentLayer;
    [Tooltip("List of agent prefabs to identify agents in the scene.")]
    public List<GameObject> agentPrefabs = new List<GameObject>();
    [Tooltip("Time in seconds between scans for new agents.")]
    public float scanInterval = 1.0f;

    [Header("Recording Settings")]
    public float recordInterval = 0.05f; // Time in seconds between recording points
    public float minDistance = 0.1f;    // Minimum distance required to record a new point

    [Header("Map Settings")]
    public float mapWidth = 100f;
    public float mapHeight = 100f;

    [Header("Image Settings")]
    public int imageResolution = 2048;
    public KeyCode captureKey = KeyCode.T;
    public string saveFileName = "TrajectoryMap.png";
    public Color backgroundColor = Color.white;
    public Color lineColor = Color.red;
    public int lineThickness = 3;
    public int endPointRadius = 10;

    // Internal storage for paths: Agent Transform -> List of recorded positions
    private Dictionary<Transform, List<Vector3>> recordedPaths = new Dictionary<Transform, List<Vector3>>();

    private float recordTimer = 0f;
    private float scanTimer = 0f;

    private void Start()
    {
        ScanForAgents();
    }

    private void Update()
    {
        // 1. Scan for new agents periodically
        scanTimer += Time.deltaTime;
        if (scanTimer >= scanInterval)
        {
            ScanForAgents();
            scanTimer = 0f;
        }

        // 2. Record positions
        recordTimer += Time.deltaTime;
        if (recordTimer >= recordInterval)
        {
            RecordPositions();
            recordTimer = 0f;
        }

        // 3. User Input
        if (Input.GetKeyDown(captureKey))
        {
            GenerateAndSaveImage();
        }
    }

    private void OnApplicationQuit()
    {
        GenerateAndSaveImage();
    }

    private void ScanForAgents()
    {
        // Optimization: Use the layer to quickly filter.
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        int agentLayerValue = agentLayer.value;

        foreach (var go in allObjects)
        {
            // Skip if already tracking
            if (recordedPaths.ContainsKey(go.transform)) continue;

            bool isAgent = CheckIfAgent(go, agentLayerValue);

            if (isAgent)
            {
                // Check if this object is a child of another agent we could track
                // If any parent matches the criteria, assume the parent is the true agent and this is just a part.
                Transform parent = go.transform.parent;
                while (parent != null)
                {
                    if (CheckIfAgent(parent.gameObject, agentLayerValue))
                    {
                        isAgent = false; // It's a child, so ignore it
                        break;
                    }
                    parent = parent.parent;
                }

                if (isAgent)
                {
                    recordedPaths.Add(go.transform, new List<Vector3>());
                }
            }
        }
    }

    private bool CheckIfAgent(GameObject go, int layerValue)
    {
        // 1. Check layer first (most efficient)
        if (layerValue != 0 && ((1 << go.layer) & layerValue) != 0)
        {
            return true;
        }
        // 2. Fallback to name match if layer didn't match or wasn't specified
        else if (agentPrefabs != null && agentPrefabs.Count > 0)
        {
            foreach (var prefab in agentPrefabs)
            {
                if (prefab != null && go.name.Contains(prefab.name))
                {
                    return true;
                }
            }
        }
        return false;
    }

    private void RecordPositions()
    {
        // We must use a list of keys to avoid modification errors if we were removing, 
        // but here we are just iterating. However, if an object is destroyed, we should handle it.

        List<Transform> invalidAgents = new List<Transform>();

        foreach (var kvp in recordedPaths)
        {
            Transform agent = kvp.Key;
            List<Vector3> path = kvp.Value;

            if (agent == null)
            {
                // Agent destroyed
                // We keep the data, but maybe mark it as inactive? 
                // For now, just skip recording. We don't remove the data so we can draw the path later.
                continue;
            }

            if (!agent.gameObject.activeInHierarchy) continue;

            // Use XZ plane only (Y=0)
            Vector3 currentPosFlat = new Vector3(agent.position.x, 0, agent.position.z);

            // Add point if it's the first one or far enough from the last one (2D distance)
            if (path.Count == 0 || Vector3.Distance(path[path.Count - 1], currentPosFlat) >= minDistance)
            {
                path.Add(currentPosFlat);
            }
        }
    }

    public void GenerateAndSaveImage()
    {
        if (recordedPaths.Count == 0)
        {
            Debug.LogWarning("TrajectoryMapGenerator: No agents tracked.");
            return;
        }

        // Collect all valid paths
        List<List<Vector3>> validPaths = new List<List<Vector3>>();
        foreach (var path in recordedPaths.Values)
        {
            if (path != null && path.Count > 0)
            {
                validPaths.Add(path);
            }
        }

        // Use fixed bounds centered at 0,0
        float minX = -mapWidth / 2f;
        float maxX = mapWidth / 2f;
        float minZ = -mapHeight / 2f;
        float maxZ = mapHeight / 2f;

        float widthWorld = mapWidth;
        float heightWorld = mapHeight;

        if (widthWorld <= 0 || heightWorld <= 0) return;

        // Initialize Texture
        Texture2D texture = new Texture2D(imageResolution, imageResolution);
        Color[] resetColor = new Color[imageResolution * imageResolution];
        for (int i = 0; i < resetColor.Length; i++) resetColor[i] = backgroundColor;
        texture.SetPixels(resetColor);

        // Draw Paths
        foreach (var path in validPaths)
        {
            if (path.Count < 2) continue;

            Vector2 prevPixel = WorldToPixel(path[0], minX, minZ, widthWorld, heightWorld);

            for (int i = 1; i < path.Count; i++)
            {
                Vector2 currentPixel = WorldToPixel(path[i], minX, minZ, widthWorld, heightWorld);
                DrawLine(texture, prevPixel, currentPixel, lineColor, lineThickness);
                prevPixel = currentPixel;
            }

            // Draw End Point
            DrawCircle(texture, prevPixel, endPointRadius, lineColor);
        }

        texture.Apply();

        // Save to file
        byte[] bytes = texture.EncodeToPNG();
        string pathToFile = Path.Combine(Application.dataPath, saveFileName); // Save to Assets folder
        File.WriteAllBytes(pathToFile, bytes);

        Debug.Log($"Trajectory Map saved to: {pathToFile}");

#if UNITY_EDITOR
        // Refresh Asset Database to show the file in Unity Editor
        UnityEditor.AssetDatabase.Refresh();
#endif
    }

    private Vector2 WorldToPixel(Vector3 worldPos, float minX, float minZ, float width, float height)
    {
        // Normalize 0-1
        float u = (worldPos.x - minX) / width;
        float v = (worldPos.z - minZ) / height;

        // Map to pixel coords
        return new Vector2(u * (imageResolution - 1), v * (imageResolution - 1));
    }

    private void DrawLine(Texture2D tex, Vector2 p1, Vector2 p2, Color col, int thickness)
    {
        int x0 = (int)p1.x;
        int y0 = (int)p1.y;
        int x1 = (int)p2.x;
        int y1 = (int)p2.y;

        int dx = Mathf.Abs(x1 - x0);
        int dy = Mathf.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            DrawBrush(tex, x0, y0, col, thickness);

            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }

    private void DrawBrush(Texture2D tex, int x, int y, Color col, int thickness)
    {
        int half = thickness / 2;
        for (int i = -half; i <= half; i++)
        {
            for (int j = -half; j <= half; j++)
            {
                if (x + i >= 0 && x + i < tex.width && y + j >= 0 && y + j < tex.height)
                {
                    tex.SetPixel(x + i, y + j, col);
                }
            }
        }
    }

    private void DrawCircle(Texture2D tex, Vector2 center, int radius, Color col)
    {
        int cx = (int)center.x;
        int cy = (int)center.y;

        for (int x = -radius; x <= radius; x++)
        {
            for (int y = -radius; y <= radius; y++)
            {
                if (x * x + y * y <= radius * radius)
                {
                    int px = cx + x;
                    int py = cy + y;
                    if (px >= 0 && px < tex.width && py >= 0 && py < tex.height)
                    {
                        tex.SetPixel(px, py, col);
                    }
                }
            }
        }
    }
}
