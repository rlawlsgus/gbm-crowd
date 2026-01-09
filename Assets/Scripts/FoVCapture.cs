using UnityEngine;
using System.IO;

[RequireComponent(typeof(Camera))]
public class FoVCapture : MonoBehaviour
{
    private Camera fovCamera;
    public string LastSavedImagePath { get; private set; }
    void Awake()
    {
        fovCamera = GetComponent<Camera>();
    }

    public void Capture(int agentId, int timeStep, int width = 512, int height = 512)
    {
        // Create a RenderTexture
        RenderTexture rt = new RenderTexture(width, height, 24);
        fovCamera.targetTexture = rt;

        // Render the camera's view
        Texture2D screenShot = new Texture2D(width, height, TextureFormat.RGB24, false);
        fovCamera.Render();
        RenderTexture.active = rt;
        screenShot.ReadPixels(new Rect(0, 0, width, height), 0, 0);

        // Reset camera and clean up RenderTexture
        fovCamera.targetTexture = null;
        RenderTexture.active = null;
        Destroy(rt);

        // Encode to JPG
        byte[] bytes = screenShot.EncodeToJPG();
        Destroy(screenShot);

        // --- File Path and Saving ---
        // Create a unique path for each agent's images
        string directoryPath = Path.Combine(Application.dataPath, "ThirdViewImages", $"zara01_{agentId}");
        Directory.CreateDirectory(directoryPath);

        // Define the full file path
        string fileName = $"t_{timeStep}.jpg";
        string fullPath = Path.Combine(directoryPath, fileName);

        // Write the file
        File.WriteAllBytes(fullPath, bytes);

        // Store the relative path for the CSV file
        LastSavedImagePath = Path.Combine("ThirdViewImages", $"zara01_{agentId}", fileName).Replace("\\", "/");

    }
}
