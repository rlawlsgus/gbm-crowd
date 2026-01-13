using System.Collections.Generic;
using UnityEngine;

public abstract class SimulatorBase : MonoBehaviour 
{
	public SimulatorSettings Settings;
	protected Texture2D m_VisionTexture;  // Texture used for retrieving the NPC's visual data.
	public Camera AgentCamera; 	    // Camera to be attached to the NPC's head.
	
    private void Awake()
    {
		if (Settings.VisionShader == null)
		{
			Debug.LogError("Vision shader could not be found!");
		}

		if(AgentCamera == null)
        {
			Debug.LogError("Camera could not be found!");
        }

		// Setup camera and texture for rendering NPC's vision.
		AgentCamera.clearFlags = CameraClearFlags.SolidColor;
		AgentCamera.backgroundColor = Color.black;
		AgentCamera.targetTexture = new RenderTexture (Settings.VisionWidth, Settings.VisionHeight, 24, RenderTextureFormat.ARGBFloat);
		m_VisionTexture = new Texture2D (Settings.VisionWidth, Settings.VisionHeight, TextureFormat.RGBAFloat, false);
	}

	private void SetupAgentCamera(AgentBase agent) 
	{
		// Place camera at NPC's head.
		Vector3 cameraPosition = agent.transform.position;
		cameraPosition.y += agent.transform.localScale.y;
		AgentCamera.transform.position = cameraPosition;
		AgentCamera.transform.rotation = agent.transform.rotation;
		AgentCamera.transform.Rotate(Vector3.up, 90.0f);	
		AgentCamera.transform.Rotate(Vector3.right, Settings.SightTiltAngle); // Looking down 40 degrees.

		// Set shader for rendering the model vision.
		AgentCamera.SetReplacementShader(Settings.VisionShader, "RenderType");
	}

	private Color[] GetAgentVision() 
	{
		m_VisionTexture.ReadPixels( new Rect( 0, 0, Settings.VisionWidth, Settings.VisionHeight ), 0, 0, false );
		m_VisionTexture.Apply();

		return m_VisionTexture.GetPixels();
	}

	public void ProcessAgents<T>(List<T> agents) where T : AgentBase 
	{
		RenderTexture.active = AgentCamera.targetTexture;

		foreach(T agent in agents) 
		{
            if (agent == null || !agent.gameObject.activeInHierarchy) continue;

			SetupAgentCamera(agent);
			SetupVisionShader(agent);

			int originalCullingMask = AgentCamera.cullingMask;
			AgentCamera.cullingMask = 1 << LayerMask.NameToLayer("Obstacle");
			AgentCamera.Render();
			AgentCamera.cullingMask = originalCullingMask;


			// Process and update NPC's properties.
			ProcessAgent (agent, GetAgentVision());
			agent.UpdateMaterial();
		}

		RenderTexture.active = null;
	}

	protected abstract void SetupVisionShader (AgentBase agent);
	protected abstract void ProcessAgent (AgentBase agent, Color[] pixels);
}
