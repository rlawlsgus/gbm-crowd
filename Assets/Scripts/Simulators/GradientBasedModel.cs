using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class GradientBasedModel : SimulatorBase 
{
	// Constants
	const float k_ExpNeg05       = 0.6065306597126334f;
	const float k_Log05BExpNeg05 = 1.3862943611198906f; 
	const float k_MaxDV 		 = 0.5f;
	const float k_MaxDT 		 = Mathf.PI/4.0f;
	const float k_InvMaxDV 		 = 1.0f/k_MaxDV;
	const float k_InvMaxDT	     = 1.0f/k_MaxDT;

	private static readonly List<AgentGBM> m_Agents = new List<AgentGBM>();

	public static void AddAgent(AgentGBM agent)
    {
		m_Agents.Add(agent);
    }

	public static void ClearAgents()
    {
		m_Agents.Clear();
    }

	private void Start()
	{
		if (m_Agents.Count == 0)
		{
			m_Agents.AddRange(FindObjectsOfType<AgentGBM>());
		}
	}

    protected override void SetupVisionShader(AgentBase agent)
	{
		if (agent is AgentGBM agentGBM)
		{
			AgentGBMSettings settings = agentGBM.Settings as AgentGBMSettings;

			Shader.SetGlobalVector("_AgentPosition", new Vector4(agent.transform.position.x,
																 agent.transform.position.y,
																 agent.transform.position.z));
			Shader.SetGlobalVector("_AgentVelocity", new Vector4(agent.Velocity.x,
																 agent.Velocity.y,
																 agent.Velocity.z));
			Shader.SetGlobalFloat("_DistanceToGoal", agent.GetDistanceToGoal());
			Shader.SetGlobalFloat("_SigmaTtca", settings.SigmaTtca);
			Shader.SetGlobalFloat("_SigmaDca", settings.SigmaDca);
		}
	}

    private void Update()
    {
		ProcessAgents(m_Agents);
    }

    protected override void ProcessAgent (AgentBase agent, Color[] pixels) 
	{
		Vector2 dC = Vector2.zero; // dC(dV,dT)
		int pixelCount = 0;
		
		int rightPixelsCount = 0;
		float rightdCdT 	 = 0.0f;

		Vector2 obstacledC     = Vector2.zero; // obstacledC(dV,dT)
		int obstaclePixelCount = 0;
		Vector2 moffset 	   = new Vector2(10.0f, 10.0f);

		for(int p = 0; p < pixels.Length; p++) 
		{
			Color pixel = pixels[p];

			Vector2 pixeldC = new Vector2(pixel.r, pixel.g); // pixeldC(dV,dT)

			if(pixeldC == Vector2.zero)
				continue;
			
			if(pixeldC.x > 0.0f) 
			{
				pixeldC -= moffset;
				dC 		+= pixeldC;
				pixelCount++;
			} else 
			{
				pixeldC    += moffset;
				obstacledC += pixeldC;
				obstaclePixelCount++;
			}
			
			// Separating by sign.
			if(pixeldC.y >= 0.0f) 
			{
				rightPixelsCount++;
				rightdCdT += pixeldC.y;
			}
		}
		
		dC = (pixelCount > 0) ? dC / pixelCount : Vector2.zero;
		dC += (obstaclePixelCount > 0) ? obstacledC / obstaclePixelCount : Vector2.zero;
		rightdCdT = (rightPixelsCount > 0) ? rightdCdT/rightPixelsCount : 0.0f;
		
		if(Mathf.Abs(dC.y) < 1e-1f && Mathf.Abs(rightdCdT) > 1e-2f)
			dC.y += Mathf.Sign(dC.y) * (75e-3f + Random.value * 5e-2f);
		
		// Update Agent Properties
		float distanceToGoal    = agent.GetDistanceToGoal();
		float goalBearingAngle  = agent.GetGoalBearingAngle();
		float agentSpeed        = agent.Velocity.magnitude;
		float agentComfortSpeed = agent.Settings.ComfortSpeed;

		Vector2 d = new Vector2 (-agentSpeed, 0.0f); // (dV,dTheta)

		if(distanceToGoal > 0.3f) 
		{
			float A = 1.0f;
			
			/* **************************** */
			/* Compute gradient due to Goal */
			/*   (Sum of two Gaussians)     */
			/* **************************** */

			Vector2 dG;
			float A_speed = A;
			float A_orientation = A;

			AgentGBMSettings settings = agent.Settings as AgentGBMSettings;

			float deltaSpeed = agentSpeed - agentComfortSpeed;
			float aux = deltaSpeed / Mathf.Pow(settings.SigmaSpeed, 2.0f);
			dG.x = A_speed * Mathf.Pow(k_ExpNeg05, deltaSpeed * aux + k_Log05BExpNeg05) * aux;
			
			aux = goalBearingAngle / Mathf.Pow(settings.SigmaAlpha, 2.0f);
			dG.y = - A_orientation * Mathf.Pow(k_ExpNeg05, goalBearingAngle * aux + k_Log05BExpNeg05) * aux;
			
			/* ********************************** */
			/* Adding obstacles cost to the total */
			/* ********************************** */
			
			float lambda = 1;

			d = -lambda * (dG - dC);

			/* ********************************** */
			/*          Clampling gradient        */
			/* ********************************** */
			float dVratio = Mathf.Abs(d.x * k_InvMaxDV);
			float dTratio = Mathf.Abs(d.y * k_InvMaxDT);
			
			if(dVratio > 1.0f || dTratio > 1.0f) 
			{
				if(dVratio > dTratio) 
				{
					d.y /= dVratio;
					d.x = Mathf.Sign(d.x) * k_MaxDV;
				} else 
				{
					d.x /= dTratio;
					d.y = Mathf.Sign(d.y) * k_MaxDT;
				}
			}
		}
		
		//update
		agent.transform.Rotate(Vector3.up, Mathf.Rad2Deg * d.y);
		
		Vector3 orientation_v = new Vector3 ( Mathf.Cos (Mathf.Deg2Rad * agent.transform.rotation.eulerAngles.y ), 
		                                     0.0f,
		                                     -Mathf.Sin (Mathf.Deg2Rad * agent.transform.rotation.eulerAngles.y));
		
		Vector3 desiredVelocity = Mathf.Max(0.0f, agent.Velocity.magnitude + d.x) * 
			orientation_v;
		
		agent.Velocity = desiredVelocity;
		agent.transform.position += agent.Velocity * Time.deltaTime;
	}
}
