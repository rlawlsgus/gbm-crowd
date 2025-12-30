using UnityEngine;
using System.Collections.Generic;

public class PureReactiveModel : SimulatorBase 
{
	private static readonly List<AgentPRM> m_Agents = new List<AgentPRM>();
	
	public static void AddAgent(AgentPRM agent)
	{
		m_Agents.Add(agent);
	}

	public static void ClearAgents()
	{
		m_Agents.Clear();
	}

	private void Start()
    {
        AgentCamera.backgroundColor = new Color(0.0f, 0.0f, -1.0f, -1.0f);

		if (m_Agents.Count == 0)
		{
			m_Agents.AddRange(FindObjectsOfType<AgentPRM>());
		}
	}

	protected override void SetupVisionShader(AgentBase agent)
	{
		if (agent is AgentPRM agentPRM)
		{
			AgentPRMSettings settings = agentPRM.Settings as AgentPRMSettings;

			Shader.SetGlobalVector("_AgentPosition", new Vector4(agent.transform.position.x,
																 agent.transform.position.y,
																 agent.transform.position.z));
			Shader.SetGlobalVector("_AgentVelocity", new Vector4(agent.Velocity.x,
																 agent.Velocity.y,
																 agent.Velocity.z));
			Shader.SetGlobalFloat("_A", settings.A);
			Shader.SetGlobalFloat("_B", settings.B);
			Shader.SetGlobalFloat("_C", settings.C);

			Vector3 goalVelocity = agent.GetComfortVelocity();
			Shader.SetGlobalVector("_GoalVelocity", new Vector4(goalVelocity.x, goalVelocity.y, goalVelocity.z));
			Shader.SetGlobalFloat("_Ttg", agent.GetTimeToGoal());
		}
	}

	private void Update()
	{
		ProcessAgents(m_Agents);
	}

	protected override void ProcessAgent (AgentBase agent, Color[] pixels) 
	{
		float phiR   = float.MaxValue;
		float phiL   = float.MinValue;
		float tti    = float.MaxValue;
		float firstR = float.MaxValue;
		float firstL = float.MaxValue;
		
		for(int p = 0; p < pixels.Length; p++) 
		{
			Color pixel = pixels[p];

            if (pixel.b >= 0.0f && pixel.b < 10.0f) 
			{
				float lPhiR   = pixel.r; // thetaDot1
				float lPhiL   = pixel.g; // thetaDot2
				float lTti    = pixel.b; // ttc
				float lFirstR = 1.0f;    // go first if taking thetaDot1
				float lFirstL = 1.0f;    // go first if taking thetaDot2

				// ttc < 3s and giving a way
				if (pixel.b < 3.0f) 
				{
					if(pixel.a <= 0.0f) 
					{
						if (Mathf.Abs(pixel.r) < Mathf.Abs(pixel.g)) 
						{
							lFirstR = -1.0f;
						} else 
						{
							lFirstL = -1.0f;
						}
					} else 
					{
						if (Mathf.Abs(pixel.r) < Mathf.Abs(pixel.g)) 
						{
							lFirstL = -1.0f;
						} else 
						{
							lFirstR = -1.0f;
						}
					}
				}
				
				phiR   = Mathf.Min(phiR,   lPhiR);
				phiL   = Mathf.Max(phiL,   lPhiL);
				tti    = Mathf.Min(tti,    lTti);
				firstR = Mathf.Min(firstR, lFirstR);
				firstL = Mathf.Min(firstL, lFirstL);
			}
		}

		if (tti == float.MaxValue)
		{
			tti = -1.0f;
		}

		float  thetaMin   = 0.0f;
		float  thetaMax   = 0.0f;
		bool   goFirstMin = true;
		bool   goFirstMax = true;
		float  ttcMin     = 10.0f;
		
		float thetaDotValue1 = phiR;
		float thetaDotValue2 = phiL;
		float ttcValue       = tti; 
		
		if(thetaDotValue1 != 0 && ttcValue >= 0) 
		{
			float  thetaDotMinus = thetaDotValue1;
			float  thetaDotPlus  = thetaDotValue2;
			bool    goFirstR     = (firstR > 0.0f) ? true: false;;
			bool    goFirstL     = (firstL > 0.0f) ? true: false;;
			ttcMin               = ttcValue;
			
			if(Mathf.Abs(thetaDotMinus) > Mathf.Abs(thetaDotPlus)) 
			{
				thetaMin   = thetaDotPlus;
				thetaMax   = thetaDotMinus;
				goFirstMin = goFirstL;
				goFirstMax = goFirstR;
			} else 
			{
				thetaMin   = thetaDotMinus;
				thetaMax   = thetaDotPlus;
				goFirstMin = goFirstR;
				goFirstMax = goFirstL;
			}
		}
		
		UpdateVelocityAndPosition(agent, thetaMin, thetaMax, ttcMin, goFirstMin, goFirstMax);
	}

	private void UpdateVelocityAndPosition(AgentBase agent, float thetaMin, float thetaMax, 
	                                       float ttcMin, bool goFirstMin, bool goFirstMax) 
	{
		float newSpeed = agent.Settings.ComfortSpeed;
		Vector3 newOrientation;
		float currentSpeed = agent.Velocity.magnitude;
		Vector3 newVelocity;
		bool goFirst = goFirstMin;
		float thetaDot = thetaMin;
		float ttcThreshold = 3.0f;
		float maxAcc = 2.0f;
		float maxSpeed = 2.0f;
		Vector3 orientation_v =  new Vector3 ( Mathf.Cos (Mathf.Deg2Rad * agent.transform.rotation.eulerAngles.y ), 
		                                       0.0f,
											  -Mathf.Sin (Mathf.Deg2Rad * agent.transform.rotation.eulerAngles.y));

		float alphaDotG  = agent.GetGoalBearingAngleDerivative();
		float distToGoal = agent.GetDistanceToGoal();
		
		if(distToGoal > 0.3f) 
		{
			if ( (thetaMin * alphaDotG > 0.0f && Mathf.Abs(thetaMin) < Mathf.Abs(alphaDotG)) ||
			     (thetaMax * alphaDotG > 0.0f && Mathf.Abs(thetaMax) < Mathf.Abs(alphaDotG)) ||
			    ttcMin < 0.0f || ttcMin >= 10.0f) 
			{
				thetaDot = alphaDotG;
			}
			else if (ttcMin > 0.5f) // else avoid collisions
			{ 
				float t1 = Mathf.Abs(thetaMin - alphaDotG);
				float t2 = Mathf.Abs(thetaMax - alphaDotG);
				
				if ( (t1 < 0.08f && t2 < 0.08f) || t1 <= t2 ) 
				{
					thetaDot = thetaMin;
				} else 
				{
					thetaDot = thetaMax;
					goFirst = goFirstMax;
				}
			}
			
			if (ttcMin < ttcThreshold && !goFirst) 
			{
				newSpeed *= ((1 - Mathf.Pow(2.718f , -(ttcMin*ttcMin)*0.4f)));
				newSpeed = (newSpeed < 0.5f) ? 0.0f : newSpeed;
			}
			
			// limit turning speed
			if(currentSpeed > 0.0f && newSpeed != 0.0f) 
			{
				float thetaDotOld = agent.AngularSpeed;
				thetaDot = thetaDotOld + Mathf.Clamp(thetaDot - thetaDotOld, -0.5f, 0.5f);
			}

			agent.AngularSpeed = thetaDot;

			newOrientation = Quaternion.Euler(0.0f, thetaDot * Mathf.Rad2Deg, 0.0f) * orientation_v;

			if(newSpeed <= 0.0f) 
			{
				newSpeed = 0;
				newOrientation = agent.GetDirectionToGoal();
			}

		} else 
		{
			newSpeed = 0;
			newOrientation = agent.GetDirectionToGoal();
		}
		
		newVelocity = newOrientation * newSpeed;
		
		Vector3 acc = newVelocity - agent.Velocity;
		
		if (acc.magnitude > maxAcc * Time.deltaTime) 
		{
			newVelocity = agent.Velocity + 
						  acc.normalized * maxAcc * Time.deltaTime;
		}
		
		newSpeed = Mathf.Min(maxSpeed, Mathf.Min(distToGoal, newVelocity.magnitude));
		
		newOrientation = newSpeed > 0.0f ? newVelocity.normalized : newOrientation;

		//update
		agent.transform.rotation = Quaternion.FromToRotation (orientation_v, newOrientation) * agent.transform.rotation;

		orientation_v = new Vector3 ( Mathf.Cos (Mathf.Deg2Rad * agent.transform.rotation.eulerAngles.y ), 
									  0.0f,
		                             -Mathf.Sin (Mathf.Deg2Rad * agent.transform.rotation.eulerAngles.y));

		agent.Velocity = orientation_v * newSpeed;
		agent.transform.position += agent.Velocity * Time.deltaTime;
	}
}
