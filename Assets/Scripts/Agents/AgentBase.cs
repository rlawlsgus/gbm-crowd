using UnityEngine;

public class AgentBase : MonoBehaviour 
{
	public AgentSettings Settings;
	public Vector3 GoalPosition;
	public Vector3 Velocity;
	public float AngularSpeed;
	public Color Color = Color.white;
	public Renderer Renderer;
	public bool pdmMode = false;

	float OrientedAngle(Vector2 v1, Vector2 v2) 
	{
		float dot = v1.x * v2.x + v1.y * v2.y;
		float det = v1.x * v2.y - v1.y * v2.x;
		return Mathf.Atan2(det,dot);
	}

	public float GetDistanceToGoal() 
	{
		return ((new Vector3(transform.position.x,0.0f,transform.position.z)) - GoalPosition).magnitude;
	}

	public float GetGoalBearingAngle() 
	{
		Vector2 agent_position = new Vector2(transform.position.x, transform.position.z);
		Vector2 goal_position  = new Vector2(GoalPosition.x, GoalPosition.z);
		Vector2 orientation_v  = new Vector2 (Mathf.Cos (Mathf.Deg2Rad * transform.rotation.eulerAngles.y), 
		                                     -Mathf.Sin (Mathf.Deg2Rad * transform.rotation.eulerAngles.y));
		
		Vector2 relativePosition = goal_position - agent_position;

		return -OrientedAngle(orientation_v, relativePosition.normalized);
	}

	public float GetGoalBearingAngleDerivative() 
	{
		Vector2 agent_position = new Vector2(transform.position.x, transform.position.z);
		Vector2 goal_position  = new Vector2(GoalPosition.x, GoalPosition.z);
		Vector2 orientation_v  = new Vector2 (Mathf.Cos (Mathf.Deg2Rad * transform.rotation.eulerAngles.y), 
		                                      -Mathf.Sin (Mathf.Deg2Rad * transform.rotation.eulerAngles.y));

		Vector2 vectorFromAgentToGoal = goal_position - agent_position;
		Vector2 goalDir     = vectorFromAgentToGoal.normalized;

		// Vector from agent to goal after movement.
		Vector2 goalDirN = (vectorFromAgentToGoal - (orientation_v * Settings.ComfortSpeed)).normalized;
		
		// Bearing angle related to goal.
		float alpha  = - OrientedAngle(orientation_v, goalDir);

		// Bearing angle related to goal after movement.
		float alphaN = - OrientedAngle(orientation_v, goalDirN);

		// Time derivative of bearing goal
		float alphaDotGoal = FixAngleValue(alphaN - alpha);
		
		if(Vector2.Dot(goalDir, orientation_v) <= 0.0f)
			alphaDotGoal += (alphaDotGoal > 0.0f) ? Mathf.PI/2.0f : - Mathf.PI/2.0f;
		
		return alphaDotGoal;
	}

	public float GetTimeToGoal() 
	{
		Vector2 agent_position = new Vector2(transform.position.x, transform.position.z);
		Vector2 goal_position  = new Vector2(GoalPosition.x, GoalPosition.z);
		Vector2 agent_velocity = new Vector2(Velocity.x    , Velocity.z);
		
		float ttg = 1e30f;
		
		Vector2 vectorFromAgentToGoal = goal_position - agent_position;
		float distanceToGoal = vectorFromAgentToGoal.magnitude;
		Vector2 goalDir     = vectorFromAgentToGoal.normalized;
		
		// Projection of agentVelocity on goalDir.
		float speedAlpha = Vector2.Dot(agent_velocity, goalDir);
		
		// time = distance / speed.
		if (speedAlpha > 0.0)
			ttg = distanceToGoal / speedAlpha;

		return ttg;
	}

	public Vector3 GetComfortVelocity() 
	{
		return GetDirectionToGoal () * Settings.ComfortSpeed;
	}

	public Vector3 GetDirectionToGoal() 
	{
		return ((new Vector3 (transform.position.x, 0.0f, transform.position.z)) - GoalPosition).normalized;
	}

	private float interpolate( float val, float y0, float x0, float y1, float x1 ) 
	{
		return (val-x0)*(y1-y0)/(x1-x0) + y0;
	}

	private float mbase( float val ) 
	{
		if ( val <= -0.75f ) return 0.0f;
		else if ( val <= -0.25f ) return interpolate( val, 0.0f, -0.75f, 1.0f, -0.25f );
		else if ( val <= 0.25f ) return 1.0f;
		else if ( val <= 0.75f ) return interpolate( val, 1.0f, 0.25f, 0.0f, 0.75f );
		else return 0.0f;
	}
	
	private float red( float gray ) 
	{
		return mbase( gray - 0.5f );
	}
	
	private float green( float gray ) 
	{
		return mbase( gray );
	}
	
	private float blue( float gray ) 
	{
		return mbase( gray + 0.5f );
	}

	public void UpdateMaterial () 
	{
		Renderer.material.SetVector("_ObstacleVelocity", new Vector4(Velocity.x, Velocity.y, Velocity.z));
		Renderer.material.SetFloat("_ObstacleID", 1.0f);

		if (Settings.SpeedColor) 
		{
			float delta = 0.5f;
			float maxSpeed = delta * Settings.ComfortSpeed;
			float minSpeed = -maxSpeed;
			
			float agentSpeed = Velocity.magnitude;
			
			float normalizedSpeed = (agentSpeed - Settings.ComfortSpeed) / (delta * Settings.ComfortSpeed);
			
			normalizedSpeed = Mathf.Max(normalizedSpeed, minSpeed);
			normalizedSpeed = Mathf.Min(normalizedSpeed, maxSpeed);

			Color = new Vector4(red(normalizedSpeed), green(normalizedSpeed), blue(normalizedSpeed), 1.0f);
		} 

		Renderer.material.SetColor ("_Color", Color);
	}

	private float FixAngleValue(float angle)
	{
		return (Mathf.Abs(angle) > Mathf.PI) ? angle - Mathf.Sign(angle) * 2.0f * Mathf.PI : angle;
	}

}
