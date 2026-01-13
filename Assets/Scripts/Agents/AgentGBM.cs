using UnityEngine;

public class AgentGBM : AgentBase
{
    public bool GoalReached = false;

    void Update()
    {
        if (GetDistanceToGoal() <= 1.0f)
        {
            // Debug.Log($"[AgentGBM] {name} reached goal. Deactivating.");
            
            // Detach TrailRenderer if it exists so the path remains visible
            TrailRenderer tr = GetComponentInChildren<TrailRenderer>();
            if (tr != null)
            {
                tr.transform.SetParent(null);
                tr.autodestruct = false; 
            }

            GoalReached = true;
            
            if (!pdmMode)
            {
                gameObject.SetActive(false);
            }
        }
    }
}