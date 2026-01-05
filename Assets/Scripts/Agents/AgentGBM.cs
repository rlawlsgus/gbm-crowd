using UnityEngine;

public class AgentGBM : AgentBase
{
    public bool GoalReached = false;

    void Update()
    {
        if (GetDistanceToGoal() <= 1.0f)
        {
            // Debug.Log($"[AgentGBM] {name} reached goal. Deactivating.");
            GoalReached = true;
            gameObject.SetActive(false);
        }
    }
}
