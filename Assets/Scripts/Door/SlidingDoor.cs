using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// A sliding door script that follows precise agent detection and movement logic.
/// </summary>
public class SlidingDoor : MonoBehaviour
{
    [Header("Door Panels")]
    public Transform leftDoor;
    public Transform rightDoor;

    [Header("Movement Settings")]
    public Vector3 leftDoorOpenOffset = new Vector3(-1.2f, 0, 0);
    public Vector3 rightDoorOpenOffset = new Vector3(1.2f, 0, 0);
    public float moveSpeed = 3f;
    [Tooltip("Time to wait before closing when no movement or agents are detected.")]
    public float closeDelay = 1.0f;

    [Header("Detection Settings")]
    public string detectionTag = "Agent";
    [Tooltip("Speed threshold to consider an agent as 'moving'.")]
    public float minMovementSpeed = 0.1f;

    public bool isOpen { get; private set; } = false;
    private float closeTimer = 0f;

    private List<Collider> agentsInZone = new List<Collider>();
    private Dictionary<Collider, Vector3> lastPositions = new Dictionary<Collider, Vector3>();

    // Position properties used by WorldModelStateProvider
    public Vector3 leftDoorClosedPos { get; private set; }
    public Vector3 rightDoorClosedPos { get; private set; }
    public Vector3 leftDoorOpenPos { get; private set; }
    public Vector3 rightDoorOpenPos { get; private set; }

    private void Start()
    {
        if (leftDoor != null)
        {
            leftDoorClosedPos = leftDoor.localPosition;
            leftDoorOpenPos = leftDoorClosedPos + leftDoorOpenOffset;
        }
        if (rightDoor != null)
        {
            rightDoorClosedPos = rightDoor.localPosition;
            rightDoorOpenPos = rightDoorClosedPos + rightDoorOpenOffset;
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag(detectionTag) || other.name.StartsWith("ped_") || other.name.StartsWith("GhostAgent"))
        {
            if (!agentsInZone.Contains(other))
            {
                agentsInZone.Add(other);
                lastPositions[other] = other.transform.position;
                isOpen = true;
                closeTimer = 0f;
            }
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (agentsInZone.Contains(other))
        {
            agentsInZone.Remove(other);
            lastPositions.Remove(other);
        }
    }

    private void Update()
    {
        bool anyMoving = false;

        for (int i = agentsInZone.Count - 1; i >= 0; i--)
        {
            Collider agent = agentsInZone[i];

            if (agent == null || !agent.gameObject.activeInHierarchy)
            {
                agentsInZone.RemoveAt(i);
                if (agent != null) lastPositions.Remove(agent);
                continue;
            }

            Vector3 currentPos = agent.transform.position;
            if (lastPositions.TryGetValue(agent, out Vector3 lastPos))
            {
                float distance = Vector3.Distance(currentPos, lastPos);
                float speed = Time.deltaTime > 0 ? distance / Time.deltaTime : 0;

                if (speed > minMovementSpeed)
                {
                    anyMoving = true;
                }
            }
            lastPositions[agent] = currentPos;
        }

        if (anyMoving)
        {
            isOpen = true;
            closeTimer = 0f;
        }
        else
        {
            if (isOpen)
            {
                closeTimer += Time.deltaTime;
                if (closeTimer >= closeDelay)
                {
                    isOpen = false;
                }
            }
        }

        float t = Time.deltaTime * moveSpeed;
        if (leftDoor != null)
        {
            Vector3 target = isOpen ? leftDoorOpenPos : leftDoorClosedPos;
            leftDoor.localPosition = Vector3.Lerp(leftDoor.localPosition, target, t);
        }
        if (rightDoor != null)
        {
            Vector3 target = isOpen ? rightDoorOpenPos : rightDoorClosedPos;
            rightDoor.localPosition = Vector3.Lerp(rightDoor.localPosition, target, t);
        }
    }

    private void OnDrawGizmos()
    {
        BoxCollider box = GetComponent<BoxCollider>();
        if (box != null)
        {
            Gizmos.color = new Color(0, 1, 0, 0.2f);
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.DrawCube(box.center, box.size);
        }
    }
}
