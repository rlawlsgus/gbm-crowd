using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;

public enum VehicleType
{
    Car,
    Bus
}

public class CarController : MonoBehaviour
{
    [Header("Vehicle Settings")]
    public VehicleType vehicleType = VehicleType.Car;

    [Header("Path Following")]
    public SplineContainer splineContainer;
    public int splineIndex = 0;
    public float speed = 5f;
    public float acceleration = 2f;
    public float brakingDeceleration = 5f;
    
    [Header("Detection")]
    public float detectionDistance = 15f;
    public float stopDistance = 8f;
    public string obstacleTag = "Vehicle";
    public Vector3 rayOffset = new Vector3(0, 1, 2);

    [Header("Current State")]
    [SerializeField] private float currentSpeed;
    [SerializeField] private float distanceTraveled;
    private bool isStoppedByObstacle;
    public bool isStoppedByLight;
    private float startDelay;
    private float timeElapsed;
    private float actualStopDistance;

    // Is the vehicle currently at a designated bus stop? Set by BusStopManager.
    public bool isAtBusStop { get; set; }

    /// <summary>
    /// Public getter for the current speed of the vehicle.
    /// </summary>
    public float CurrentSpeed => currentSpeed;

    public System.Action OnReachEnd;

    private void Awake()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb == null) rb = gameObject.AddComponent<Rigidbody>();
        rb.isKinematic = true;
        rb.useGravity = false;
    }

    private void Start()
    {
        // Initial setup if not managed by Pool
        if (OnReachEnd == null)
        {
            ResetPath();
        }
    }

    public void ResetPath()
    {
        distanceTraveled = 0f;
        currentSpeed = 0f;
        timeElapsed = 0f;
        startDelay = UnityEngine.Random.Range(0f, 1f);
        
        // 차마다 다른 정지 간격 (버스는 덩치가 크므로 더 길게)
        float baseStop = (vehicleType == VehicleType.Bus) ? stopDistance + 4f : stopDistance;
        actualStopDistance = baseStop + UnityEngine.Random.Range(-1.0f, 1.0f);
        
        isStoppedByLight = false;
        isStoppedByObstacle = false;
        currentObstacleTargetSpeed = speed;
    }

    private void Update()
    {
        if (splineContainer == null) return;

        timeElapsed += Time.deltaTime;
        if (timeElapsed < startDelay) return;

        HandleObstacleDetection();
        HandleSpeed();
        MoveAlongSpline();
    }

    private void HandleObstacleDetection()
    {
        Vector3 rayStart = transform.position + transform.TransformDirection(rayOffset);
        Vector3 rayDir = transform.forward;

        RaycastHit[] hits = Physics.RaycastAll(rayStart, rayDir, detectionDistance);
        
        float closestDistance = float.MaxValue;
        bool foundValidObstacle = false;

        foreach (var h in hits)
        {
            if (h.transform == transform || h.transform.IsChildOf(transform)) continue;

            if (h.collider.CompareTag(obstacleTag))
            {
                if (h.distance < closestDistance)
                {
                    closestDistance = h.distance;
                    foundValidObstacle = true;
                }
            }
        }

        if (foundValidObstacle)
        {
            if (closestDistance <= actualStopDistance)
            {
                isStoppedByObstacle = true;
                currentObstacleTargetSpeed = 0f;
            }
            else
            {
                isStoppedByObstacle = false;
                float speedFactor = (closestDistance - actualStopDistance) / (detectionDistance - actualStopDistance);
                currentObstacleTargetSpeed = speed * speedFactor;
            }
        }
        else
        {
            isStoppedByObstacle = false;
            currentObstacleTargetSpeed = speed;
        }
    }

    [SerializeField] private float currentObstacleTargetSpeed;

    private void HandleSpeed()
    {
        float targetSpeed = isStoppedByLight ? 0f : currentObstacleTargetSpeed;

        if (currentSpeed < targetSpeed)
        {
            currentSpeed = Mathf.MoveTowards(currentSpeed, targetSpeed, acceleration * Time.deltaTime);
        }
        else if (currentSpeed > targetSpeed)
        {
            float decel = (targetSpeed < currentSpeed) ? brakingDeceleration : acceleration;
            currentSpeed = Mathf.MoveTowards(currentSpeed, targetSpeed, decel * Time.deltaTime);
        }
    }

    private void MoveAlongSpline()
    {
        if (currentSpeed <= 0.001f && isStoppedByObstacle) return; 

        float length = splineContainer[splineIndex].CalculateLength(splineContainer.transform.localToWorldMatrix);
        
        if (distanceTraveled >= length)
        {
            OnReachEnd?.Invoke();
            return;
        }

        distanceTraveled += currentSpeed * Time.deltaTime;
        float t = Mathf.Clamp01(distanceTraveled / length);

        float3 position;
        float3 tangent;
        float3 upVector;

        splineContainer.Evaluate(splineIndex, t, out position, out tangent, out upVector);

        transform.position = position;
        if (!tangent.Equals(float3.zero))
        {
            transform.rotation = Quaternion.LookRotation(tangent, upVector);
        }
    }

    public void SetTrafficLightStop(bool stop)
    {
        isStoppedByLight = stop;
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.red;
        Vector3 rayStart = transform.position + transform.TransformDirection(rayOffset);
        Gizmos.DrawRay(rayStart, transform.forward * detectionDistance);
        
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(rayStart + transform.forward * actualStopDistance, 0.5f);
    }
}
