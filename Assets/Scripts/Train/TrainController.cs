using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;
using System.Collections.Generic;

/// <summary>
/// Spline을 따라 이동하며 철도 건널목 차단기와 연동되는 열차 컨트롤러.
/// </summary>
public class TrainController : MonoBehaviour
{
    [Header("Path Following")]
    public SplineContainer splineContainer;
    public int splineIndex = 0;
    public float speed = 10f;
    
    [Header("Railway Crossing Settings")]
    [Tooltip("연동할 차단기들")]
    public List<RailwayBarrier> barriers;
    
    [Tooltip("Spline 시작점으로부터 건널목(차단기)까지의 거리 (미터)")]
    public float crossingDistance = 50f;
    
    [Tooltip("열차가 건널목에 도착하기 몇 초 전에 차단기를 내릴지")]
    public float closeLeadTime = 5f;
    
    [Tooltip("열차가 건널목을 완전히 통과하는 데 걸리는 시간 (초)")]
    public float passDuration = 8f;

    [Header("Current State")]
    [SerializeField] private float distanceTraveled;
    private bool isBarrierClosed = false;

    private void Start()
    {
        // 시작 시 차단기 초기화 (열림 상태)
        SetBarriersState(PedestrianLightState.Green);
    }

    private void Update()
    {
        if (splineContainer == null) return;

        MoveAlongSpline();
        HandleBarrierLogic();
    }

    private void MoveAlongSpline()
    {
        float length = splineContainer[splineIndex].CalculateLength(splineContainer.transform.localToWorldMatrix);
        
        // 이동 거리 업데이트
        distanceTraveled += speed * Time.deltaTime;
        
        // 끝에 도달하면 처음으로 루프 (필요에 따라 수정 가능)
        if (distanceTraveled > length)
        {
            distanceTraveled = 0f;
            isBarrierClosed = false;
            SetBarriersState(PedestrianLightState.Green);
        }

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

    private void HandleBarrierLogic()
    {
        if (barriers == null || barriers.Count == 0) return;

        // 건널목 도달까지 남은 시간 계산
        float distanceToCrossing = crossingDistance - distanceTraveled;
        float timeToCrossing = distanceToCrossing / speed;

        // 차단기를 닫아야 하는 조건:
        // 1. 건널목 도달 5초(closeLeadTime) 전일 때
        // 2. 아직 건널목을 다 지나가지 않았을 때 (도달 후 passDuration 동안 유지)
        
        bool shouldClose = (timeToCrossing <= closeLeadTime && distanceToCrossing > 0) || 
                           (distanceToCrossing <= 0 && distanceTraveled < crossingDistance + (speed * passDuration));

        if (shouldClose && !isBarrierClosed)
        {
            isBarrierClosed = true;
            SetBarriersState(PedestrianLightState.Red);
            Debug.Log("[TrainController] Barrier Closing...");
        }
        else if (!shouldClose && isBarrierClosed)
        {
            isBarrierClosed = false;
            SetBarriersState(PedestrianLightState.Green);
            Debug.Log("[TrainController] Barrier Opening...");
        }
    }

    private void SetBarriersState(PedestrianLightState state)
    {
        foreach (var barrier in barriers)
        {
            if (barrier != null) barrier.SetState(state);
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (splineContainer == null) return;

        // 에디터에서 건널목 위치 시각화
        float length = splineContainer[splineIndex].CalculateLength(splineContainer.transform.localToWorldMatrix);
        float t = Mathf.Clamp01(crossingDistance / length);
        
        float3 pos;
        float3 tan;
        float3 up;
        splineContainer.Evaluate(splineIndex, t, out pos, out tan, out up);

        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(pos, 1f);
        Gizmos.DrawLine(pos, pos + up * 5f);
        
        // 닫히기 시작하는 지점 시각화
        float closeDist = crossingDistance - (speed * closeLeadTime);
        if (closeDist > 0)
        {
            float tClose = Mathf.Clamp01(closeDist / length);
            float3 posClose;
            splineContainer.Evaluate(splineIndex, tClose, out posClose, out tan, out up);
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(posClose, 0.5f);
        }
    }
}
