using UnityEngine;

/// <summary>
/// 철도 건널목 차단기 스크립트.
/// 신호 상태에 따라 지정된 축을 기준으로 차단기를 회전시키며, 시각적 표시등을 제어합니다.
/// </summary>
public class RailwayBarrier : MonoBehaviour
{
    [Header("State Settings")]
    public PedestrianLightState currentState = PedestrianLightState.Red;

    [Header("Visuals (Lights)")]
    public GameObject redLightObject;
    public GameObject greenLightObject;

    [Header("Movement Settings")]
    [Tooltip("차단기가 닫혔을 때(빨간불)의 로컬 회전값 (기본적으로 0,0,0)")]
    public Vector3 closedRotation = Vector3.zero;
    [Tooltip("차단기가 열렸을 때(초록불)의 로컬 회전값 (예: Z축으로 90도)")]
    public Vector3 openRotation = new Vector3(0, 0, 90f);
    [Tooltip("회전 속도")]
    public float rotationSpeed = 2f;

    private void Start()
    {
        UpdateVisuals();
        // 초기 각도 즉시 설정
        transform.localRotation = Quaternion.Euler(currentState == PedestrianLightState.Green ? openRotation : closedRotation);
    }

    private void Update()
    {
        // 타겟 회전값 계산
        Vector3 targetEuler = (currentState == PedestrianLightState.Green) ? openRotation : closedRotation;
        Quaternion targetRotation = Quaternion.Euler(targetEuler);

        // 부드럽게 회전 적용
        transform.localRotation = Quaternion.Slerp(transform.localRotation, targetRotation, Time.deltaTime * rotationSpeed);
    }

    /// <summary>
    /// 외부(예: TrafficLightManager나 열차 감지 센서)에서 상태를 바꿀 때 호출
    /// </summary>
    public void SetState(PedestrianLightState newState)
    {
        currentState = newState;
        UpdateVisuals();
    }

    private void UpdateVisuals()
    {
        if (redLightObject != null) redLightObject.SetActive(currentState == PedestrianLightState.Red);
        if (greenLightObject != null) greenLightObject.SetActive(currentState == PedestrianLightState.Green);
    }
}
