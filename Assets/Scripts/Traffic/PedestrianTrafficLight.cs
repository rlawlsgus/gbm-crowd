using UnityEngine;
using System.Collections.Generic;

public class PedestrianTrafficLight : MonoBehaviour
{
    [Header("Settings")]
    public PedestrianLightState currentState = PedestrianLightState.Red;
    private PedestrianLightState lastVisualState = PedestrianLightState.Red;

    [Header("Visuals (Optional)")]
    public GameObject redLightObject;
    public GameObject greenLightObject;

    [Header("Filtering")]
    public bool affectAllVehicles = true;
    public List<VehicleType> affectedVehicleTypes;

    private void Start()
    {
        UpdateVisuals();
    }

    public void SetState(PedestrianLightState newState)
    {
        // WaitClear 상태가 아닐 때만 시각적 상태를 업데이트하여 이전 신호를 유지합니다.
        if (newState != PedestrianLightState.WaitClear)
        {
            lastVisualState = newState;
        }
        currentState = newState;
        UpdateVisuals();
    }

    private void UpdateVisuals()
    {
        if (redLightObject != null) redLightObject.SetActive(lastVisualState == PedestrianLightState.Red);
        if (greenLightObject != null) greenLightObject.SetActive(lastVisualState == PedestrianLightState.Green);
    }

    private void OnTriggerEnter(Collider other)
    {
        UpdateCarState(other);
    }

    private void OnTriggerStay(Collider other)
    {
        // 멈춰있는 차도 신호 변화를 감지할 수 있도록 Stay에서 지속 업데이트
        UpdateCarState(other);
    }

    private void UpdateCarState(Collider other)
    {
        CarController car = other.GetComponent<CarController>();
        if (car != null)
        {
            // 필터링: 이 신호등이 해당 차량 종류에 영향을 주는지 확인
            if (!affectAllVehicles && !affectedVehicleTypes.Contains(car.vehicleType))
            {
                return;
            }

            // 보행자 신호 Green 또는 WaitClear = 차 정지
            bool shouldCarStop = (currentState == PedestrianLightState.Green || currentState == PedestrianLightState.WaitClear);

            // [추가 로직] 이미 트리거 내에서 중앙을 지났거나 일정 속도 이상으로 통과 중이라면 멈추지 않음
            // 차량이 트리거 중심을 향해 다가오는 중일 때만 정지 명령을 적용
            Vector3 toLight = transform.position - car.transform.position;
            float approachFactor = Vector3.Dot(toLight, transform.forward);

            // approachFactor < 0 이면 이미 정지선을 지나치기 시작한 것으로 간주
            if (shouldCarStop && approachFactor < 0)
            {
                shouldCarStop = false;
            }

            // 방향 판정 완화 (0.1f 이상이면 같은 방향으로 간주)
            float dot = Vector3.Dot(car.transform.forward, transform.forward);
            if (dot > 0.1f)
            {
                car.SetTrafficLightStop(shouldCarStop);
            }
        }
    }

    private void OnTriggerExit(Collider other)
    {
        CarController car = other.GetComponent<CarController>();
        if (car != null)
        {
            // 정지선을 완전히 빠져나가면 정지 명령 해제
            car.SetTrafficLightStop(false);
        }
    }
}