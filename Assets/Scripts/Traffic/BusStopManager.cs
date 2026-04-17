using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class BusStopManager : MonoBehaviour
{
    [Header("Settings")]
    public PedestrianTrafficLight stopLine;
    public float stopDuration = 5f; // 버스가 멈춰있을 시간 (n초)

    private HashSet<CarController> busesAtStop = new HashSet<CarController>();
    private bool isReleasing = false;

    private void Start()
    {
        if (stopLine == null)
        {
            stopLine = GetComponent<PedestrianTrafficLight>();
        }
        
        // 초기 상태: 항상 Green (버스 정지 대기)
        if (stopLine != null)
        {
            stopLine.SetState(PedestrianLightState.Green);
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        CarController car = other.GetComponent<CarController>();
        if (car != null && car.vehicleType == VehicleType.Bus)
        {
            car.isAtBusStop = true; // Mark as being at the stop immediately
            busesAtStop.Add(car);
            
            // 아직 출발 허용(Red) 상태가 아닐 때만 정차 핸들러 실행
            if (!isReleasing)
            {
                StartCoroutine(HandleBusStop());
            }
        }
    }

    private void OnTriggerExit(Collider other)
    {
        CarController car = other.GetComponent<CarController>();
        if (car != null && car.vehicleType == VehicleType.Bus)
        {
            car.isAtBusStop = false; // Mark as having left the stop area
            busesAtStop.Remove(car);
        }
    }

    private IEnumerator HandleBusStop()
    {
        // 1. n초간 정차 대기
        yield return new WaitForSeconds(stopDuration);

        // 2. 버스 출발 허용 (Red 상태가 자동차 통행 허가임)
        isReleasing = true;
        stopLine.SetState(PedestrianLightState.Red);
        
        // Mark buses that are about to leave as no longer being 'at the stop'.
        foreach (var bus in busesAtStop)
        {
            if (bus != null) bus.isAtBusStop = false;
        }

        // 3. 트리거 영역 안에 버스가 한 대도 없을 때까지 대기
        yield return new WaitUntil(() => busesAtStop.Count == 0);
        
        // 4. 다시 정지 상태(Green)로 복구
        stopLine.SetState(PedestrianLightState.Green);
        isReleasing = false;
    }
}
