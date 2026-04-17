using UnityEngine;
using System.Collections.Generic;

public class IntersectionZone : MonoBehaviour
{
    [SerializeField] private List<CarController> carsInIntersection = new List<CarController>();

    public bool IsClear
    {
        get
        {
            // 리스트의 모든 차량에 대해 확인
            foreach (var car in carsInIntersection)
            {
                // 유효한 차량이고, 신호에 의해 정지된 상태가 아니라면 교차로가 비어있지 않은 것으로 간주
                if (car != null && car.gameObject.activeInHierarchy && !car.isStoppedByLight)
                {
                    return false;
                }
            }
            // 신호 대기 중인 차들만 있거나 차가 한 대도 없으면 Clear로 판단
            return true;
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        CarController car = other.GetComponent<CarController>();
        if (car != null && !carsInIntersection.Contains(car))
        {
            carsInIntersection.Add(car);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        CarController car = other.GetComponent<CarController>();
        if (car != null)
        {
            carsInIntersection.Remove(car);
        }
    }

    // 성능 저하로 인해 Exit이 제대로 호출되지 않을 경우를 대비한 정기적 체크
    private void Update()
    {
        if (Time.frameCount % 30 == 0) // 매 30프레임마다 유효하지 않은(파괴된) 객체 제거
        {
            carsInIntersection.RemoveAll(car => car == null || !car.gameObject.activeInHierarchy);
        }
    }
}
