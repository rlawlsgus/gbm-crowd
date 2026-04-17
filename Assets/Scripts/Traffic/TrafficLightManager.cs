using UnityEngine;
using System.Collections.Generic;

public class TrafficLightManager : MonoBehaviour
{
    [System.Serializable]
    public class LightGroup
    {
        public string groupName = "Intersection Phase";
        public List<PedestrianTrafficLight> lights;
    }

    [Header("Cycle Settings")]
    public float greenDuration = 10f;       // 특정 그룹의 자동차 통행(보행자 빨간불) 유지 시간
    public float yellowDuration = 2f;      // 신호 변경 전 미리 차량 진입을 차단하는 시간

    [Header("Safety Settings")]
    public IntersectionZone intersectionZone; // 교차로 내 차량 감지기

    [Header("Managed Groups")]
    public List<LightGroup> lightGroups;

    private int currentGroupIndex = 0;
    private float timer;
    private bool isYellowPhase;   // 노란불(선제 정지) 상태 여부

    private void Start()
    {
        if (lightGroups == null || lightGroups.Count == 0)
        {
            Debug.LogWarning("[TrafficLightManager] No light groups assigned!");
            return;
        }

        // 초기 상태: 첫 번째 그룹만 자동차 통행 허가(Red), 나머지는 보행자 통행(Green)
        InitializeSignals();
        timer = 0f;
        isYellowPhase = false;
    }

    private void InitializeSignals()
    {
        // 모든 신호를 일단 Green(보행자 통행/자동차 정지)으로 설정
        SetAllLights(PedestrianLightState.Green);
        // 첫 번째 그룹만 Red(보행자 정지/자동차 통행)로 변경
        SetOneGroupState(currentGroupIndex, PedestrianLightState.Red);
    }

    private void Update()
    {
        if (lightGroups == null || lightGroups.Count == 0) return;

        timer += Time.deltaTime;

        // 1. 노란불(선제 정지) 진입 로직
        // 설정된 yellowDuration 만큼 미리 차량 진입을 막음
        if (!isYellowPhase && timer >= (greenDuration - yellowDuration))
        {
            isYellowPhase = true;
            SetOneGroupState(currentGroupIndex, PedestrianLightState.WaitClear);
            Debug.Log($"[TrafficLightManager] Group {currentGroupIndex} entering Yellow Phase (WaitClear).");
        }

        // 2. 신호 전환 로직
        // 전체 greenDuration이 지났을 때만 교차로 상태 확인 후 전환
        if (timer >= greenDuration)
        {
            if (intersectionZone == null || intersectionZone.IsClear)
            {
                // 교차로가 비어있으면 다음 그룹으로 전환
                TransitionToNextGroup();
            }
            else
            {
                // 시간이 다 됐지만 차가 남아있으면 비워질 때까지 대기 (이미 WaitClear 상태임)
                // Debug.Log($"[TrafficLightManager] Waiting for Intersection to clear...");
            }
        }
    }

    private void TransitionToNextGroup()
    {
        // 1. 현재 그룹을 보행자 통행(Green)으로 변경 (차량 완전 정지)
        SetOneGroupState(currentGroupIndex, PedestrianLightState.Green);

        // 2. 인덱스 전환
        currentGroupIndex = (currentGroupIndex + 1) % lightGroups.Count;

        // 3. 다음 그룹의 자동차 통행 허가(Red)
        SetOneGroupState(currentGroupIndex, PedestrianLightState.Red);

        timer = 0f;
        isYellowPhase = false;
        Debug.Log($"[TrafficLightManager] Switched to Group {currentGroupIndex}.");
    }

    private void SetAllLights(PedestrianLightState state)
    {
        foreach (var group in lightGroups)
        {
            foreach (var light in group.lights)
                if (light != null) light.SetState(state);
        }
    }

    private void SetOneGroupState(int groupIndex, PedestrianLightState state)
    {
        if (groupIndex >= 0 && groupIndex < lightGroups.Count)
        {
            foreach (var light in lightGroups[groupIndex].lights)
            {
                if (light != null) light.SetState(state);
            }
        }
    }
}
