using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.IO;
using System.Reflection;

public class SocialTaskCompletionRate : MonoBehaviour
{
    [Header("Agent Settings")]
    [Tooltip("측정할 agent들이 들어있는 parent transform")]
    public Transform agentsRoot;
    [Tooltip("새 agent 탐색 주기 (초)")]
    public float searchInterval = 0.5f;
    [Tooltip("agentsRoot가 비어 있으면 이름으로 자동 탐색")]
    public bool autoFindAgentsRoot = true;
    public string agentsRootObjectName = "Agents";

    [Header("Success Settings")]
    [Tooltip("Agents(root) 아래에서 inactive 상태인 agent를 success로 간주")]
    public bool countInactiveAgentsAsSuccess = true;
    [Tooltip("child까지 내려가며 success flag를 찾음")]
    public bool searchSuccessFlagsInChildren = true;
    [Tooltip("success 감지 로그 출력")]
    public bool logSuccessDetection = true;

    [Tooltip("success로 간주할 bool field/property 이름들")]
    public List<string> successBoolNames = new List<string>
    {
        "GoalReached",
        "HasReachedGoal",
        "goalReached",
        "hasReachedGoal",
        "ReachedGoal",
        "reachedGoal",
        "Success",
        "success",
        "IsSuccess",
        "isSuccess",
        "Arrived",
        "arrived"
    };

    [Tooltip("success로 간주할 bool method 이름들")]
    public List<string> successMethodNames = new List<string>
    {
        "IsGoalReached",
        "HasReachedGoal",
        "ReachedGoal",
        "IsSuccess",
        "HasSucceeded"
    };

    [Header("Environment Settings")]
    [Tooltip("Danger Zone들이 들어있는 parent. 비어 있으면 자동 탐색")]
    public Transform dangerZonesRoot;
    [Tooltip("Obstacle들이 들어있는 parent. 비어 있으면 자동 탐색")]
    public Transform obstaclesRoot;
    public string dangerZonesRootObjectName = "Danger Zones";
    public string obstaclesRootObjectName = "Obstacles";

    [Header("Collision Settings")]
    [Tooltip("ComputePenetration이 false여도 bounds가 겹치면 충돌로 볼지")]
    public bool useBoundsIntersectionAsFallback = true;
    [Tooltip("agent-agent collision에서 trigger collider도 포함할지")]
    public bool includeTriggerCollidersForAgentCollision = false;

    [Header("Social Context Metrics")]
    [Tooltip("Nearest-agent distance와 TTC(Time-To-Collision)를 측정할지")]
    public bool enableSocialContextMetrics = true;
    [Tooltip("거리/TTC 계산 시 Y축을 무시하고 XZ 평면에서 계산")]
    public bool useXZPlaneForSocialMetrics = true;
    [Tooltip("Nearest-agent distance에서 collider 반경을 빼서 surface clearance로 기록. false면 transform 중심 간 거리")]
    public bool subtractApproxAgentRadiusFromNearestDistance = false;
    [Tooltip("Social metric 계산에 trigger collider를 포함할지")]
    public bool includeTriggerCollidersForSocialMetrics = false;
    [Tooltip("TTC에서 충돌/near-miss로 볼 반경. 0 이하면 두 agent의 collider 반경 합을 사용")]
    public float ttcCollisionRadius = 0f;
    [Tooltip("Low-TTC로 집계할 임계값(초)")]
    public float lowTTCThreshold = 2.0f;
    [Tooltip("TTC 계산에 사용할 최소 상대 속도. 너무 작으면 TTC 없음으로 처리")]
    public float minRelativeSpeedForTTC = 0.05f;
    [Tooltip("이 값보다 큰 TTC는 평균/최소 TTC 집계에서 제외. 0 이하면 제한 없음")]
    public float maxTTCToRecord = 10.0f;

    [Header("Navigation Performance Metrics")]
    [Tooltip("Time-to-goal과 path length를 측정할지")]
    public bool enableNavigationPerformanceMetrics = true;
    [Tooltip("Path length 계산 시 Y축을 무시하고 XZ 평면에서 계산")]
    public bool useXZPlaneForPathLength = true;
    [Tooltip("false면 success 감지 시점까지만 path length를 누적. true면 success 이후 움직임도 observed path length에 계속 누적")]
    public bool continuePathLengthAfterGoal = false;

    public enum STENormalizationMode
    {
        SceneAverage,
        RawInverseCost,
        AutoSceneAverageOrRawInverseCost
    }

    [Header("STE Settings")]
    [Tooltip("STCR Summary에 Spatio-Temporal Efficiency(STE)를 추가합니다.")]
    public bool enableSTEMetric = true;
    [Tooltip("Auto는 scene 평균값이 있으면 논문식 STE를 쓰고, 없으면 raw inverse-cost STE를 사용해서 NA를 피합니다.")]
    public STENormalizationMode steNormalizationMode = STENormalizationMode.AutoSceneAverageOrRawInverseCost;
    [Tooltip("논문식 STE에 사용할 scene-level 평균 Time To Goal. 모든 method 평균값을 넣으세요. 0이면 fallback 사용.")]
    public float sceneAverageTimeToGoalForSTE = 0f;
    [Tooltip("논문식 STE에 사용할 scene-level 평균 Path Length. 모든 method 평균값을 넣으세요. 0이면 fallback 사용.")]
    public float sceneAveragePathLengthForSTE = 0f;
    [Tooltip("Raw fallback STE = rawSTEScale / sqrt(Time To Goal * Path Length).")]
    public float rawSTEScale = 100f;
    [Tooltip("STE 출력 소수점 자리수")]
    public int steDecimalPlaces = 3;

    [Header("Debug Settings")]
    public bool enableDebugLog = true;
    [Tooltip("몇 초마다 agent 상태를 콘솔에 찍을지")]
    public float debugLogInterval = 1.0f;
    [Tooltip("비워두면 전체 agent 출력, 값이 있으면 이름에 해당 문자열이 포함된 agent만 출력")]
    public string debugAgentNameFilter = "";
    [Tooltip("register 시 collider 목록도 출력")]
    public bool logRegisteredColliders = true;

    [Header("Trajectory Map Settings")]
    public bool enableTrajectoryMap = true;
    public int mapResolution = 2048;
    public float mapWidth = 100f;
    public float mapHeight = 100f;
    public Color normalPathColor = Color.red;
    public Color dangerPathColor = Color.yellow;
    public Color dangerZoneOutlineColor = Color.blue;
    public Color obstacleOutlineColor = new Color(1f, 0.5f, 0f);
    public string mapFileName = "SocialTrajectoryMap.png";
    public float minRecordDistance = 0.1f;
    public int endPointRadius = 10;

    private List<Collider> dangerZoneColliders = new List<Collider>();
    private List<Collider> obstacleColliders = new List<Collider>();

    private float searchTimer = 0f;
    private float debugTimer = 0f;
    private bool warnedAgentsRootMissing = false;

    public struct TrajectoryPoint
    {
        public Vector3 position;
        public bool isDanger;

        public TrajectoryPoint(Vector3 pos, bool danger)
        {
            position = pos;
            isDanger = danger;
        }
    }

    public class AgentTrackingData
    {
        public float totalTime;
        public float dangerZoneTime;
        public float obstacleTime;
        public float agentCollisionTime;
        public float unsafeAnyTime;

        public Vector3 previousSocialMetricPosition;
        public Vector3 currentSocialMetricVelocity;
        public bool hasPreviousSocialMetricPosition;
        public float approxSocialRadius;

        public float nearestAgentDistanceSum;
        public float nearestAgentDistanceSampleTime;
        public float minNearestAgentDistance = Mathf.Infinity;
        public float lastNearestAgentDistance = Mathf.Infinity;

        public float ttcSum;
        public float ttcSampleTime;
        public float minTTC = Mathf.Infinity;
        public float lastTTC = Mathf.Infinity;
        public float lowTTCTime;

        public Vector3 previousPathMetricPosition;
        public bool hasPreviousPathMetricPosition;
        public float pathLength;
        public float pathLengthToGoal;
        public bool hasPathLengthToGoal;
        public float timeToGoal;
        public bool hasTimeToGoal;

        public bool hasReachedGoal;
        public bool wasEverActive;
        public bool previousActiveState;
        public string successDetectedBy = "";

        public List<Collider> agentColliders = new List<Collider>();
        public List<TrajectoryPoint> trajectory = new List<TrajectoryPoint>();

        public bool lastDanger;
        public bool lastObstacle;
        public bool lastAgentCollision;
    }

    private Dictionary<Transform, AgentTrackingData> trackingData = new Dictionary<Transform, AgentTrackingData>();

    void Start()
    {
        ResolveSceneReferences();
        SearchForAgents();
        RefreshInactiveSuccessFromRoot();
    }

    void Update()
    {
        searchTimer += Time.deltaTime;
        if (searchTimer >= searchInterval)
        {
            SearchForAgents();
            searchTimer = 0f;
        }

        UpdateAgentStats();
        RefreshInactiveSuccessFromRoot();

        if (enableDebugLog)
        {
            debugTimer += Time.deltaTime;
            if (debugTimer >= debugLogInterval)
            {
                PrintDebugStatus();
                debugTimer = 0f;
            }
        }
    }

    private void ResolveSceneReferences()
    {
        if (agentsRoot == null && autoFindAgentsRoot)
        {
            GameObject foundAgents = GameObject.Find(agentsRootObjectName);
            if (foundAgents != null)
            {
                agentsRoot = foundAgents.transform;
                Debug.Log($"[SocialTaskCompletionRate] Found agents root: '{agentsRoot.name}'");
            }
        }

        if (dangerZonesRoot == null)
        {
            GameObject foundObj = GameObject.Find(dangerZonesRootObjectName);
            if (foundObj != null)
            {
                dangerZonesRoot = foundObj.transform;
            }
        }

        if (dangerZonesRoot != null)
        {
            dangerZoneColliders = dangerZonesRoot.GetComponentsInChildren<Collider>(true).ToList();
            Debug.Log($"[SocialTaskCompletionRate] Found {dangerZoneColliders.Count} danger zone colliders under '{dangerZonesRoot.name}'.");
        }
        else
        {
            Debug.LogWarning($"[SocialTaskCompletionRate] '{dangerZonesRootObjectName}' object not found in scene and not assigned.");
        }

        if (obstaclesRoot == null)
        {
            GameObject foundObj = GameObject.Find(obstaclesRootObjectName);
            if (foundObj != null)
            {
                obstaclesRoot = foundObj.transform;
            }
        }

        if (obstaclesRoot != null)
        {
            obstacleColliders = obstaclesRoot.GetComponentsInChildren<Collider>(true).ToList();
            Debug.Log($"[SocialTaskCompletionRate] Found {obstacleColliders.Count} obstacle colliders under '{obstaclesRoot.name}'.");
        }
        else
        {
            Debug.LogWarning($"[SocialTaskCompletionRate] '{obstaclesRootObjectName}' object not found in scene and not assigned.");
        }
    }

    private void SearchForAgents()
    {
        if (agentsRoot == null)
        {
            if (autoFindAgentsRoot)
            {
                GameObject foundAgents = GameObject.Find(agentsRootObjectName);
                if (foundAgents != null)
                {
                    agentsRoot = foundAgents.transform;
                    Debug.Log($"[SocialTaskCompletionRate] Found agents root during runtime: '{agentsRoot.name}'");
                }
            }

            if (agentsRoot == null)
            {
                if (!warnedAgentsRootMissing)
                {
                    Debug.LogWarning("[SocialTaskCompletionRate] agentsRoot is not assigned.");
                    warnedAgentsRootMissing = true;
                }
                return;
            }
        }

        warnedAgentsRootMissing = false;

        AgentBase[] agents = agentsRoot.GetComponentsInChildren<AgentBase>(true);
        foreach (AgentBase agent in agents)
        {
            if (agent == null) continue;
            RegisterAgent(agent.transform);
        }
    }

    private void RegisterAgent(Transform agentTransform)
    {
        if (agentTransform == null) return;
        if (trackingData.ContainsKey(agentTransform)) return;

        AgentBase agentBase = agentTransform.GetComponent<AgentBase>();
        if (agentBase == null)
        {
            agentBase = agentTransform.GetComponentInChildren<AgentBase>(true);
            if (agentBase != null)
            {
                agentTransform = agentBase.transform;
            }
        }

        if (agentBase == null) return;

        List<Collider> cols = agentTransform
            .GetComponentsInChildren<Collider>(true)
            .Where(c => c != null)
            .ToList();

        if (cols.Count == 0)
        {
            Debug.LogWarning($"[SocialTaskCompletionRate] Collider not found for agent: {agentTransform.name}");
            return;
        }

        AgentTrackingData newData = new AgentTrackingData();
        newData.agentColliders = cols;
        newData.previousActiveState = agentTransform.gameObject.activeInHierarchy;
        trackingData.Add(agentTransform, newData);

        Debug.Log($"[SocialTaskCompletionRate] Registered agent: {agentTransform.name} | colliders={cols.Count}");

        if (logRegisteredColliders)
        {
            for (int i = 0; i < cols.Count; i++)
            {
                Collider c = cols[i];
                Debug.Log($"   └ Collider[{i}] {c.name} ({c.GetType().Name}) trigger={c.isTrigger}");
            }
        }
    }

    private void UpdateAgentStats()
    {
        float dt = Time.deltaTime;
        if (trackingData.Count == 0) return;

        if (enableSocialContextMetrics)
        {
            RefreshSocialKinematicState(dt);
        }

        foreach (var kvp in trackingData)
        {
            Transform agent = kvp.Key;
            AgentTrackingData data = kvp.Value;

            if (agent == null) continue;

            bool currentActive = agent.gameObject.activeInHierarchy;

            if (currentActive)
            {
                data.wasEverActive = true;
            }

            if (!currentActive)
            {
                UpdateSuccessStateFromFlags(agent, data);

                data.lastDanger = false;
                data.lastObstacle = false;
                data.lastAgentCollision = false;
                data.lastNearestAgentDistance = Mathf.Infinity;
                data.lastTTC = Mathf.Infinity;
                data.hasPreviousSocialMetricPosition = false;
                data.currentSocialMetricVelocity = Vector3.zero;
                data.hasPreviousPathMetricPosition = false;
                data.previousActiveState = currentActive;
                continue;
            }

            data.totalTime += dt;

            if (enableNavigationPerformanceMetrics)
            {
                UpdateNavigationPerformanceMetrics(agent, data);
            }

            UpdateSuccessStateFromFlags(agent, data);

            bool inDangerZone = IsInDangerZone(data.agentColliders);
            bool inObstacle = IsCollidingWithObstacle(data.agentColliders);
            bool inAgentCollision = IsCollidingWithAgent(data.agentColliders, agent);

            data.lastDanger = inDangerZone;
            data.lastObstacle = inObstacle;
            data.lastAgentCollision = inAgentCollision;

            if (inDangerZone) data.dangerZoneTime += dt;
            if (inObstacle) data.obstacleTime += dt;
            if (inAgentCollision) data.agentCollisionTime += dt;

            bool inAnyUnsafe = inDangerZone || inObstacle || inAgentCollision;
            if (inAnyUnsafe) data.unsafeAnyTime += dt;

            if (enableSocialContextMetrics)
            {
                UpdateSocialContextMetrics(agent, data, dt);
            }

            if (enableTrajectoryMap)
            {
                Vector3 currentPos = agent.position;
                bool isDanger = inDangerZone;

                if (data.trajectory.Count == 0 ||
                    Vector3.Distance(data.trajectory[data.trajectory.Count - 1].position, currentPos) >= minRecordDistance)
                {
                    data.trajectory.Add(new TrajectoryPoint(currentPos, isDanger));
                }
            }

            data.previousActiveState = currentActive;
        }
    }

    private void RefreshInactiveSuccessFromRoot()
    {
        if (!countInactiveAgentsAsSuccess) return;
        if (agentsRoot == null) return;

        AgentBase[] allAgentsUnderRoot = agentsRoot.GetComponentsInChildren<AgentBase>(true);
        foreach (AgentBase agentBase in allAgentsUnderRoot)
        {
            if (agentBase == null) continue;

            Transform agent = agentBase.transform;

            if (!trackingData.ContainsKey(agent))
            {
                RegisterAgent(agent);
            }

            if (!trackingData.TryGetValue(agent, out AgentTrackingData data))
                continue;

            if (!agent.gameObject.activeInHierarchy)
            {
                if (!data.hasReachedGoal)
                {
                    MarkSuccess(agent, data, "Inactive under Agents(root)");
                }
            }
        }
    }

    private void UpdateSuccessStateFromFlags(Transform agent, AgentTrackingData data)
    {
        if (data.hasReachedGoal) return;

        MonoBehaviour[] behaviours = searchSuccessFlagsInChildren
            ? agent.GetComponentsInChildren<MonoBehaviour>(true)
            : agent.GetComponents<MonoBehaviour>();

        foreach (MonoBehaviour mb in behaviours)
        {
            if (mb == null) continue;

            foreach (string boolName in successBoolNames)
            {
                if (TryReadBoolMember(mb, boolName, out bool boolValue) && boolValue)
                {
                    MarkSuccess(agent, data, $"{mb.GetType().Name}.{boolName}");
                    return;
                }
            }

            foreach (string methodName in successMethodNames)
            {
                if (TryInvokeBoolMethod(mb, methodName, out bool methodValue) && methodValue)
                {
                    MarkSuccess(agent, data, $"{mb.GetType().Name}.{methodName}()");
                    return;
                }
            }
        }
    }

    private void MarkSuccess(Transform agent, AgentTrackingData data, string source)
    {
        data.hasReachedGoal = true;

        if (enableNavigationPerformanceMetrics)
        {
            if (!data.hasTimeToGoal)
            {
                data.timeToGoal = data.totalTime;
                data.hasTimeToGoal = true;
            }

            if (!data.hasPathLengthToGoal)
            {
                data.pathLengthToGoal = data.pathLength;
                data.hasPathLengthToGoal = true;
            }
        }

        if (string.IsNullOrEmpty(data.successDetectedBy))
        {
            data.successDetectedBy = source;
        }

        if (logSuccessDetection)
        {
            Debug.Log($"[STCR Success] {agent.name} success detected by: {source}");
        }
    }

    private bool TryReadBoolMember(object obj, string memberName, out bool value)
    {
        value = false;
        if (obj == null) return false;

        System.Type type = obj.GetType();

        FieldInfo field = type.GetField(memberName, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
        if (field != null && field.FieldType == typeof(bool))
        {
            value = (bool)field.GetValue(obj);
            return true;
        }

        PropertyInfo prop = type.GetProperty(memberName, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
        if (prop != null && prop.PropertyType == typeof(bool) && prop.CanRead)
        {
            value = (bool)prop.GetValue(obj, null);
            return true;
        }

        return false;
    }

    private bool TryInvokeBoolMethod(object obj, string methodName, out bool value)
    {
        value = false;
        if (obj == null) return false;

        System.Type type = obj.GetType();
        MethodInfo method = type.GetMethod(methodName, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);

        if (method == null) return false;
        if (method.ReturnType != typeof(bool)) return false;
        if (method.GetParameters().Length != 0) return false;

        value = (bool)method.Invoke(obj, null);
        return true;
    }

    private bool IsInDangerZone(List<Collider> agentColliders)
    {
        if (agentColliders == null || agentColliders.Count == 0) return false;

        foreach (Collider agentCol in agentColliders)
        {
            if (!IsUsableCollider(agentCol)) continue;

            foreach (Collider zoneCol in dangerZoneColliders)
            {
                if (!IsUsableCollider(zoneCol)) continue;
                if (CheckCollision(agentCol, zoneCol)) return true;
            }
        }

        return false;
    }

    private bool IsCollidingWithObstacle(List<Collider> agentColliders)
    {
        if (agentColliders == null || agentColliders.Count == 0) return false;

        foreach (Collider agentCol in agentColliders)
        {
            if (!IsUsableCollider(agentCol)) continue;

            foreach (Collider obstacleCol in obstacleColliders)
            {
                if (!IsUsableCollider(obstacleCol)) continue;
                if (CheckCollision(agentCol, obstacleCol)) return true;
            }
        }

        return false;
    }

    private bool IsCollidingWithAgent(List<Collider> currentAgentColliders, Transform currentAgentTransform)
    {
        if (currentAgentColliders == null || currentAgentColliders.Count == 0) return false;

        foreach (var kvp in trackingData)
        {
            Transform otherAgent = kvp.Key;
            AgentTrackingData otherData = kvp.Value;

            if (otherAgent == null) continue;
            if (otherAgent == currentAgentTransform) continue;
            if (!otherAgent.gameObject.activeInHierarchy) continue;
            if (otherData.agentColliders == null || otherData.agentColliders.Count == 0) continue;

            foreach (Collider myCol in currentAgentColliders)
            {
                if (!IsUsableCollider(myCol)) continue;
                if (!includeTriggerCollidersForAgentCollision && myCol.isTrigger) continue;

                foreach (Collider otherCol in otherData.agentColliders)
                {
                    if (!IsUsableCollider(otherCol)) continue;
                    if (!includeTriggerCollidersForAgentCollision && otherCol.isTrigger) continue;

                    if (CheckCollision(myCol, otherCol)) return true;
                }
            }
        }

        return false;
    }

    private void UpdateNavigationPerformanceMetrics(Transform agent, AgentTrackingData data)
    {
        if (agent == null || data == null) return;

        Vector3 currentPosition = GetPathMetricPosition(agent.position);

        if (!data.hasPreviousPathMetricPosition)
        {
            data.previousPathMetricPosition = currentPosition;
            data.hasPreviousPathMetricPosition = true;
            return;
        }

        if (!data.hasReachedGoal || continuePathLengthAfterGoal)
        {
            float stepDistance = Vector3.Distance(data.previousPathMetricPosition, currentPosition);
            if (stepDistance > 0f)
            {
                data.pathLength += stepDistance;
            }
        }

        data.previousPathMetricPosition = currentPosition;
    }

    private Vector3 GetPathMetricPosition(Vector3 worldPosition)
    {
        if (!useXZPlaneForPathLength)
        {
            return worldPosition;
        }

        return new Vector3(worldPosition.x, 0f, worldPosition.z);
    }

    private void RefreshSocialKinematicState(float dt)
    {
        if (dt <= 0.000001f) return;

        foreach (var kvp in trackingData)
        {
            Transform agent = kvp.Key;
            AgentTrackingData data = kvp.Value;

            if (agent == null)
            {
                continue;
            }

            if (!agent.gameObject.activeInHierarchy)
            {
                data.currentSocialMetricVelocity = Vector3.zero;
                data.hasPreviousSocialMetricPosition = false;
                continue;
            }

            Vector3 currentPosition = GetSocialMetricPosition(agent.position);

            if (data.hasPreviousSocialMetricPosition)
            {
                data.currentSocialMetricVelocity = (currentPosition - data.previousSocialMetricPosition) / dt;
            }
            else
            {
                data.currentSocialMetricVelocity = Vector3.zero;
                data.hasPreviousSocialMetricPosition = true;
            }

            data.previousSocialMetricPosition = currentPosition;
            data.approxSocialRadius = EstimateAgentSocialRadius(data);
        }
    }

    private void UpdateSocialContextMetrics(Transform currentAgent, AgentTrackingData data, float dt)
    {
        if (currentAgent == null || data == null || dt <= 0.000001f) return;

        float nearestDistance = ComputeNearestAgentDistance(currentAgent, data);
        data.lastNearestAgentDistance = nearestDistance;

        if (IsFiniteMetric(nearestDistance))
        {
            data.nearestAgentDistanceSum += nearestDistance * dt;
            data.nearestAgentDistanceSampleTime += dt;

            if (nearestDistance < data.minNearestAgentDistance)
            {
                data.minNearestAgentDistance = nearestDistance;
            }
        }

        float minTTC = ComputeMinimumTTC(currentAgent, data);
        data.lastTTC = minTTC;

        if (IsFiniteMetric(minTTC) && ShouldRecordTTC(minTTC))
        {
            data.ttcSum += minTTC * dt;
            data.ttcSampleTime += dt;

            if (minTTC < data.minTTC)
            {
                data.minTTC = minTTC;
            }
        }

        if (IsFiniteMetric(minTTC) && minTTC <= lowTTCThreshold)
        {
            data.lowTTCTime += dt;
        }
    }

    private Vector3 GetSocialMetricPosition(Vector3 worldPosition)
    {
        if (!useXZPlaneForSocialMetrics)
        {
            return worldPosition;
        }

        return new Vector3(worldPosition.x, 0f, worldPosition.z);
    }

    private float ComputeNearestAgentDistance(Transform currentAgent, AgentTrackingData currentData)
    {
        float nearestDistance = Mathf.Infinity;
        Vector3 currentPosition = GetSocialMetricPosition(currentAgent.position);

        foreach (var kvp in trackingData)
        {
            Transform otherAgent = kvp.Key;
            AgentTrackingData otherData = kvp.Value;

            if (otherAgent == null) continue;
            if (otherAgent == currentAgent) continue;
            if (!otherAgent.gameObject.activeInHierarchy) continue;

            Vector3 otherPosition = GetSocialMetricPosition(otherAgent.position);
            float distance = Vector3.Distance(currentPosition, otherPosition);

            if (subtractApproxAgentRadiusFromNearestDistance)
            {
                distance -= currentData.approxSocialRadius + otherData.approxSocialRadius;
            }

            if (distance < nearestDistance)
            {
                nearestDistance = distance;
            }
        }

        return nearestDistance;
    }

    private float ComputeMinimumTTC(Transform currentAgent, AgentTrackingData currentData)
    {
        float minTTC = Mathf.Infinity;
        Vector3 currentPosition = GetSocialMetricPosition(currentAgent.position);

        foreach (var kvp in trackingData)
        {
            Transform otherAgent = kvp.Key;
            AgentTrackingData otherData = kvp.Value;

            if (otherAgent == null) continue;
            if (otherAgent == currentAgent) continue;
            if (!otherAgent.gameObject.activeInHierarchy) continue;

            Vector3 otherPosition = GetSocialMetricPosition(otherAgent.position);

            if (TryComputePairTTC(currentPosition, currentData, otherPosition, otherData, out float pairTTC))
            {
                if (pairTTC < minTTC)
                {
                    minTTC = pairTTC;
                }
            }
        }

        return minTTC;
    }

    private bool TryComputePairTTC(
        Vector3 currentPosition,
        AgentTrackingData currentData,
        Vector3 otherPosition,
        AgentTrackingData otherData,
        out float ttc)
    {
        ttc = Mathf.Infinity;

        Vector3 relativePosition = otherPosition - currentPosition;
        Vector3 relativeVelocity = otherData.currentSocialMetricVelocity - currentData.currentSocialMetricVelocity;

        float relativeSpeedSq = relativeVelocity.sqrMagnitude;
        float minRelativeSpeedSq = minRelativeSpeedForTTC * minRelativeSpeedForTTC;

        if (relativeSpeedSq < minRelativeSpeedSq)
        {
            return false;
        }

        float radius = ttcCollisionRadius > 0f
            ? ttcCollisionRadius
            : Mathf.Max(0.01f, currentData.approxSocialRadius + otherData.approxSocialRadius);

        float a = relativeSpeedSq;
        float b = 2f * Vector3.Dot(relativePosition, relativeVelocity);
        float c = Vector3.Dot(relativePosition, relativePosition) - radius * radius;

        if (c <= 0f)
        {
            ttc = 0f;
            return true;
        }

        // 두 agent가 서로 가까워지고 있지 않으면 TTC 없음으로 처리한다.
        if (b >= 0f)
        {
            return false;
        }

        float discriminant = b * b - 4f * a * c;
        if (discriminant < 0f)
        {
            return false;
        }

        float sqrtDiscriminant = Mathf.Sqrt(discriminant);
        float firstHitTime = (-b - sqrtDiscriminant) / (2f * a);

        if (firstHitTime < 0f)
        {
            return false;
        }

        ttc = firstHitTime;
        return true;
    }

    private bool ShouldRecordTTC(float ttc)
    {
        if (!IsFiniteMetric(ttc)) return false;
        if (maxTTCToRecord <= 0f) return true;
        return ttc <= maxTTCToRecord;
    }

    private float EstimateAgentSocialRadius(AgentTrackingData data)
    {
        if (data == null || data.agentColliders == null || data.agentColliders.Count == 0)
        {
            return 0.01f;
        }

        bool hasBounds = false;
        Bounds combinedBounds = new Bounds();

        foreach (Collider col in data.agentColliders)
        {
            if (!IsUsableCollider(col)) continue;
            if (!includeTriggerCollidersForSocialMetrics && col.isTrigger) continue;

            if (!hasBounds)
            {
                combinedBounds = col.bounds;
                hasBounds = true;
            }
            else
            {
                combinedBounds.Encapsulate(col.bounds);
            }
        }

        // non-trigger collider가 없으면 fallback으로 usable collider 전체를 사용한다.
        if (!hasBounds)
        {
            foreach (Collider col in data.agentColliders)
            {
                if (!IsUsableCollider(col)) continue;

                if (!hasBounds)
                {
                    combinedBounds = col.bounds;
                    hasBounds = true;
                }
                else
                {
                    combinedBounds.Encapsulate(col.bounds);
                }
            }
        }

        if (!hasBounds)
        {
            return 0.01f;
        }

        if (useXZPlaneForSocialMetrics)
        {
            return Mathf.Max(0.01f, Mathf.Max(combinedBounds.extents.x, combinedBounds.extents.z));
        }

        return Mathf.Max(0.01f, Mathf.Max(combinedBounds.extents.x, Mathf.Max(combinedBounds.extents.y, combinedBounds.extents.z)));
    }

    private bool IsUsableCollider(Collider col)
    {
        if (col == null) return false;
        if (!col.enabled) return false;
        if (!col.gameObject.activeInHierarchy) return false;
        return true;
    }

    private bool CheckCollision(Collider c1, Collider c2)
    {
        if (!IsUsableCollider(c1) || !IsUsableCollider(c2)) return false;

        if (!c1.bounds.Intersects(c2.bounds))
            return false;

        Vector3 direction;
        float distance;
        bool penetrated = Physics.ComputePenetration(
            c1, c1.transform.position, c1.transform.rotation,
            c2, c2.transform.position, c2.transform.rotation,
            out direction, out distance
        );

        if (penetrated) return true;
        if (useBoundsIntersectionAsFallback) return true;

        return false;
    }

    public float CalculateSafeRate(AgentTrackingData data)
    {
        if (data.totalTime <= 0.0001f) return 1f;

        float rate = 1.0f - (data.unsafeAnyTime / data.totalTime);
        return Mathf.Clamp01(rate);
    }

    public float CalculateAverageNearestAgentDistance(AgentTrackingData data)
    {
        if (data == null || data.nearestAgentDistanceSampleTime <= 0.0001f) return float.NaN;
        return data.nearestAgentDistanceSum / data.nearestAgentDistanceSampleTime;
    }

    public float CalculateAverageTTC(AgentTrackingData data)
    {
        if (data == null || data.ttcSampleTime <= 0.0001f) return float.NaN;
        return data.ttcSum / data.ttcSampleTime;
    }

    public float CalculateLowTTCRate(AgentTrackingData data)
    {
        if (data == null || data.totalTime <= 0.0001f) return 0f;
        return Mathf.Clamp01(data.lowTTCTime / data.totalTime);
    }

    private bool IsFiniteMetric(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private float CalculateMeanFiniteMetric(List<float> values)
    {
        if (values == null || values.Count == 0) return float.NaN;

        List<float> finiteValues = values
            .Where(v => IsFiniteMetric(v))
            .ToList();

        if (finiteValues.Count == 0) return float.NaN;
        return finiteValues.Average();
    }

    private float CalculateSTE(float averageTimeToGoal, float averagePathLength)
    {
        if (!enableSTEMetric) return float.NaN;
        if (!IsFiniteMetric(averageTimeToGoal) || !IsFiniteMetric(averagePathLength)) return float.NaN;
        if (averageTimeToGoal <= 0.0001f || averagePathLength <= 0.0001f) return float.NaN;

        bool hasSceneAverages =
            IsFiniteMetric(sceneAverageTimeToGoalForSTE) &&
            IsFiniteMetric(sceneAveragePathLengthForSTE) &&
            sceneAverageTimeToGoalForSTE > 0.0001f &&
            sceneAveragePathLengthForSTE > 0.0001f;

        if (steNormalizationMode == STENormalizationMode.SceneAverage)
        {
            if (!hasSceneAverages) return float.NaN;
            return Mathf.Sqrt(
                (sceneAverageTimeToGoalForSTE / averageTimeToGoal) *
                (sceneAveragePathLengthForSTE / averagePathLength));
        }

        if (steNormalizationMode == STENormalizationMode.AutoSceneAverageOrRawInverseCost && hasSceneAverages)
        {
            return Mathf.Sqrt(
                (sceneAverageTimeToGoalForSTE / averageTimeToGoal) *
                (sceneAveragePathLengthForSTE / averagePathLength));
        }

        float safeScale = rawSTEScale > 0.0001f ? rawSTEScale : 1f;
        return safeScale / Mathf.Sqrt(averageTimeToGoal * averagePathLength);
    }

    private string FormatSTE(float value)
    {
        if (!IsFiniteMetric(value)) return "NA";
        int decimals = Mathf.Clamp(steDecimalPlaces, 0, 6);
        return value.ToString("F" + decimals);
    }

    private string FormatMetricValue(float value, string suffix = "")
    {
        if (!IsFiniteMetric(value)) return "N/A";
        return $"{value:F2}{suffix}";
    }

    private string FormatAverageMetric(List<float> values, string suffix = "")
    {
        if (values == null || values.Count == 0) return "N/A";
        return $"{values.Average():F2}{suffix}";
    }

    private string FormatMinimumMetric(List<float> values, string suffix = "")
    {
        if (values == null || values.Count == 0) return "N/A";
        return $"{values.Min():F2}{suffix}";
    }

    private string FormatMaximumMetric(List<float> values, string suffix = "")
    {
        if (values == null || values.Count == 0) return "N/A";
        return $"{values.Max():F2}{suffix}";
    }

    private string FormatAveragePercent(List<float> values)
    {
        if (values == null || values.Count == 0) return "N/A";
        return $"{values.Average() * 100f:F2}%";
    }

    private void PrintDebugStatus()
    {
        foreach (var kvp in trackingData)
        {
            Transform agent = kvp.Key;
            AgentTrackingData data = kvp.Value;

            if (agent == null) continue;

            if (!string.IsNullOrEmpty(debugAgentNameFilter) &&
                !agent.name.Contains(debugAgentNameFilter))
            {
                continue;
            }

            Debug.Log(
                $"[STCR Debug] {agent.name} | " +
                $"active={agent.gameObject.activeInHierarchy} | " +
                $"success={data.hasReachedGoal} | " +
                $"successBy={data.successDetectedBy} | " +
                $"danger={data.lastDanger} | " +
                $"obstacle={data.lastObstacle} | " +
                $"collision={data.lastAgentCollision} | " +
                $"total={data.totalTime:F2} | " +
                $"dangerTime={data.dangerZoneTime:F2} | " +
                $"obstacleTime={data.obstacleTime:F2} | " +
                $"collisionTime={data.agentCollisionTime:F2} | " +
                $"unsafeAny={data.unsafeAnyTime:F2} | " +
                $"timeToGoal={FormatMetricValue(data.hasTimeToGoal ? data.timeToGoal : Mathf.Infinity, "s")} | " +
                $"pathLength={data.pathLength:F2}m | " +
                $"pathLengthToGoal={FormatMetricValue(data.hasPathLengthToGoal ? data.pathLengthToGoal : Mathf.Infinity, "m")} | " +
                $"nearest={FormatMetricValue(data.lastNearestAgentDistance, "m")} | " +
                $"ttc={FormatMetricValue(data.lastTTC, "s")}"
            );
        }
    }

    private void OnApplicationQuit()
    {
        RefreshInactiveSuccessFromRoot();

        int totalAgents = 0;
        int successCount = 0;

        List<float> allAgentCollisionRates = new List<float>();
        List<float> allObstacleRates = new List<float>();
        List<float> allDangerZoneViolationRates = new List<float>(); // Danger Zone Violation = dangerZoneTime / totalTime

        List<float> allTimeToGoalValues = new List<float>();
        List<float> allPathLengthValues = new List<float>();

        List<float> allAvgNearestAgentDistances = new List<float>();
        List<float> allAvgTTCs = new List<float>();
        List<float> allMinTTCs = new List<float>();
        List<float> allLowTTCRates = new List<float>();

        foreach (var kvp in trackingData)
        {
            Transform agent = kvp.Key;
            AgentTrackingData data = kvp.Value;

            if (agent == null) continue;

            totalAgents++;

            if (data.hasReachedGoal)
            {
                successCount++;
            }

            if (enableNavigationPerformanceMetrics)
            {
                bool hasObservedNavigation = data.totalTime > 0.0001f || data.pathLength > 0.0001f || data.hasReachedGoal;
                if (hasObservedNavigation)
                {
                    // All 기준:
                    // - 성공 agent는 실제 goal 도달 시점의 time/path를 사용
                    // - 미도달 agent는 관측 종료 시점까지의 active time/path를 사용
                    allTimeToGoalValues.Add(data.hasTimeToGoal ? data.timeToGoal : data.totalTime);
                    allPathLengthValues.Add(data.hasPathLengthToGoal ? data.pathLengthToGoal : data.pathLength);
                }
            }

            if (data.totalTime > 0.0001f)
            {
                float obstacleRate = data.obstacleTime / data.totalTime;
                float collisionRate = data.agentCollisionTime / data.totalTime;
                float dangerZoneViolationRate = data.dangerZoneTime / data.totalTime;

                allObstacleRates.Add(obstacleRate);
                allAgentCollisionRates.Add(collisionRate);
                allDangerZoneViolationRates.Add(dangerZoneViolationRate);

                if (enableSocialContextMetrics)
                {
                    float avgNearestDistance = CalculateAverageNearestAgentDistance(data);
                    if (IsFiniteMetric(avgNearestDistance))
                    {
                        allAvgNearestAgentDistances.Add(avgNearestDistance);
                    }

                    float avgTTC = CalculateAverageTTC(data);
                    if (IsFiniteMetric(avgTTC))
                    {
                        allAvgTTCs.Add(avgTTC);
                    }

                    if (IsFiniteMetric(data.minTTC))
                    {
                        allMinTTCs.Add(data.minTTC);
                    }

                    float lowTTCRate = CalculateLowTTCRate(data);
                    allLowTTCRates.Add(lowTTCRate);
                }
            }
        }

        float goalSuccessRate = totalAgents > 0 ? (float)successCount / totalAgents : 0f;
        float averageTimeToGoalForSTE = CalculateMeanFiniteMetric(allTimeToGoalValues);
        float averagePathLengthForSTE = CalculateMeanFiniteMetric(allPathLengthValues);
        float ste = CalculateSTE(averageTimeToGoalForSTE, averagePathLengthForSTE);

        string header =
            "agent collision | obstacle collision | GSR | Danger Zone Violation | " +
            "Time To Goal (All) | Path Length (All) | STE | Nearest Agent Distance | " +
            $"Finite TTC (≤{maxTTCToRecord:F1}) | Min TTC(s) | Low-TTC Rate (≤{lowTTCThreshold:F2}s)";

        string values =
            $"{FormatAveragePercent(allAgentCollisionRates)} | " +
            $"{FormatAveragePercent(allObstacleRates)} | " +
            $"{goalSuccessRate * 100f:F2}% | " +
            $"{FormatAveragePercent(allDangerZoneViolationRates)} | " +
            $"{FormatAverageMetric(allTimeToGoalValues, "s")} | " +
            $"{FormatAverageMetric(allPathLengthValues, "m")} | " +
            $"{FormatSTE(ste)} | " +
            $"{FormatAverageMetric(allAvgNearestAgentDistances, "m")} | " +
            $"{FormatAverageMetric(allAvgTTCs, "s")} | " +
            $"{FormatMinimumMetric(allMinTTCs, "s")} | " +
            $"{FormatAveragePercent(allLowTTCRates)}";

        Debug.Log($"[STCR Summary]\n{header}\n{values}");

        if (enableTrajectoryMap)
        {
            GenerateTrajectoryMap();
        }
    }

    private void GenerateTrajectoryMap()
    {
        Texture2D texture = new Texture2D(mapResolution, mapResolution);

        Color[] resetColor = new Color[mapResolution * mapResolution];
        for (int i = 0; i < resetColor.Length; i++) resetColor[i] = Color.white;
        texture.SetPixels(resetColor);

        float minX = -mapWidth / 2f;
        float minZ = -mapHeight / 2f;

        DrawColliders(texture, dangerZoneColliders, dangerZoneOutlineColor, minX, minZ);
        DrawColliders(texture, obstacleColliders, obstacleOutlineColor, minX, minZ);

        foreach (var kvp in trackingData)
        {
            List<TrajectoryPoint> path = kvp.Value.trajectory;
            if (path.Count < 2) continue;

            Vector2 prevPixel = WorldToPixel(path[0].position, minX, minZ);

            for (int i = 1; i < path.Count; i++)
            {
                Vector2 currentPixel = WorldToPixel(path[i].position, minX, minZ);
                Color color = path[i].isDanger ? dangerPathColor : normalPathColor;
                DrawLine(texture, prevPixel, currentPixel, color, 3);
                prevPixel = currentPixel;
            }

            DrawCircle(texture, prevPixel, endPointRadius, normalPathColor);
        }

        texture.Apply();

        byte[] bytes = texture.EncodeToPNG();
        string pathToFile = Path.Combine(Application.dataPath, mapFileName);
        File.WriteAllBytes(pathToFile, bytes);

        Debug.Log($"[SocialTaskCompletionRate] Trajectory map saved to: {pathToFile}");
    }

    private void DrawColliders(Texture2D texture, List<Collider> colliders, Color color, float minX, float minZ)
    {
        foreach (var col in colliders)
        {
            if (col == null) continue;

            Bounds b = col.bounds;

            Vector3 p1 = new Vector3(b.min.x, 0f, b.min.z);
            Vector3 p2 = new Vector3(b.max.x, 0f, b.min.z);
            Vector3 p3 = new Vector3(b.max.x, 0f, b.max.z);
            Vector3 p4 = new Vector3(b.min.x, 0f, b.max.z);

            Vector2 px1 = WorldToPixel(p1, minX, minZ);
            Vector2 px2 = WorldToPixel(p2, minX, minZ);
            Vector2 px3 = WorldToPixel(p3, minX, minZ);
            Vector2 px4 = WorldToPixel(p4, minX, minZ);

            DrawLine(texture, px1, px2, color, 2);
            DrawLine(texture, px2, px3, color, 2);
            DrawLine(texture, px3, px4, color, 2);
            DrawLine(texture, px4, px1, color, 2);
        }
    }

    private Vector2 WorldToPixel(Vector3 worldPos, float minX, float minZ)
    {
        float u = (worldPos.x - minX) / mapWidth;
        float v = (worldPos.z - minZ) / mapHeight;

        u = Mathf.Clamp01(u);
        v = Mathf.Clamp01(v);

        return new Vector2(u * (mapResolution - 1), v * (mapResolution - 1));
    }

    private void DrawLine(Texture2D tex, Vector2 p1, Vector2 p2, Color col, int thickness)
    {
        int x0 = (int)p1.x;
        int y0 = (int)p1.y;
        int x1 = (int)p2.x;
        int y1 = (int)p2.y;

        int dx = Mathf.Abs(x1 - x0);
        int dy = Mathf.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            DrawBrush(tex, x0, y0, col, thickness);

            if (x0 == x1 && y0 == y1) break;

            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }

    private void DrawBrush(Texture2D tex, int x, int y, Color col, int thickness)
    {
        int half = thickness / 2;
        for (int i = -half; i <= half; i++)
        {
            for (int j = -half; j <= half; j++)
            {
                if (x + i >= 0 && x + i < tex.width && y + j >= 0 && y + j < tex.height)
                {
                    tex.SetPixel(x + i, y + j, col);
                }
            }
        }
    }

    private void DrawCircle(Texture2D tex, Vector2 center, int radius, Color col)
    {
        int cx = (int)center.x;
        int cy = (int)center.y;

        for (int x = -radius; x <= radius; x++)
        {
            for (int y = -radius; y <= radius; y++)
            {
                if (x * x + y * y <= radius * radius)
                {
                    int px = cx + x;
                    int py = cy + y;

                    if (px >= 0 && px < tex.width && py >= 0 && py < tex.height)
                    {
                        tex.SetPixel(px, py, col);
                    }
                }
            }
        }
    }
}