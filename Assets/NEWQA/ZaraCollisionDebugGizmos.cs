using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

[DefaultExecutionOrder(-100)] // Recorder보다 먼저 돌게
public class ZaraCollisionDebugGizmos : MonoBehaviour
{
    [Header("Refs")]
    public ZaraGroupSimulator simulator;     // 비워두면 자동 탐색
    public Transform agentsRootOverride;     // 비워두면 simulator.transform 사용

    [Header("Agent discovery")]
    public string agentNamePrefix = "ped_";
    public string cameraPathHint = "Rig 1/HeadAim/Camera";

    [Header("Prediction")]
    public float collisionRadius = 0.22f;
    public float horizonSec = 5.0f;
    public float detectionRadius = 12f; // 0이면 제한 없음
    public float fovDegFallback = 90f;

    [Tooltip("필터를 켜면(especially FOV) 충돌이 '안 보일' 수 있음. 일단 false 권장.")]
    public bool requireInFront = false;
    public bool requireInsideFov = false;

    [Header("Drawing")]
    public bool draw = true;
    public bool drawPredictedClosest = true;
    public float yLift = 0.05f;
    public bool avoidDuplicatePairs = true;

    class AgentState
    {
        public int id;
        public Transform t;
        public Camera cam;

        public bool hasPrev;
        public Vector3 prevPos;

        public bool hasPrevSimF;
        public float prevSimFrameF;

        public Vector3 vel; // world units / dataset-sec

        public bool willCollide;
        public int otherId;
        public float tMin;
        public float minDist;
        public float nowDist;
    }

    readonly Dictionary<int, AgentState> agents = new Dictionary<int, AgentState>(256);

    // publish cache
    readonly Dictionary<int, ZaraCollisionBus.CollisionResult> _pub = new Dictionary<int, ZaraCollisionBus.CollisionResult>(256);

    int lastComputedGlobalFrame = int.MinValue;

    void Awake()
    {
        if (simulator == null) simulator = GetComponent<ZaraGroupSimulator>();
        if (simulator == null) simulator = FindAny<ZaraGroupSimulator>();

        RefreshAgents(full: true);
    }

    void LateUpdate()
    {
        if (!Application.isPlaying) return;

        if (simulator == null) simulator = FindAny<ZaraGroupSimulator>();
        int gFrame = (simulator != null) ? simulator.GlobalFrame : Time.frameCount;

        // globalFrame 단위로만 계산/Publish (Recorder도 globalFrame 단위로 기록)
        if (gFrame == lastComputedGlobalFrame) return;
        lastComputedGlobalFrame = gFrame;

        RefreshAgents(full: false);
        UpdateVelocitiesAndPredictions();
        PublishToBus(gFrame);
    }

    void RefreshAgents(bool full)
    {
        Transform root = agentsRootOverride != null
            ? agentsRootOverride
            : (simulator != null ? simulator.transform : transform);

        // ped_ 자식들을 스캔해서 상태 등록
        foreach (Transform child in root)
        {
            if (child == null) continue;
            var go = child.gameObject;
            if (go == null) continue;
            if (!go.name.StartsWith(agentNamePrefix, StringComparison.Ordinal)) continue;

            int id;
            if (!TryParseId(go.name, agentNamePrefix, out id)) continue;
            if (agents.ContainsKey(id)) continue;

            agents[id] = new AgentState
            {
                id = id,
                t = child,
                cam = FindAgentCamera(child)
            };
        }

        if (full)
        {
            foreach (var kv in agents)
                if (kv.Value.cam == null) kv.Value.cam = FindAgentCamera(kv.Value.t);
        }
    }

    void UpdateVelocitiesAndPredictions()
    {
        float fps = (simulator != null) ? Mathf.Max(1f, simulator.fps) : 25f;
        float curF = (simulator != null) ? simulator.CurrentFrameF : Time.frameCount;

        // 1) velocity 업데이트
        foreach (var kv in agents)
        {
            var a = kv.Value;
            if (a.t == null) continue;
            if (!a.t.gameObject.activeInHierarchy)
            {
                a.hasPrev = false;
                a.hasPrevSimF = false;
                a.willCollide = false;
                continue;
            }

            Vector3 p = a.t.position;

            float dtDataset;
            if (simulator != null)
            {
                if (a.hasPrevSimF)
                {
                    float df = curF - a.prevSimFrameF;

                    // loop 프레임 점프 보정(최단거리)
                    if (simulator.loop)
                    {
                        float len = (simulator.MaxFrame - simulator.MinFrame + 1);
                        if (len > 1f)
                        {
                            if (df > len * 0.5f) df -= len;
                            else if (df < -len * 0.5f) df += len;
                        }
                    }

                    dtDataset = Mathf.Abs(df) / fps;
                }
                else
                {
                    dtDataset = 1f / fps;
                }

                a.prevSimFrameF = curF;
                a.hasPrevSimF = true;
            }
            else
            {
                dtDataset = Mathf.Max(1e-6f, Time.deltaTime);
            }

            dtDataset = Mathf.Max(1e-6f, dtDataset);

            a.vel = a.hasPrev ? (p - a.prevPos) / dtDataset : Vector3.zero;
            a.prevPos = p;
            a.hasPrev = true;

            a.willCollide = false;
            a.otherId = -1;
            a.tMin = -1f;
            a.minDist = float.PositiveInfinity;
            a.nowDist = float.PositiveInfinity;
        }

        // 2) 충돌 후보 선택 (각 agent당 가장 위험 1개)
        foreach (var kv in agents)
        {
            var me = kv.Value;
            if (me.t == null || !me.t.gameObject.activeInHierarchy) continue;

            Vector3 myPos = me.t.position;

            // "앞" 기준: 속도 방향이 있으면 그걸 쓰고, 없으면 카메라/forward
            Vector3 myFwd = me.vel;
            myFwd.y = 0f;
            if (myFwd.sqrMagnitude > 1e-6f)
            {
                myFwd.Normalize();
            }
            else
            {
                myFwd = (me.cam != null) ? me.cam.transform.forward : me.t.forward;
                myFwd.y = 0f;
                if (myFwd.sqrMagnitude < 1e-6f) myFwd = Vector3.forward;
                myFwd.Normalize();
            }

            float fov = (me.cam != null) ? me.cam.fieldOfView : fovDegFallback;
            float cosHalf = Mathf.Cos(0.5f * fov * Mathf.Deg2Rad);

            int bestId = -1;
            float bestMinDist = float.PositiveInfinity;
            float bestT = -1f;
            float bestNow = float.PositiveInfinity;

            foreach (var kv2 in agents)
            {
                var other = kv2.Value;
                if (other.id == me.id) continue;
                if (other.t == null || !other.t.gameObject.activeInHierarchy) continue;

                Vector3 op = other.t.position;
                Vector3 dir = op - myPos;
                dir.y = 0f;

                float distNow = dir.magnitude;
                if (distNow < 1e-6f) continue;

                if (detectionRadius > 0f && distNow > detectionRadius) continue;

                Vector3 dirN = dir / distNow;

                if (requireInFront && Vector3.Dot(myFwd, dirN) <= 0f) continue;
                if (requireInsideFov && Vector3.Dot(myFwd, dirN) < cosHalf) continue;

                float tMin, minDist;
                PredictMinDistanceTime(myPos, me.vel, op, other.vel, horizonSec, out tMin, out minDist);

                float thr = collisionRadius * 2f;
                if (minDist <= thr)
                {
                    // 가장 작은 minDist 우선
                    if (minDist < bestMinDist)
                    {
                        bestMinDist = minDist;
                        bestId = other.id;
                        bestT = tMin;
                        bestNow = distNow;
                    }
                }
            }

            if (bestId >= 0)
            {
                me.willCollide = true;
                me.otherId = bestId;
                me.tMin = bestT;
                me.minDist = bestMinDist;
                me.nowDist = bestNow;
            }
        }
    }

    void PublishToBus(int gFrame)
    {
        _pub.Clear();

        foreach (var kv in agents)
        {
            var a = kv.Value;

            bool active = (a.t != null && a.t.gameObject.activeInHierarchy);

            if (!active || !a.willCollide)
            {
                _pub[a.id] = new ZaraCollisionBus.CollisionResult
                {
                    ahead = false,
                    otherId = -1,
                    timeToMinDistSec = -1f,
                    minDist = -1f,
                    nowDist = -1f
                };
                continue;
            }

            float nowDist = a.nowDist;
            if (nowDist <= 0f || float.IsInfinity(nowDist))
            {
                // 혹시 nowDist가 비었으면 계산
                AgentState other;
                if (agents.TryGetValue(a.otherId, out other) && other != null && other.t != null)
                {
                    var p1 = a.t.position; p1.y = 0f;
                    var p2 = other.t.position; p2.y = 0f;
                    nowDist = (p2 - p1).magnitude;
                }
            }

            _pub[a.id] = new ZaraCollisionBus.CollisionResult
            {
                ahead = true,
                otherId = a.otherId,
                timeToMinDistSec = a.tMin,
                minDist = a.minDist,
                nowDist = nowDist
            };
        }

        ZaraCollisionBus.Publish(gFrame, _pub);
    }

    void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;
        if (!draw) return;

        foreach (var kv in agents)
        {
            var me = kv.Value;
            if (me.t == null || !me.t.gameObject.activeInHierarchy) continue;
            if (!me.willCollide) continue;

            AgentState other;
            if (!agents.TryGetValue(me.otherId, out other)) continue;
            if (other.t == null || !other.t.gameObject.activeInHierarchy) continue;

            if (avoidDuplicatePairs && me.id > other.id) continue;

            Vector3 p1 = me.t.position; p1.y = yLift;
            Vector3 p2 = other.t.position; p2.y = yLift;

            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(p1, collisionRadius);
            Gizmos.DrawLine(p1, p2);
            Gizmos.DrawWireSphere(p2, collisionRadius);

            if (drawPredictedClosest && me.tMin >= 0f)
            {
                float t = me.tMin;

                Vector3 c1 = me.t.position + me.vel * t;
                Vector3 c2 = other.t.position + other.vel * t;
                c1.y = yLift; c2.y = yLift;

                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(c1, collisionRadius);
                Gizmos.DrawLine(c1, c2);
                Gizmos.DrawWireSphere(c2, collisionRadius);
            }
        }
    }

    static void PredictMinDistanceTime(
        Vector3 p1, Vector3 v1,
        Vector3 p2, Vector3 v2,
        float horizon,
        out float tMin,
        out float minDist)
    {
        Vector3 p = p2 - p1;
        Vector3 v = v2 - v1;
        p.y = 0f; v.y = 0f;

        float vv = Vector3.Dot(v, v);
        if (vv < 1e-8f)
        {
            tMin = 0f;
            minDist = p.magnitude;
            return;
        }

        float t = -Vector3.Dot(p, v) / vv;
        t = Mathf.Clamp(t, 0f, Mathf.Max(0f, horizon));

        Vector3 closest = p + v * t;
        tMin = t;
        minDist = closest.magnitude;
    }

    Camera FindAgentCamera(Transform pedRoot)
    {
        if (pedRoot == null) return null;

        if (!string.IsNullOrWhiteSpace(cameraPathHint))
        {
            var t = pedRoot.Find(cameraPathHint);
            Camera cam0;
            if (t != null && t.TryGetComponent<Camera>(out cam0)) return cam0;
        }

        var cams = pedRoot.GetComponentsInChildren<Camera>(true);
        return (cams != null && cams.Length > 0) ? cams[0] : null;
    }

    static bool TryParseId(string name, string prefix, out int id)
    {
        id = -1;
        if (!name.StartsWith(prefix, StringComparison.Ordinal)) return false;
        string s = name.Substring(prefix.Length);
        return int.TryParse(s, NumberStyles.Integer, CultureInfo.InvariantCulture, out id);
    }

    static T FindAny<T>() where T : UnityEngine.Object
    {
#if UNITY_2023_1_OR_NEWER
        return UnityEngine.Object.FindFirstObjectByType<T>();
#else
        return UnityEngine.Object.FindObjectOfType<T>();
#endif
    }
}
