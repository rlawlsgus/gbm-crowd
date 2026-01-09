using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.Networking;
using Newtonsoft.Json;

public class Anchor3Client : MonoBehaviour
{
    [Header("Server Settings")]
    [SerializeField] private string serverUrl = "http://163.152.162.171:8000/predict";
    public int requestTimeoutSec = 5;

    [Header("Goal & Stop Settings")]
    public Transform Goal;
    public float goalReachThreshold = 1.0f;
    public bool stopOnGoal = true;
    public bool deactivateOnGoal = true;

    [Header("Coordinate Settings")]
    public bool flipX = false;
    public bool flipZ = false;

    [Header("Prediction Interpret Settings")]
    public float predScale = 1f;
    public float predEps = 1e-6f;
    public bool offsetsAreCumulative = true;
    public bool predIsInAgentLocalFrame = false;
    public float yawOffsetDeg = 0f;

    [Header("Playback Timing")]
    [Min(1f)] public float fps = 25f;
    public bool useFpsAsServerStepTime = true;
    public float batchPlaybackSeconds = 0.8f;
    public float minStepDt = 0.001f;

    [Header("Remaining Time Settings")]
    public float remainingTimeOverride = 50f;
    public float remainingTimeRandomMin = 10f;
    public float remainingTimeRandomMax = 30f;
    [SerializeField] private float _currentRemainingTime;

    [Header("History Settings")]
    public bool useAnchorHistory = true;
    public bool sendEmptyHistoryField = false;
    public int maxHistoryLength = 50;

    [Header("Camera Settings")]
    public Camera agentCamera;
    public int imageWidth = 224;
    public int imageHeight = 224;

    // ===== Prefetch / Stitch Settings =====
    [Header("Prefetch Settings")]
    public bool enablePrefetch = true;
    public float prefetchSafetyMarginSec = 0.15f;
    public int minLeadSteps = 2;
    public int maxBufferedTrajs = 3;

    [Header("Stitch/Smoothing Settings")]
    public int stitchSmoothSteps = 3;

    [Range(0.0f, 1.0f)]
    public float stitchLerpAlpha = 0.7f;

    public bool startFromNearestWaypoint = true;

    // ===== Trajectory selection / visualization =====
    [Header("Trajectory Selection")]
    public float trajAngleWeight = 1f;
    public float trajGoalWeight = 1f;
    public float trajPathLenWeight = 0f;

    [Tooltip("연속 target 간 이동 간격이 이 값(미터) 이하이면 회전 업데이트를 하지 않습니다.")]
    public float rotationSkipMoveThreshold = 0.01f;

    [Header("Trajectory End Collision Filter")]
    public bool disqualifyOnEndCollision = true;
    public float endCollisionCheckRadius = 0.3f;
    [Tooltip("끝점에서 Agent 충돌 체크 반경(Agent만)")]
    public float agentEndCollisionCheckRadius = 0.1f;

    public LayerMask endCollisionLayerMask = ~0;
    public QueryTriggerInteraction endCollisionTriggerInteraction = QueryTriggerInteraction.Collide;

    [Header("Retry When All Candidates Blocked")]
    [Tooltip("모든 후보가 end collision으로 막히면 잠깐 멈춘 뒤 재요청합니다.")]
    public bool retryWhenAllCandidatesBlocked = true;

    [Tooltip("모든 후보가 막혔을 때 재요청까지 대기 시간(초)")]
    public float allBlockedRetryDelaySec = 0.2f;

    [Tooltip("한 번의 CoRequestAndEnqueueBatch 호출에서 최대 재시도 횟수")]
    public int maxAllBlockedRetriesPerRequest = 5;

    [Header("Trajectory Gizmos")]
    public bool drawCandidateTrajs = true;
    public float gizmoTrajYOffset = 0.05f;
    public float gizmoEndSphereRadius = 0.07f;

    // ===== Internal buffer types =====
    private class TrajBatch
    {
        public float[][] traj;
        public Vector3 requestBasePos;
        public Quaternion requestBaseRot;
        public float stepDt;
        public float rttSec;
        public float serverLatencySec;
        public int? anchorIdx;
        public int startIndexHint;
    }

    [Serializable]
    private class ServerResponseData
    {
        public float[][] output1;
        public float[][] output2;
        public float[][] output3;

        public float[][][] output;

        public float latency_sec;
        public int? anchor_idx;
    }

    private class TrajViz
    {
        public Vector3[] points;
        public float angleDeg;
        public float endDist;
        public float pathLen;
        public bool endBlocked;
        public bool chosen;
    }

    // camera
    private RenderTexture _renderTexture;
    private Texture2D _texture2D;

    private Vector3 _globalGoalPosition;
    private Vector3 _predictedWorldTarget;
    private bool _isGoalReached = false;

    // anchor history
    private readonly List<int> _anchorHistory = new List<int>();
    private int? _lastAnchorIdx = null;

    // Prefetch buffers
    private readonly Queue<TrajBatch> _trajQueue = new Queue<TrajBatch>();
    private bool _requestInFlight = false;
    private float _avgRttSec = 0.8f;
    private float _lastStepDt = 0.04f;

    // Playback state
    private TrajBatch _currentlyPlaying = null;
    private int _currentIndex = 0;

    // group
    [Header("Group(Rel) Settings")]
    public bool includeGroupRel = true;
    public List<Transform> group = new List<Transform>();
    public string groupRelFieldName = "group_rel_positions";

    // Candidate viz cache
    private TrajViz[] _lastTrajViz = null;
    private int _lastChosenTrajIndex = -1;

    // Overlap buffer
    private readonly Collider[] _overlapBuf = new Collider[64];

    void Start()
    {
        if (animator == null) animator = GetComponentInChildren<Animator>();

        InitializeCamera();
        InitializeRemainingTime();
        SetNewGlobalGoal();
        StartCoroutine(CoBootstrapAndRun());
    }

    void OnDestroy()
    {
        if (_renderTexture != null) _renderTexture.Release();
        if (_texture2D != null) Destroy(_texture2D);
    }

    IEnumerator CoBootstrapAndRun()
    {
        yield return StartCoroutine(CoRequestAndEnqueueBatch());

        StartCoroutine(CoPlaybackLoop());

        if (enablePrefetch)
            StartCoroutine(CoPrefetchLoop());
    }

    IEnumerator CoPlaybackLoop()
    {
        while (!_isGoalReached)
        {
            if (stopOnGoal && Vector3.Distance(transform.position, _globalGoalPosition) < goalReachThreshold)
            {
                _isGoalReached = true;
                if (deactivateOnGoal) gameObject.SetActive(false);
                yield break;
            }

            if (_trajQueue.Count == 0)
            {
                // ✅ prefetch가 꺼져 있어도 계속 새 배치 요청하도록
                if (!enablePrefetch && !_requestInFlight)
                {
                    yield return StartCoroutine(CoRequestAndEnqueueBatch());
                }
                else
                {
                    yield return null;
                }
                continue;
            }

            _currentlyPlaying = _trajQueue.Dequeue();
            if (_currentlyPlaying == null || _currentlyPlaying.traj == null || _currentlyPlaying.traj.Length == 0)
            {
                _currentlyPlaying = null;
                yield return null;
                continue;
            }

            if (useAnchorHistory && _currentlyPlaying.anchorIdx.HasValue)
                AppendAnchorHistory(_currentlyPlaying.anchorIdx.Value);

            yield return StartCoroutine(CoPlayBatch(_currentlyPlaying));
            _currentlyPlaying = null;
        }
    }

    IEnumerator CoPrefetchLoop()
    {
        while (!_isGoalReached)
        {
            if (_requestInFlight) { yield return null; continue; }

            if (_trajQueue.Count >= Mathf.Max(1, maxBufferedTrajs))
            {
                yield return null;
                continue;
            }

            if (_currentlyPlaying == null)
            {
                yield return StartCoroutine(CoRequestAndEnqueueBatch());
                yield return null;
                continue;
            }

            float stepDt = Mathf.Max(minStepDt, _currentlyPlaying.stepDt);
            int leadSteps = Mathf.CeilToInt((_avgRttSec + prefetchSafetyMarginSec) / stepDt);
            leadSteps = Mathf.Max(minLeadSteps, leadSteps);

            int remainingSteps = GetRemainingSteps(_currentlyPlaying, _currentIndex);

            if (remainingSteps <= leadSteps)
            {
                yield return StartCoroutine(CoRequestAndEnqueueBatch());
            }

            yield return null;
        }
    }

    int GetRemainingSteps(TrajBatch batch, int currentIndex)
    {
        if (batch == null || batch.traj == null) return 0;
        int start = Mathf.Clamp(batch.startIndexHint, 0, batch.traj.Length);
        int idx = Mathf.Clamp(currentIndex, start, batch.traj.Length);
        return Mathf.Max(0, batch.traj.Length - idx);
    }

    IEnumerator CoRequestAndEnqueueBatch()
    {
        if (_requestInFlight) yield break;
        _requestInFlight = true;

        int allBlockedRetryCount = 0;

        while (!_isGoalReached)
        {
            // ---- 요청 시점 base ----
            Vector3 requestBasePos = transform.position;
            Quaternion requestBaseRot = transform.rotation;

            byte[] imageBytes = CaptureView();
            if (imageBytes == null || imageBytes.Length == 0)
            {
                _requestInFlight = false;
                yield break;
            }

            string relativeGoalStr = GetRelativeGoalPositionJson();
            string groupRelJson = BuildGroupRel2DListJson();
            string timeStr = Mathf.Max(0f, _currentRemainingTime).ToString("F2", CultureInfo.InvariantCulture);
            string historyStr = GetAnchorHistoryCsv();

            string jsonResponse = null;
            float t0 = Time.realtimeSinceStartup;

            yield return StartCoroutine(CoSendRequest(
                imageBytes,
                relativeGoalStr,
                timeStr,
                historyStr,
                groupRelJson,
                (res) => { jsonResponse = res; }
            ));

            float rtt = Mathf.Max(0f, Time.realtimeSinceStartup - t0);

            if (string.IsNullOrEmpty(jsonResponse))
            {
                // 네트워크/서버 실패 -> 다음 프레임에 재시도
                _avgRttSec = 0.9f * _avgRttSec + 0.1f * rtt;
                yield return new WaitForSeconds(allBlockedRetryDelaySec);
                continue;
            }

            ServerResponseData data = null;
            Exception parseEx = null;

            try
            {
                data = JsonConvert.DeserializeObject<ServerResponseData>(jsonResponse);
            }
            catch (Exception e)
            {
                parseEx = e;
            }

            // catch 밖에서 yield
            if (parseEx != null || data == null)
            {
                Debug.LogError($"JSON 파싱 에러: {parseEx?.Message}\n{jsonResponse}");
                yield return new WaitForSeconds(Mathf.Max(0f, allBlockedRetryDelaySec));
                continue;
            }


            List<float[][]> candidates = ExtractTrajCandidates(data);
            if (candidates == null || candidates.Count == 0)
            {
                yield return new WaitForSeconds(allBlockedRetryDelaySec);
                continue;
            }

            List<int> startHints = new List<int>(candidates.Count);
            for (int i = 0; i < candidates.Count; i++)
                startHints.Add(ComputeStartIndexHint(candidates[i]));

            Vector3 evalBasePos = transform.position;
            Quaternion evalBaseRot = transform.rotation;

            int chosenIdx = ChooseBestTrajectoryIndex(
                candidates,
                startHints,
                evalBasePos,
                evalBaseRot,
                _globalGoalPosition,
                out TrajViz[] vizArr
            );

            _lastTrajViz = vizArr;
            _lastChosenTrajIndex = chosenIdx;

            _avgRttSec = 0.9f * _avgRttSec + 0.1f * rtt;

            // ✅ 전부 충돌로 막힌 경우: "멈춤 + 재요청"
            if (chosenIdx < 0)
            {
                _predictedWorldTarget = transform.position; // 시각화 라인도 현재로
                ForceBlend(0f);
                if (!retryWhenAllCandidatesBlocked)
                {
                    _requestInFlight = false;
                    yield break;
                }

                allBlockedRetryCount++;
                if (allBlockedRetryCount > Mathf.Max(0, maxAllBlockedRetriesPerRequest))
                {
                    // 이 호출에서는 더 이상 재시도 안 하고 빠져나감 (다음 루프/다음 호출에서 또 시도됨)
                    _requestInFlight = false;
                    yield break;
                }

                yield return new WaitForSeconds(Mathf.Max(0f, allBlockedRetryDelaySec));
                continue; // 다시 요청
            }

            if (chosenIdx >= candidates.Count || candidates[chosenIdx] == null || candidates[chosenIdx].Length == 0)
            {
                yield return new WaitForSeconds(allBlockedRetryDelaySec);
                continue;
            }

            float[][] chosenTraj = candidates[chosenIdx];
            int startIndex = startHints[chosenIdx];

            int stepsToPlay = Mathf.Max(0, chosenTraj.Length - startIndex);

            float stepDt;
            if (useFpsAsServerStepTime)
                stepDt = 1f / Mathf.Max(1f, fps);
            else
                stepDt = batchPlaybackSeconds / Mathf.Max(1, stepsToPlay);

            stepDt = Mathf.Max(minStepDt, stepDt);

            TrajBatch batch = new TrajBatch
            {
                traj = chosenTraj,
                requestBasePos = requestBasePos,
                requestBaseRot = requestBaseRot,
                stepDt = stepDt,
                rttSec = rtt,
                serverLatencySec = data.latency_sec,
                anchorIdx = data.anchor_idx,
                startIndexHint = startIndex
            };

            if (_trajQueue.Count >= Mathf.Max(1, maxBufferedTrajs))
                _trajQueue.Dequeue();

            _trajQueue.Enqueue(batch);

            _requestInFlight = false;
            yield break; // ✅ 성공적으로 1개 배치 넣었으면 종료
        }

        _requestInFlight = false;
    }

    IEnumerator CoPlayBatch(TrajBatch batch)
    {
        if (batch == null || batch.traj == null) yield break;
        Vector3 prevPosForSpeed = transform.position;

        Vector3 basePos = transform.position;
        Quaternion baseRot = transform.rotation;

        Vector3 runningPos = basePos;

        int startIndex = Mathf.Clamp(batch.startIndexHint, 0, batch.traj.Length);

        if (startFromNearestWaypoint)
        {
            int nearest = FindNearestWaypointIndex(batch, basePos, baseRot, transform.position, startIndex);
            startIndex = Mathf.Clamp(nearest, startIndex, batch.traj.Length - 1);
        }

        _currentIndex = startIndex;
        _lastStepDt = batch.stepDt;

        if (!offsetsAreCumulative && startIndex > 0)
        {
            runningPos = basePos;
            for (int k = 0; k < startIndex; k++)
            {
                var pk = batch.traj[k];
                if (pk == null || pk.Length < 2) continue;

                float dxk = flipX ? -pk[0] : pk[0];
                float dzk = flipZ ? -pk[1] : pk[1];
                Vector3 offk = new Vector3(dxk, 0f, dzk) * predScale;
                if (predIsInAgentLocalFrame) offk = baseRot * offk;
                runningPos += offk;
            }
        }

        int smoothLeft = Mathf.Max(0, stitchSmoothSteps);

        float rotSkipSqr = rotationSkipMoveThreshold * rotationSkipMoveThreshold;
        Vector3 prevPlannedTarget = transform.position;

        for (int i = startIndex; i < batch.traj.Length; i++)
        {
            _currentIndex = i;

            if (_isGoalReached) yield break;

            if (stopOnGoal && Vector3.Distance(transform.position, _globalGoalPosition) < goalReachThreshold)
            {
                _isGoalReached = true;
                if (deactivateOnGoal) gameObject.SetActive(false);
                yield break;
            }

            var p = batch.traj[i];
            if (p == null || p.Length < 2) { yield return new WaitForSeconds(batch.stepDt); continue; }

            float dx = flipX ? -p[0] : p[0];
            float dz = flipZ ? -p[1] : p[1];

            Vector3 offset = new Vector3(dx, 0f, dz) * predScale;
            if (predIsInAgentLocalFrame) offset = baseRot * offset;

            Vector3 target;
            if (offsetsAreCumulative)
                target = basePos + offset;
            else
            {
                runningPos += offset;
                target = runningPos;
            }

            target.y = transform.position.y;
            _predictedWorldTarget = target;

            Vector3 plannedGap = target - prevPlannedTarget;
            plannedGap.y = 0f;
            bool skipRotationThisStep = (plannedGap.sqrMagnitude <= rotSkipSqr);
            prevPlannedTarget = target;

            if (!skipRotationThisStep)
            {
                Vector3 dir = target - transform.position;
                dir.y = 0f;
                if (dir.sqrMagnitude > 1e-10f)
                {
                    Quaternion look = Quaternion.LookRotation(dir.normalized, Vector3.up);
                    look = look * Quaternion.Euler(0f, yawOffsetDeg, 0f);

                    if (smoothLeft > 0)
                        transform.rotation = Quaternion.Slerp(transform.rotation, look, stitchLerpAlpha);
                    else
                        transform.rotation = look;
                }
            }

            if (smoothLeft > 0)
            {
                transform.position = Vector3.Lerp(transform.position, target, stitchLerpAlpha);
                smoothLeft--;
            }
            else
            {
                transform.position = target;
            }

            // --- Animator Blend 업데이트 (이번 step의 실제 이동량 기반) ---
            Vector3 newPosForSpeed = transform.position;
            Vector3 delta = newPosForSpeed - prevPosForSpeed;
            delta.y = 0f;

            float stepDt = Mathf.Max(0.0001f, batch.stepDt);   // WaitForSeconds(stepDt) 쓰는 구조라 stepDt가 더 맞음
            float speedMps = delta.magnitude / stepDt;

            UpdateBlendBySpeed(speedMps);


            if (_currentRemainingTime > 1f)
                _currentRemainingTime = Mathf.Max(0f, _currentRemainingTime - batch.stepDt);
            else
            {
                if (Vector3.Distance(transform.position, _globalGoalPosition) < 5.0f)
                    _currentRemainingTime += 0.5f;
                else
                    _currentRemainingTime += 1.0f;
            }

            yield return new WaitForSeconds(batch.stepDt);
        }
    }

    int FindNearestWaypointIndex(TrajBatch batch, Vector3 basePos, Quaternion baseRot, Vector3 currentPos, int startIndex)
    {
        int bestIdx = startIndex;
        float bestDist = float.PositiveInfinity;

        Vector3 runningPos = basePos;

        for (int i = startIndex; i < batch.traj.Length; i++)
        {
            var p = batch.traj[i];
            if (p == null || p.Length < 2) continue;

            float dx = flipX ? -p[0] : p[0];
            float dz = flipZ ? -p[1] : p[1];

            Vector3 offset = new Vector3(dx, 0f, dz) * predScale;
            if (predIsInAgentLocalFrame) offset = baseRot * offset;

            Vector3 wp;
            if (offsetsAreCumulative)
                wp = basePos + offset;
            else
            {
                runningPos += offset;
                wp = runningPos;
            }

            wp.y = currentPos.y;
            float d = (wp - currentPos).sqrMagnitude;
            if (d < bestDist)
            {
                bestDist = d;
                bestIdx = i;
            }
        }

        return bestIdx;
    }

    // =======================
    // Candidate extraction / scoring / visualization
    // =======================

    private List<float[][]> ExtractTrajCandidates(ServerResponseData data)
    {
        var list = new List<float[][]>(3);
        if (data == null) return list;

        if (data.output1 != null) list.Add(data.output1);
        if (data.output2 != null) list.Add(data.output2);
        if (data.output3 != null) list.Add(data.output3);

        if (list.Count == 0 && data.output != null)
        {
            for (int i = 0; i < data.output.Length && i < 3; i++)
                if (data.output[i] != null) list.Add(data.output[i]);
        }

        for (int i = list.Count - 1; i >= 0; i--)
            if (list[i] == null || list[i].Length == 0) list.RemoveAt(i);

        return list;
    }

    private int ComputeStartIndexHint(float[][] traj)
    {
        if (traj == null || traj.Length == 0) return 0;
        if (traj[0] == null || traj[0].Length < 2) return 0;

        float x0 = flipX ? -traj[0][0] : traj[0][0];
        float z0 = flipZ ? -traj[0][1] : traj[0][1];

        return (x0 * x0 + z0 * z0 <= predEps) ? 1 : 0;
    }

    private int ChooseBestTrajectoryIndex(
        List<float[][]> candidates,
        List<int> startHints,
        Vector3 basePos,
        Quaternion baseRot,
        Vector3 goalPos,
        out TrajViz[] vizArr)
    {
        vizArr = null;
        if (candidates == null || candidates.Count == 0) return -1;

        vizArr = new TrajViz[candidates.Count];

        Vector3 fwd = transform.forward;
        fwd.y = 0f;
        if (fwd.sqrMagnitude < 1e-10f) fwd = Vector3.forward;
        fwd.Normalize();

        int bestIdx = -1;
        float bestScore = float.PositiveInfinity;

        for (int i = 0; i < candidates.Count; i++)
        {
            float[][] traj = candidates[i];
            int startIndex = (startHints != null && i < startHints.Count) ? startHints[i] : 0;

            Vector3[] pts = BuildWorldPoints(traj, startIndex, basePos, baseRot);

            float angleDeg = 999f;
            float endDist = 999999f;
            float pathLen = 0f;
            bool endBlocked = true;

            if (pts != null && pts.Length > 0)
            {
                Vector3 first = pts[0];
                Vector3 dir = first - basePos; dir.y = 0f;

                if (dir.sqrMagnitude > 1e-10f)
                {
                    dir.Normalize();
                    angleDeg = Vector3.Angle(fwd, dir);
                }
                else angleDeg = 180f;

                Vector3 last = pts[pts.Length - 1];
                last.y = goalPos.y;
                endDist = Vector3.Distance(last, goalPos);

                pathLen = ComputePathLengthXZ(pts);

                endBlocked = IsEndPointBlocked(pts[pts.Length - 1]);
            }

            vizArr[i] = new TrajViz
            {
                points = pts,
                angleDeg = angleDeg,
                endDist = endDist,
                pathLen = pathLen,
                endBlocked = endBlocked,
                chosen = false
            };

            if (endBlocked) continue; // ✅ 충돌이면 무조건 선택 불가

            float score =
                trajGoalWeight * endDist +
                trajAngleWeight * angleDeg;

            //if (_currentRemainingTime<10)
            //{
            //    score =
            //    1.0f * endDist +
            //    trajAngleWeight * 0 * angleDeg;
            //}

            if (score < bestScore)
            {
                bestScore = score;
                bestIdx = i;
            }
        }

        if (bestIdx >= 0 && bestIdx < vizArr.Length)
            vizArr[bestIdx].chosen = true;

        return bestIdx; // ✅ 전부 막히면 -1
    }

    private Vector3[] BuildWorldPoints(float[][] traj, int startIndex, Vector3 basePos, Quaternion baseRot)
    {
        if (traj == null || traj.Length == 0) return null;

        startIndex = Mathf.Clamp(startIndex, 0, traj.Length);
        int n = traj.Length - startIndex;
        if (n <= 0) return null;

        Vector3[] pts = new Vector3[n];
        Vector3 runningPos = basePos;

        for (int i = startIndex; i < traj.Length; i++)
        {
            var p = traj[i];
            if (p == null || p.Length < 2)
            {
                pts[i - startIndex] = (i == startIndex) ? basePos : pts[i - startIndex - 1];
                continue;
            }

            float dx = flipX ? -p[0] : p[0];
            float dz = flipZ ? -p[1] : p[1];

            Vector3 offset = new Vector3(dx, 0f, dz) * predScale;
            if (predIsInAgentLocalFrame) offset = baseRot * offset;

            Vector3 wp;
            if (offsetsAreCumulative)
                wp = basePos + offset;
            else
            {
                runningPos += offset;
                wp = runningPos;
            }

            wp.y = basePos.y;
            pts[i - startIndex] = wp;
        }

        return pts;
    }

    private float ComputePathLengthXZ(Vector3[] pts)
    {
        if (pts == null || pts.Length < 2) return 0f;

        float sum = 0f;
        for (int i = 0; i < pts.Length - 1; i++)
        {
            Vector3 a = pts[i]; a.y = 0f;
            Vector3 b = pts[i + 1]; b.y = 0f;
            sum += Vector3.Distance(a, b);
        }
        return sum;
    }

    private bool IsEndPointBlocked(Vector3 endPos)
    {
        if (!disqualifyOnEndCollision) return false;

        // 1) Agent는 더 작은 반경으로 체크
        if (IsEndPointBlockedByTag(endPos, Mathf.Max(0.001f, agentEndCollisionCheckRadius), checkAgent: true, checkOthers: false))
            return true;

        // 2) 나머지는 기존 반경으로 체크
        if (IsEndPointBlockedByTag(endPos, Mathf.Max(0.001f, endCollisionCheckRadius), checkAgent: false, checkOthers: true))
            return true;

        return false;
    }

    private bool IsEndPointBlockedByTag(Vector3 endPos, float radius, bool checkAgent, bool checkOthers)
    {
        int hitCount = Physics.OverlapSphereNonAlloc(
            endPos,
            radius,
            _overlapBuf,
            endCollisionLayerMask,
            endCollisionTriggerInteraction
        );

        for (int i = 0; i < hitCount; i++)
        {
            Collider c = _overlapBuf[i];
            if (c == null) continue;

            // self/children 무시
            if (c.transform == transform || c.transform.IsChildOf(transform))
                continue;

            // 부모까지 훑어서 태그 판정
            Transform t = c.transform;
            while (t != null)
            {
                if (checkAgent && t.CompareTag("Agent")) return true;

                if (checkOthers && (
                        t.CompareTag("Obstacle") ||
                        t.CompareTag("Building") ||
                        t.CompareTag("Vehicle")))
                    return true;

                t = t.parent;
            }
        }

        return false;
    }

    private bool HasBlockedTagInParents(Transform t)
    {
        while (t != null)
        {
            if (t.CompareTag("Agent") ||
                t.CompareTag("Obstacle") ||
                t.CompareTag("Building") ||
                t.CompareTag("Vehicle"))
                return true;

            t = t.parent;
        }
        return false;
    }

    // =======================
    // Goal/Time/Camera/Data helpers
    // =======================

    string GetRelativeGoalPositionJson()
    {
        Vector3 offset = _globalGoalPosition - transform.position;
        float gx = flipX ? -offset.x : offset.x;
        float gz = flipZ ? -offset.z : offset.z;
        return JsonConvert.SerializeObject(new float[] { gx, gz });
    }

    private string BuildGroupRel2DListJson()
    {
        if (!includeGroupRel) return null;
        if (group == null || group.Count == 0) return null;

        Vector3 myPos = transform.position;
        var relList = new List<List<float>>(group.Count);

        for (int i = 0; i < group.Count; i++)
        {
            Transform m = group[i];
            if (m == null) continue;
            if (m == transform) continue;

            Vector3 rel = m.position - myPos;
            float rx = flipX ? -rel.x : rel.x;
            float rz = flipZ ? -rel.z : rel.z;
            relList.Add(new List<float> { rx, rz });
        }

        if (relList.Count == 0) return null;
        return JsonConvert.SerializeObject(relList);
    }

    void InitializeRemainingTime()
    {
        if (remainingTimeOverride > 0f)
            _currentRemainingTime = remainingTimeOverride;
        else
            _currentRemainingTime = UnityEngine.Random.Range(remainingTimeRandomMin, remainingTimeRandomMax);
    }

    void SetNewGlobalGoal()
    {
        _globalGoalPosition = (Goal != null) ? Goal.position : transform.position;
    }

    string GetAnchorHistoryCsv()
    {
        if (!useAnchorHistory) return null;
        if (_anchorHistory.Count == 0) return sendEmptyHistoryField ? "" : null;
        return string.Join(",", _anchorHistory);
    }

    void AppendAnchorHistory(int anchorIdx)
    {
        _lastAnchorIdx = anchorIdx;
        _anchorHistory.Add(anchorIdx);

        if (maxHistoryLength > 0)
        {
            int overflow = _anchorHistory.Count - maxHistoryLength;
            if (overflow > 0) _anchorHistory.RemoveRange(0, overflow);
        }
    }

    IEnumerator CoSendRequest(
        byte[] image,
        string goalCsv,
        string time,
        string historyCsv,
        string groupRelJson,
        Action<string> onComplete)
    {
        WWWForm form = new WWWForm();

        form.AddField("goal_position", goalCsv);
        form.AddField("remaining_time", time);

        if (historyCsv != null)
            form.AddField("history", historyCsv);

        if (!string.IsNullOrEmpty(groupRelJson))
            form.AddField(groupRelFieldName, groupRelJson);

        form.AddBinaryData("image", image, "capture.png", "image/png");

        using (UnityWebRequest www = UnityWebRequest.Post(serverUrl, form))
        {
            www.timeout = Mathf.Max(1, requestTimeoutSec);
            yield return www.SendWebRequest();

            if (www.result == UnityWebRequest.Result.Success)
                onComplete?.Invoke(www.downloadHandler.text);
            else
                onComplete?.Invoke(null);
        }
    }

    void InitializeCamera()
    {
        if (agentCamera == null) agentCamera = GetComponentInChildren<Camera>();
        _renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        _texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
    }

    byte[] CaptureView()
    {
        if (agentCamera == null || _renderTexture == null || _texture2D == null) return null;

        RenderTexture prev = RenderTexture.active;

        agentCamera.targetTexture = _renderTexture;
        agentCamera.Render();

        RenderTexture.active = _renderTexture;
        _texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        _texture2D.Apply();

        agentCamera.targetTexture = null;
        RenderTexture.active = prev;

        return _texture2D.EncodeToPNG();
    }

    void OnDrawGizmos()
    {
        if (Goal != null) _globalGoalPosition = Goal.position;

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(_globalGoalPosition, goalReachThreshold);
        Gizmos.DrawLine(transform.position + Vector3.up * 0.1f, _globalGoalPosition + Vector3.up * 0.1f);

        if (!Application.isPlaying) return;

        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position + Vector3.up * 0.1f, _predictedWorldTarget + Vector3.up * 0.1f);

        if (!drawCandidateTrajs) return;
        if (_lastTrajViz == null) return;

        Color[] baseColors = new Color[]
        {
            new Color(1f, 0.85f, 0.1f, 1f),
            new Color(0.2f, 1f, 0.35f, 1f),
            new Color(1f, 0.2f, 1f, 1f),
        };

        for (int i = 0; i < _lastTrajViz.Length; i++)
        {
            var v = _lastTrajViz[i];
            if (v == null || v.points == null || v.points.Length < 2) continue;

            if (v.endBlocked)
                Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 1f);
            else
                Gizmos.color = v.chosen ? Color.white : baseColors[Mathf.Clamp(i, 0, baseColors.Length - 1)];

            for (int k = 0; k < v.points.Length - 1; k++)
            {
                Vector3 a = v.points[k] + Vector3.up * gizmoTrajYOffset;
                Vector3 b = v.points[k + 1] + Vector3.up * gizmoTrajYOffset;
                Gizmos.DrawLine(a, b);
            }

            Vector3 end = v.points[v.points.Length - 1] + Vector3.up * gizmoTrajYOffset;
            Gizmos.DrawWireSphere(end, gizmoEndSphereRadius);

            if (v.endBlocked && disqualifyOnEndCollision)
                Gizmos.DrawWireSphere(v.points[v.points.Length - 1] + Vector3.up * gizmoTrajYOffset, endCollisionCheckRadius);
        }
    }

    [Header("Animation")]
    public Animator animator;

    [Tooltip("Animator float 파라미터 이름")]
    public string blendParam = "Blend";

    [Tooltip("이 속도(m/s)에서 Blend=1")]
    public float maxSpeedForBlend = 2.0f;

    [Tooltip("Blend 값 변화 부드럽게 (SetFloat damp)")]
    public float blendDampTime = 0.08f;

    private void UpdateBlendBySpeed(float speedMps)
    {
        if (animator == null) return;

        float denom = Mathf.Max(0.0001f, maxSpeedForBlend);
        float blend01 = Mathf.Clamp01(speedMps / denom);

        // damping 적용 (부드럽게)
        animator.SetFloat(blendParam, blend01, Mathf.Max(0f, blendDampTime), Time.deltaTime);
    }

    private void ForceBlend(float blend01)
    {
        if (animator == null) return;
        animator.SetFloat(blendParam, Mathf.Clamp01(blend01));
    }


}
