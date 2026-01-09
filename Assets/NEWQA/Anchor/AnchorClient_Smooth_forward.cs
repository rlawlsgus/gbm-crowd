// AnchorClient_Smooth_forward.cs
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.Networking;
using Newtonsoft.Json;

public class AnchorClient_Smooth_forward : MonoBehaviour
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

    [Header("Forward-frame Execution (핵심)")]
    [Tooltip("서버 (dx,dz)를 '로컬(right,forward)'로 해석하고, 응답이 도착한 순간의 forward(yaw) 프레임에 고정해서 월드로 변환합니다.")]
    public bool interpretServerAsLocalRightForward = true;

    [Tooltip("응답이 도착한 순간의 forward(yaw)로 이동 프레임을 고정합니다. (이동 중 회전 영향 X)")]
    public bool lockMovementFrameAtResponseArrival = true;

    [Tooltip("forward 프레임 변환 시 pitch/roll 무시하고 yaw만 사용(권장)")]
    public bool useYawOnly = true;

    [Tooltip("서버 로컬축이 Unity 로컬(+Z forward, +X right)와 다르면 보정 (deg). 예: 모델 forward가 +X면 -90 또는 +90")]
    public float localYawOffsetDeg = 0f;

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

    [Tooltip("프리페치 안전 마진(초). 네트워크/서버 지연 변동 대비.")]
    public float prefetchSafetyMarginSec = 0.15f;

    [Tooltip("프리페치 최소 리드 스텝. 너무 늦게 요청하는 것을 방지.")]
    public int minLeadSteps = 2;

    [Tooltip("추가로 더 앞당겨서 프리페치할 스텝(프레임) 수. 예: 4면 4프레임 더 일찍 요청")]
    public int extraLeadSteps = 4;

    [Tooltip("trajectory 버퍼(큐) 최대 개수.")]
    public int maxBufferedTrajs = 3;

    [Header("Stitch/Smoothing Settings")]
    public int stitchSmoothSteps = 3;

    [Range(0.0f, 1.0f)]
    public float stitchLerpAlpha = 0.7f;

    public bool startFromNearestWaypoint = true;

    // ===== Internal buffer types =====
    private class TrajBatch
    {
        public float[][] traj;                 // data.output[0]
        public Vector3 responseArrivalPos;     // 응답 처리 시점 위치(참고)
        public Quaternion movementFrameRot;    // "고정" 이동 프레임(응답 도착 시 forward)
        public float stepDt;
        public float rttSec;
        public float serverLatencySec;
        public int? anchorIdx;
        public int startIndexHint;
    }

    // 서버 응답 포맷(필요 필드만)
    [Serializable]
    private class ServerResponseData
    {
        public float[][][] output;

        [JsonProperty("latency_sec")]
        public float latency_sec;

        [JsonProperty("anchor_idx")]
        public int? anchor_idx;
    }

    // 내부 변수 (camera)
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

    // Playback state for prefetch decision
    private TrajBatch _currentlyPlaying = null;
    private int _currentIndex = 0;

    void Start()
    {
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
        // 1) 첫 배치 요청(필수)
        yield return StartCoroutine(CoRequestAndEnqueueBatch());

        // 2) 재생 루프 + 프리페치 루프
        StartCoroutine(CoPlaybackLoop());

        if (enablePrefetch)
            StartCoroutine(CoPrefetchLoop());
    }

    // =======================
    // Playback / Prefetch core
    // =======================

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
                yield return null;
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

            // RTT 기반 leadSteps
            int leadSteps = Mathf.CeilToInt((_avgRttSec + prefetchSafetyMarginSec) / stepDt);
            leadSteps = Mathf.Max(minLeadSteps, leadSteps);

            // ✅ 여기서 "4프레임(=4스텝) 더 일찍" 요청하도록 앞당김
            leadSteps += Mathf.Max(0, extraLeadSteps);

            int remainingSteps = GetRemainingSteps(_currentlyPlaying, _currentIndex);

            if (remainingSteps <= leadSteps)
                yield return StartCoroutine(CoRequestAndEnqueueBatch());

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
        _requestInFlight = true;

        byte[] imageBytes = CaptureView();
        if (imageBytes == null || imageBytes.Length == 0)
        {
            _requestInFlight = false;
            yield break;
        }

        string relativeGoalStr = GetRelativeGoalPositionCsv(); // ✅ yaw-only local(right,forward)로 보냄
        string timeStr = Mathf.Max(0f, _currentRemainingTime).ToString("F2", CultureInfo.InvariantCulture);
        string historyStr = GetAnchorHistoryCsv();

        string jsonResponse = null;
        float t0 = Time.realtimeSinceStartup;

        yield return StartCoroutine(CoSendRequest(
            imageBytes,
            relativeGoalStr,
            timeStr,
            historyStr,
            (res) => { jsonResponse = res; }
        ));

        float rtt = Mathf.Max(0f, Time.realtimeSinceStartup - t0);

        if (string.IsNullOrEmpty(jsonResponse))
        {
            _requestInFlight = false;
            yield break;
        }

        ServerResponseData data = null;
        try
        {
            data = JsonConvert.DeserializeObject<ServerResponseData>(jsonResponse);
        }
        catch (Exception e)
        {
            Debug.LogError($"JSON 파싱 에러: {e.Message}\n{jsonResponse}");
            _requestInFlight = false;
            yield break;
        }

        if (data == null || data.output == null || data.output.Length == 0 || data.output[0] == null || data.output[0].Length == 0)
        {
            _requestInFlight = false;
            yield break;
        }

        float[][] traj = data.output[0];

        // startIndexHint: (0,0) 포함이면 0번 스킵 힌트
        int startIndex = 0;
        if (traj[0] != null && traj[0].Length >= 2)
        {
            float x0 = flipX ? -traj[0][0] : traj[0][0];
            float z0 = flipZ ? -traj[0][1] : traj[0][1];
            if (x0 * x0 + z0 * z0 <= predEps) startIndex = 1;
        }

        // stepDt
        int stepsToPlay = Mathf.Max(0, traj.Length - startIndex);
        float stepDt = useFpsAsServerStepTime
            ? 1f / Mathf.Max(1f, fps)
            : batchPlaybackSeconds / Mathf.Max(1, stepsToPlay);
        stepDt = Mathf.Max(minStepDt, stepDt);

        // RTT EMA 업데이트
        _avgRttSec = 0.9f * _avgRttSec + 0.1f * rtt;

        // === 핵심: 응답이 "도착한 순간"의 forward(yaw)로 이동 프레임 고정 ===
        Quaternion moveFrame = GetMoveFrameRotationNow();

        TrajBatch batch = new TrajBatch
        {
            traj = traj,
            responseArrivalPos = transform.position,
            movementFrameRot = lockMovementFrameAtResponseArrival ? moveFrame : Quaternion.identity,
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
    }

    IEnumerator CoPlayBatch(TrajBatch batch)
    {
        if (batch == null || batch.traj == null) yield break;

        // 이동 프레임: 응답 도착 시점에 고정한 것을 쓰되,
        // lockMovementFrameAtResponseArrival=false면 재생 시작 시점에 고정
        Quaternion movementFrameRot = lockMovementFrameAtResponseArrival ? batch.movementFrameRot : GetMoveFrameRotationNow();

        // Waypoints를 "현재 위치" 기준으로 펼친다. (이동 중 회전과 무관)
        Vector3 startPos = transform.position;
        Vector3[] waypoints = BuildWorldWaypoints(batch, startPos, movementFrameRot);

        if (waypoints == null || waypoints.Length == 0) yield break;

        int startIndex = Mathf.Clamp(batch.startIndexHint, 0, batch.traj.Length - 1);

        if (startFromNearestWaypoint)
        {
            int nearest = FindNearestWaypointIndex(waypoints, transform.position, startIndex);
            startIndex = Mathf.Clamp(nearest, startIndex, waypoints.Length - 1);
        }

        _currentIndex = startIndex;
        int smoothLeft = Mathf.Max(0, stitchSmoothSteps);

        for (int i = startIndex; i < waypoints.Length; i++)
        {
            _currentIndex = i;

            if (_isGoalReached) yield break;

            if (stopOnGoal && Vector3.Distance(transform.position, _globalGoalPosition) < goalReachThreshold)
            {
                _isGoalReached = true;
                if (deactivateOnGoal) gameObject.SetActive(false);
                yield break;
            }

            Vector3 target = waypoints[i];
            target.y = transform.position.y;
            _predictedWorldTarget = target;

            // 회전(look)은 마음대로 바뀌어도,
            // 이동은 waypoints(월드 타겟)로만 하니까 회전 영향 없음.
            Vector3 dir = target - transform.position;
            dir.y = 0f;
            if (dir.sqrMagnitude > 1e-10f)
            {
                Quaternion look = Quaternion.LookRotation(dir.normalized, Vector3.up);
                if (smoothLeft > 0)
                    transform.rotation = Quaternion.Slerp(transform.rotation, look, stitchLerpAlpha);
                else
                    transform.rotation = look;
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

            // remaining time 로직(원본 유지)
            if (_currentRemainingTime > 1f)
                _currentRemainingTime = Mathf.Max(0f, _currentRemainingTime - batch.stepDt);
            else
            {
                if (Vector3.Distance(transform.position, _globalGoalPosition) < 5.0f)
                    _currentRemainingTime += 1.0f;
                else
                    _currentRemainingTime += 10.0f;
            }

            yield return new WaitForSeconds(batch.stepDt);
        }
    }

    // =======================
    // Waypoint build (forward-frame lock)
    // =======================

    private Vector3[] BuildWorldWaypoints(TrajBatch batch, Vector3 startPos, Quaternion movementFrameRot)
    {
        if (batch == null || batch.traj == null) return null;

        int n = batch.traj.Length;
        Vector3[] waypoints = new Vector3[n];

        Quaternion localYawOffsetRot = Quaternion.Euler(0f, localYawOffsetDeg, 0f);

        Vector3 prevCum = Vector3.zero;
        Vector3 accum = Vector3.zero;

        for (int i = 0; i < n; i++)
        {
            var p = batch.traj[i];
            if (p == null || p.Length < 2)
            {
                waypoints[i] = startPos + accum;
                continue;
            }

            Vector3 v = ReadVecXZ(p) * predScale;

            Vector3 stepVec;
            if (offsetsAreCumulative)
            {
                stepVec = v - prevCum;
                prevCum = v;
            }
            else
            {
                stepVec = v;
            }

            Vector3 worldDelta;
            if (interpretServerAsLocalRightForward)
            {
                Vector3 localAdjusted = localYawOffsetRot * stepVec;
                worldDelta = movementFrameRot * localAdjusted; // movementFrameRot은 고정! (이동 중 회전 영향 X)
            }
            else
            {
                worldDelta = stepVec;
            }

            accum += worldDelta;
            waypoints[i] = startPos + accum;
        }

        return waypoints;
    }

    private int FindNearestWaypointIndex(Vector3[] waypoints, Vector3 currentPos, int startIndex)
    {
        int bestIdx = startIndex;
        float bestDist = float.PositiveInfinity;

        for (int i = startIndex; i < waypoints.Length; i++)
        {
            Vector3 wp = waypoints[i];
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

    private Quaternion GetMoveFrameRotationNow()
    {
        Quaternion q = transform.rotation;
        if (!useYawOnly) return q;

        Vector3 e = q.eulerAngles;
        return Quaternion.Euler(0f, e.y, 0f);
    }

    private Vector3 ReadVecXZ(float[] p)
    {
        float dx = flipX ? -p[0] : p[0];
        float dz = flipZ ? -p[1] : p[1];
        // x=right, z=forward
        return new Vector3(dx, 0f, dz);
    }

    // =======================
    // Goal/Time/Camera/Data helpers
    // =======================

    // ✅ goal_position을 "yaw-only 로컬(right,forward)" 기준으로 보내기
    //   worldOffset -> (inv agent yaw)로 unity-local -> (inv localYawOffset)로 server-local
    string GetRelativeGoalPositionCsv()
    {
        Vector3 worldOffset = _globalGoalPosition - transform.position;
        worldOffset.y = 0f;

        // yaw-only inverse (월드 -> 에이전트 로컬)
        float yaw = transform.eulerAngles.y;
        Quaternion invYaw = Quaternion.Euler(0f, -yaw, 0f);
        Vector3 unityLocal = invYaw * worldOffset; // +X right, +Z forward

        // server local 축 보정이 필요한 경우, traj와 반대 방향(역변환) 적용
        Quaternion invLocalYawOffset = Quaternion.Euler(0f, -localYawOffsetDeg, 0f);
        Vector3 serverLocal = invLocalYawOffset * unityLocal;

        float gx = flipX ? -serverLocal.x : serverLocal.x;
        float gz = flipZ ? -serverLocal.z : serverLocal.z;

        return $"{gx.ToString("F4", CultureInfo.InvariantCulture)},{gz.ToString("F4", CultureInfo.InvariantCulture)}";
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

    IEnumerator CoSendRequest(byte[] image, string goalCsv, string time, string historyCsv, Action<string> onComplete)
    {
        WWWForm form = new WWWForm();
        form.AddField("goal_position", goalCsv);
        form.AddField("remaining_time", time);

        if (historyCsv != null)
            form.AddField("history", historyCsv);

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
    }
}
