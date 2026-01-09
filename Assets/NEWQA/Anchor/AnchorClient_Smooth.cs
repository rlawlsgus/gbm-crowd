using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.Networking;
using Newtonsoft.Json;


public class AnchorClient_Smooth : MonoBehaviour
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
    public float predScale = 1f;                 // 서버 오프셋 스케일 보정
    public float predEps = 1e-6f;                // (0,0) 판정 임계
    public bool offsetsAreCumulative = true;     // output[t]가 (t=0 기준 누적 오프셋)인지
    public bool predIsInAgentLocalFrame = false; // 서버 오프셋이 agent 로컬축 기준이면 true
    public float yawOffsetDeg = 0f;              // 모델 forward가 +Z가 아니면 보정(90/-90/180 등)

    [Header("Playback Timing")]
    [Min(1f)] public float fps = 25f;
    public bool useFpsAsServerStepTime = true;   // true: stepDt=1/fps, false: batchPlaybackSeconds/steps
    public float batchPlaybackSeconds = 0.8f;    // (useFpsAsServerStepTime=false일 때) 20-step 전체 재생 시간
    public float minStepDt = 0.001f;

    [Header("Remaining Time Settings")]
    public float remainingTimeOverride = 50f;    // >0이면 고정 시작값, <=0이면 랜덤 사용
    public float remainingTimeRandomMin = 10f;
    public float remainingTimeRandomMax = 30f;
    [SerializeField]private float _currentRemainingTime;

    [Header("History Settings")]
    [Tooltip("서버에서 받은 anchor_idx들을 저장해서 history로 보냅니다.")]
    public bool useAnchorHistory = true;

    [Tooltip("history가 비어있을 때도 'history' 필드를 빈 문자열로 보낼지 여부")]
    public bool sendEmptyHistoryField = false;

    [Tooltip("history 최대 길이(0 이하이면 무제한)")]
    public int maxHistoryLength = 50;

    [Header("Camera Settings")]
    public Camera agentCamera;
    public int imageWidth = 224;
    public int imageHeight = 224;

    // ===== Prefetch / Stitch Settings =====
    [Header("Prefetch Settings")]
    [Tooltip("현재 배치가 끝나기 전에 미리 다음 배치를 요청합니다. (프리페치)")]
    public bool enablePrefetch = true;

    [Tooltip("프리페치 안전 마진(초). 네트워크/서버 지연 변동 대비.")]
    public float prefetchSafetyMarginSec = 0.15f;

    [Tooltip("프리페치 최소 리드 스텝. 너무 늦게 요청하는 것을 방지.")]
    public int minLeadSteps = 2;

    [Tooltip("trajectory 버퍼(큐) 최대 개수. 초과하면 가장 오래된 것을 버리거나 요청을 멈춥니다.")]
    public int maxBufferedTrajs = 3;

    [Header("Stitch/Smoothing Settings")]
    [Tooltip("새 배치 시작 시 처음 몇 스텝은 Lerp로 부드럽게 스티칭")]
    public int stitchSmoothSteps = 3;

    [Range(0.0f, 1.0f)]
    [Tooltip("스티칭 Lerp 강도(0=안 움직임, 1=즉시 target). 0.5~0.8 추천")]
    public float stitchLerpAlpha = 0.7f;

    [Tooltip("응답이 늦어서 위치가 많이 달라졌을 때, 새 traj에서 가장 가까운 waypoint부터 시작(스티칭)")]
    public bool startFromNearestWaypoint = true;

    // ===== Internal buffer types =====
    private class TrajBatch
    {
        public float[][] traj;            // data.output[0]
        public Vector3 requestBasePos;    // 요청 보낸 시점의 위치
        public Quaternion requestBaseRot; // 요청 보낸 시점의 회전
        public float stepDt;              // 이 배치 재생 step dt
        public float rttSec;              // 요청 왕복 시간(측정)
        public float serverLatencySec;    // 서버가 준 latency_sec (있으면)
        public int? anchorIdx;            // 서버가 준 anchor_idx
        public int startIndexHint;        // (0,0) 포함 스킵 처리한 시작 인덱스
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
    private float _avgRttSec = 0.8f; // 초기값(대충)
    private float _lastStepDt = 0.04f;

    // Playback state for prefetch decision
    private TrajBatch _currentlyPlaying = null;
    private int _currentIndex = 0;

    //group
    [Header("Group(Rel) Settings")]
    public bool includeGroupRel = true;

    [Tooltip("인스펙터에 그룹 멤버 Transform을 넣어주세요 (self 포함돼도 자동 제외)")]
    public List<Transform> group = new List<Transform>();

    [Tooltip("서버가 요구하는 필드명")]
    public string groupRelFieldName = "group_rel_positions";


    void Start()
    {
        InitializeCamera();
        InitializeRemainingTime();
        SetNewGlobalGoal();

        // 최초 1개는 반드시 받아야 재생 가능하니 즉시 요청
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

        // 2) 재생 루프 + (옵션) 프리페치 루프 분리
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
            // goal check
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

            // anchor history 저장
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

            // 큐가 이미 꽉 차 있으면 요청 멈춤
            if (_trajQueue.Count >= Mathf.Max(1, maxBufferedTrajs))
            {
                yield return null;
                continue;
            }

            // 아직 재생 중인 배치가 없다면(간극 발생) 바로 요청
            if (_currentlyPlaying == null)
            {
                yield return StartCoroutine(CoRequestAndEnqueueBatch());
                yield return null;
                continue;
            }

            // 리드 스텝 계산: RTT 기반
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
        _requestInFlight = true;

        // 데이터 준비 (요청 시점 base 저장)
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

        // stepDt 계산 (현재 설정 기준)
        int stepsToPlay = traj.Length;
        int startIndex = 0;

        // (0,0) 포함이면 0번 스킵 힌트
        if (traj[0] != null && traj[0].Length >= 2)
        {
            float x0 = flipX ? -traj[0][0] : traj[0][0];
            float z0 = flipZ ? -traj[0][1] : traj[0][1];
            if (x0 * x0 + z0 * z0 <= predEps) startIndex = 1;
        }
        stepsToPlay = Mathf.Max(0, traj.Length - startIndex);

        float stepDt;
        if (useFpsAsServerStepTime)
            stepDt = 1f / Mathf.Max(1f, fps);
        else
            stepDt = batchPlaybackSeconds / Mathf.Max(1, stepsToPlay);

        stepDt = Mathf.Max(minStepDt, stepDt);

        // RTT EMA 업데이트
        _avgRttSec = 0.9f * _avgRttSec + 0.1f * rtt;

        // 배치 생성
        TrajBatch batch = new TrajBatch
        {
            traj = traj,
            requestBasePos = requestBasePos,
            requestBaseRot = requestBaseRot,
            stepDt = stepDt,
            rttSec = rtt,
            serverLatencySec = data.latency_sec,
            anchorIdx = data.anchor_idx,
            startIndexHint = startIndex
        };

        // 큐에 추가 (버퍼 제한 처리)
        if (_trajQueue.Count >= Mathf.Max(1, maxBufferedTrajs))
        {
            // 가장 오래된 것 버리고 최신 넣기 (원하면 반대로 해도 됨)
            _trajQueue.Dequeue();
        }
        _trajQueue.Enqueue(batch);

        _requestInFlight = false;
    }

    IEnumerator CoPlayBatch(TrajBatch batch)
    {
        if (batch == null || batch.traj == null) yield break;

        // 현재 배치의 base를 "요청 시점"으로 쓰되,
        // 응답이 늦어 현재 위치와 크게 차이나면 startFromNearestWaypoint로 스티칭
        Vector3 basePos = transform.position;
        Quaternion baseRot = transform.rotation;

        // delta일 때를 위한 runningPos
        Vector3 runningPos = basePos;

        // 시작 인덱스 결정
        int startIndex = Mathf.Clamp(batch.startIndexHint, 0, batch.traj.Length);

        // 전체 waypoint를 월드로 펼쳐서(필요시) 가까운 곳부터 시작
        if (startFromNearestWaypoint)
        {
            int nearest = FindNearestWaypointIndex(batch, basePos, baseRot, transform.position, startIndex);
            startIndex = Mathf.Clamp(nearest, startIndex, batch.traj.Length - 1);
        }

        // 재생 상태 업데이트 (prefetch가 remainingSteps 계산할 때 사용)
        _currentIndex = startIndex;
        _lastStepDt = batch.stepDt;

        // 누적이 아니라 delta라면, startIndex 이전까지 runningPos를 쌓아야 함
        // (delta가 실제 step delta라는 가정)
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

        for (int i = startIndex; i < batch.traj.Length; i++)
        {
            _currentIndex = i;

            if (_isGoalReached) yield break;

            // 목표 도달 체크
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

            if (predIsInAgentLocalFrame)
                offset = baseRot * offset;

            Vector3 target;
            if (offsetsAreCumulative)
            {
                target = basePos + offset;
            }
            else
            {
                runningPos += offset;
                target = runningPos;
            }

            target.y = transform.position.y;
            _predictedWorldTarget = target;

            // 바라보기
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

            // 위치 업데이트 (스티칭 초반만 Lerp)
            if (smoothLeft > 0)
            {
                transform.position = Vector3.Lerp(transform.position, target, stitchLerpAlpha);
                smoothLeft--;
            }
            else
            {
                transform.position = target;
            }

            // remaining time 감소
            if (_currentRemainingTime > 1f)
                _currentRemainingTime = Mathf.Max(0f, _currentRemainingTime - batch.stepDt);
            else
            {
                if(Vector3.Distance(transform.position, _globalGoalPosition) < 5.0f)
                {
                    _currentRemainingTime += 1.0f;
                }
                else
                {
                    _currentRemainingTime += 10.0f;
                }
            }

            yield return new WaitForSeconds(batch.stepDt);
        }
    }

    int FindNearestWaypointIndex(TrajBatch batch, Vector3 basePos, Quaternion baseRot, Vector3 currentPos, int startIndex)
    {
        // 현재 위치와 가장 가까운 waypoint 인덱스를 반환
        // 너무 뒤쪽으로 점프하는 걸 막고 싶으면 탐색 범위를 제한해도 됨.
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
            {
                wp = basePos + offset;
            }
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
    // Goal/Time/Camera/Data helpers
    // =======================

    //string GetRelativeGoalPositionCsv()
    //{
    //    Vector3 offset = _globalGoalPosition - transform.position;

    //    float gx = flipX ? -offset.x : offset.x;
    //    float gz = flipZ ? -offset.z : offset.z;

    //    return $"{gx.ToString("F4", CultureInfo.InvariantCulture)},{gz.ToString("F4", CultureInfo.InvariantCulture)}";
    //}

    string GetRelativeGoalPositionJson()
    {
        Vector3 offset = _globalGoalPosition - transform.position;
        float gx = flipX ? -offset.x : offset.x;
        float gz = flipZ ? -offset.z : offset.z;

        // "[0.1823,1.3791]"
        return JsonConvert.SerializeObject(new float[] { gx, gz });
    }

    private string BuildGroupRel2DListJson()
    {
        if (!includeGroupRel) return null;
        if (group == null || group.Count == 0) return null;

        Vector3 myPos = transform.position;

        // [[x,z],[x,z],...]
        var relList = new List<List<float>>(group.Count);

        for (int i = 0; i < group.Count; i++)
        {
            Transform m = group[i];
            if (m == null) continue;
            if (m == transform) continue; // self 제외

            Vector3 rel = m.position - myPos;

            float rx = flipX ? -rel.x : rel.x;
            float rz = flipZ ? -rel.z : rel.z;

            relList.Add(new List<float> { rx, rz });
        }

        if (relList.Count == 0) return null;

        // Newtonsoft.Json 사용: "[[0.0385,-0.5449],[0.12,0.34]]" 같은 문자열 생성
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

        //group_rel_positions = [[x,z],[x,z],...]
        if (!string.IsNullOrEmpty(groupRelJson))
            form.AddField(groupRelFieldName, groupRelJson); // 기본 "group_rel_positions"

        //Debug.Log($"groupRelJson = {groupRelJson ?? "NULL"}");
        //Debug.Log($"group.Count = {(group == null ? -1 : group.Count)}");
        //for (int i = 0; group != null && i < group.Count; i++)
        //{
        //    var m = group[i];
        //    Debug.Log($"group[{i}] = {(m ? m.name : "NULL")} (isSelf={(m == transform)})");
        //}

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

        _globalGoalPosition = Goal.position;
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(_globalGoalPosition,goalReachThreshold);
        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position + Vector3.up * 0.1f, _globalGoalPosition + Vector3.up * 0.1f);

        if (!Application.isPlaying) return;
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position + Vector3.up * 0.1f, _predictedWorldTarget + Vector3.up * 0.1f);
    }
}
