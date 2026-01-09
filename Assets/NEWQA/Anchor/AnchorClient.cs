using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.Networking;
using Newtonsoft.Json;

// 서버 응답 구조체 정의
[System.Serializable]
public class ServerResponseData
{
    public float[][][] output;
    public float latency_sec;
    public float[] goal_position;
    public float remaining_time;

    // 서버가 내려주는 anchor index (없을 수도 있으니 nullable)
    public int? anchor_idx;
}

public class AnchorClient : MonoBehaviour
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
    private float _currentRemainingTime;

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

    // 내부 변수
    private RenderTexture _renderTexture;
    private Texture2D _texture2D;

    private Vector3 _globalGoalPosition;
    private Vector3 _predictedWorldTarget;
    private bool _isGoalReached = false;

    // anchor history
    private readonly List<int> _anchorHistory = new List<int>();
    private int? _lastAnchorIdx = null;

    void Start()
    {
        InitializeCamera();
        InitializeRemainingTime();
        SetNewGlobalGoal();
        StartCoroutine(CoLifeCycleLoop());
    }

    void OnDestroy()
    {
        if (_renderTexture != null) _renderTexture.Release();
        if (_texture2D != null) Destroy(_texture2D);
    }

    // --- 메인 루프 ---
    IEnumerator CoLifeCycleLoop()
    {
        while (!_isGoalReached)
        {
            // 목표 도달 체크
            if (stopOnGoal)
            {
                float distToGoal = Vector3.Distance(transform.position, _globalGoalPosition);
                if (distToGoal < goalReachThreshold)
                {
                    Debug.Log("목표 도달! 오브젝트를 비활성화합니다.");
                    _isGoalReached = true;

                    if (deactivateOnGoal)
                        gameObject.SetActive(false);

                    yield break;
                }
            }

            // 데이터 준비
            byte[] imageBytes = CaptureView();
            if (imageBytes == null || imageBytes.Length == 0)
            {
                yield return new WaitForSeconds(0.1f);
                continue;
            }

            // goal_position을 curl 예시처럼 "x,z" 문자열로 보냄
            string relativeGoalStr = GetRelativeGoalPositionCsv();
            string timeStr = Mathf.Max(0f, _currentRemainingTime).ToString("F2", CultureInfo.InvariantCulture);

            // history 준비 (CSV)
            string historyStr = GetAnchorHistoryCsv();

            // 서버 요청
            string jsonResponse = null;
            yield return StartCoroutine(CoSendRequest(
                imageBytes, relativeGoalStr, timeStr, historyStr,
                (res) => { jsonResponse = res; }
            ));

            // 서버 응답 처리 및 20-step 재생
            if (!string.IsNullOrEmpty(jsonResponse))
            {
                yield return StartCoroutine(CoPlayServerTrajectory20Steps(jsonResponse));
            }
            else
            {
                yield return new WaitForSeconds(0.2f);
            }
        }
    }

    // --- 서버 응답(20 step) 전체 재생 ---
    IEnumerator CoPlayServerTrajectory20Steps(string json)
    {
        ServerResponseData data = null;
        try
        {
            data = JsonConvert.DeserializeObject<ServerResponseData>(json);
        }
        catch (Exception e)
        {
            Debug.LogError($"JSON 파싱 에러: {e.Message}\n{json}");
            yield break;
        }

        if (data == null) yield break;

        // anchor_idx 저장 -> 다음 요청 history로 전송
        if (useAnchorHistory && data.anchor_idx.HasValue)
        {
            AppendAnchorHistory(data.anchor_idx.Value);
        }

        if (data.output == null || data.output.Length == 0) yield break;

        float[][] traj = data.output[0];
        if (traj == null || traj.Length == 0) yield break;

        // 이번 배치 시작 시점: "현재 위치를 (0,0)로 둔다"
        Vector3 basePos = transform.position;
        Quaternion baseRot = transform.rotation;

        // (0,0) 포함이면 보통 0번은 (0,0)이라 스킵
        int startIndex = 0;
        if (traj[0] != null && traj[0].Length >= 2)
        {
            float x0 = flipX ? -traj[0][0] : traj[0][0];
            float z0 = flipZ ? -traj[0][1] : traj[0][1];
            if (x0 * x0 + z0 * z0 <= predEps) startIndex = 1;
        }

        int stepsToPlay = Mathf.Max(0, traj.Length - startIndex);
        if (stepsToPlay == 0) yield break;

        float stepDt;
        if (useFpsAsServerStepTime)
            stepDt = 1f / Mathf.Max(1f, fps);
        else
            stepDt = batchPlaybackSeconds / Mathf.Max(1, stepsToPlay);

        stepDt = Mathf.Max(minStepDt, stepDt);

        // 누적이 아니라 delta일 때를 위해 runningPos 준비
        Vector3 runningPos = basePos;

        for (int i = startIndex; i < traj.Length; i++)
        {
            if (_isGoalReached) yield break;

            if (stopOnGoal && Vector3.Distance(transform.position, _globalGoalPosition) < goalReachThreshold)
            {
                _isGoalReached = true;
                if (deactivateOnGoal) gameObject.SetActive(false);
                yield break;
            }

            var p = traj[i];
            if (p == null || p.Length < 2) continue;

            float dx = flipX ? -p[0] : p[0];
            float dz = flipZ ? -p[1] : p[1];

            Vector3 offset = new Vector3(dx, 0f, dz) * predScale;

            // 로컬 프레임(heading-aligned) 오프셋이면 요청 시점 회전 기준으로 월드로
            if (predIsInAgentLocalFrame)
                offset = baseRot * offset;

            Vector3 target;
            if (offsetsAreCumulative)
            {
                // 핵심: basePos + (t=0 기준 누적 오프셋)
                target = basePos + offset;
            }
            else
            {
                // step delta면 누적해서 이동
                runningPos += offset;
                target = runningPos;
            }

            target.y = transform.position.y;
            _predictedWorldTarget = target;

            // 바라보기: 다음 타겟 방향
            Vector3 dir = target - transform.position;
            dir.y = 0f;
            if (dir.sqrMagnitude > 1e-10f)
            {
                Quaternion look = Quaternion.LookRotation(dir.normalized, Vector3.up);
                look = look * Quaternion.Euler(0f, yawOffsetDeg, 0f);
                transform.rotation = look;
            }

            // 서버가 준 좌표대로 "그대로" 찍기
            transform.position = target;

            // remaining time은 실제 재생 중에만 감소
            if (_currentRemainingTime > 0f)
                _currentRemainingTime = Mathf.Max(0f, _currentRemainingTime - stepDt);

            yield return new WaitForSeconds(stepDt);
        }
    }

    // --- Goal/Time/Camera/Data helpers ---

    // (회전 무시) 현재 위치 기준 goal 상대좌표를 "x,z" CSV로 반환 (curl 예시 스타일)
    string GetRelativeGoalPositionCsv()
    {
        Vector3 offset = _globalGoalPosition - transform.position;

        float gx = flipX ? -offset.x : offset.x;
        float gz = flipZ ? -offset.z : offset.z;

        return $"{gx.ToString("F4", CultureInfo.InvariantCulture)},{gz.ToString("F4", CultureInfo.InvariantCulture)}";
    }

    void InitializeRemainingTime()
    {
        if (remainingTimeOverride > 0f)
        {
            _currentRemainingTime = remainingTimeOverride;
        }
        else
        {
            _currentRemainingTime = UnityEngine.Random.Range(remainingTimeRandomMin, remainingTimeRandomMax);
        }
    }

    void SetNewGlobalGoal()
    {
        _globalGoalPosition = (Goal != null) ? Goal.position : transform.position;
    }

    // anchor history CSV 만들기
    string GetAnchorHistoryCsv()
    {
        if (!useAnchorHistory)
            return null;

        if (_anchorHistory.Count == 0)
            return sendEmptyHistoryField ? "" : null;

        return string.Join(",", _anchorHistory);
    }

    // anchor history에 추가 + 길이 제한
    void AppendAnchorHistory(int anchorIdx)
    {
        _lastAnchorIdx = anchorIdx;
        _anchorHistory.Add(anchorIdx);

        if (maxHistoryLength > 0)
        {
            int overflow = _anchorHistory.Count - maxHistoryLength;
            if (overflow > 0)
                _anchorHistory.RemoveRange(0, overflow);
        }
    }

    IEnumerator CoSendRequest(byte[] image, string goalCsv, string time, string historyCsv, Action<string> onComplete)
    {
        WWWForm form = new WWWForm();

        // 서버가 기대하는 form-field 이름 그대로 유지
        form.AddField("goal_position", goalCsv);
        form.AddField("remaining_time", time);

        // history 추가 (null이면 아예 안 보냄)
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
        if (!Application.isPlaying) return;

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(_globalGoalPosition, 0.5f);

        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position + Vector3.up * 0.1f, _predictedWorldTarget + Vector3.up * 0.1f);
    }
}
