// SunCycle.cs
using UnityEngine;

[ExecuteAlways]
public class SunCycle : MonoBehaviour
{
    [Header("Sun (Directional Light)")]
    public Light sun;                     // 비우면 이 오브젝트에서 자동 탐색
    public Vector3 rotationAxis = Vector3.right; // 보통 X축(동->서)

    [Header("Time")]
    [Min(0.1f)] public float dayLengthSeconds = 60f; // 1일(한 바퀴) 걸리는 시간
    [Range(0f, 1f)] public float startTime01 = 0.25f; // 시작 시각(0~1)
    public bool useUnscaledTime = false;   // 타임스케일 무시

    [Header("Rotation")]
    public float yawDeg = 0f;              // 북쪽 방향 맞추기(상하 회전축 외 추가 요 회전)
    public float pitchOffsetDeg = -90f;    // 정오 방향/지평선 맞추기용

    [Header("Color / Intensity")]
    public Gradient colorOverDay;
    public AnimationCurve intensityOverDay = AnimationCurve.EaseInOut(0, 0, 1, 1);
    [Min(0f)] public float maxIntensity = 1.2f;

    [Header("Optional: Ambient")]
    public bool driveAmbient = true;
    public Gradient ambientOverDay;

    void Reset()
    {
        sun = GetComponent<Light>();
        if (sun == null) sun = FindAnyObjectByType<Light>();

        // 기본 컬러 그라데이션 (밤->해뜰->정오->해질->밤)
        colorOverDay = new Gradient();
        colorOverDay.SetKeys(
            new[]
            {
                new GradientColorKey(new Color(0.25f,0.35f,0.6f), 0.00f),
                new GradientColorKey(new Color(1.0f,0.55f,0.25f), 0.23f),
                new GradientColorKey(new Color(1.0f,0.98f,0.90f), 0.50f),
                new GradientColorKey(new Color(1.0f,0.45f,0.20f), 0.77f),
                new GradientColorKey(new Color(0.25f,0.35f,0.6f), 1.00f),
            },
            new[]
            {
                new GradientAlphaKey(1f, 0f),
                new GradientAlphaKey(1f, 1f),
            }
        );

        ambientOverDay = new Gradient();
        ambientOverDay.SetKeys(
            new[]
            {
                new GradientColorKey(new Color(0.02f,0.03f,0.06f), 0.00f),
                new GradientColorKey(new Color(0.20f,0.18f,0.15f), 0.25f),
                new GradientColorKey(new Color(0.55f,0.55f,0.55f), 0.50f),
                new GradientColorKey(new Color(0.18f,0.14f,0.12f), 0.75f),
                new GradientColorKey(new Color(0.02f,0.03f,0.06f), 1.00f),
            },
            new[]
            {
                new GradientAlphaKey(1f, 0f),
                new GradientAlphaKey(1f, 1f),
            }
        );
    }

    void OnEnable()
    {
        if (sun == null) sun = GetComponent<Light>();
    }

    void Update()
    {
        if (sun == null) return;

        float dt = useUnscaledTime ? Time.unscaledDeltaTime : Time.deltaTime;

        // ExecuteAlways에서 에디터 정지 상태면 deltaTime이 0일 수 있어서 보정
        if (!Application.isPlaying && dt <= 0f) dt = 1f / 60f;

        float t = (float)(TimeFromNow(dt) / Mathf.Max(0.1f, dayLengthSeconds) + startTime01) % 1f;

        // 회전: 하루에 360도
        float angle = t * 360f + pitchOffsetDeg;

        Quaternion baseRot = Quaternion.AngleAxis(yawDeg, Vector3.up);
        Quaternion dayRot = Quaternion.AngleAxis(angle, rotationAxis.normalized);

        transform.rotation = baseRot * dayRot;

        // 색/밝기
        sun.color = colorOverDay.Evaluate(t);
        sun.intensity = maxIntensity * Mathf.Clamp01(intensityOverDay.Evaluate(t));

        // 앰비언트(선택)
        if (driveAmbient)
        {
            RenderSettings.ambientLight = ambientOverDay.Evaluate(t);
        }
    }

    double _accum;
    double TimeFromNow(float dt)
    {
        // 플레이/에디터 모두 안정적으로 누적되게
        _accum += dt;
        return _accum;
    }
}
