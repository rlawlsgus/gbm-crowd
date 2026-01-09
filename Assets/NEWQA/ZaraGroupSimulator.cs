using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;
using System.Text.RegularExpressions;
public class ZaraGroupSimulator : MonoBehaviour
{
    [Header("Input (TextAsset)")]
    public TextAsset trajectoriesTxt;   // output_zara01.txt
    public TextAsset groupsTxt;         // groups.txt
    public TextAsset homographyTxt;     // H.txt (optional)

    [Header("Playback")]
    [Min(1f)] public float fps = 25f;
    public float playbackSpeed = 1f;
    public bool loop = true;
    public bool playOnStart = true;

    [Header("World Mapping")]
    public bool useHomography = true;     // H.txt 적용
    public bool flipZ = false;            // 필요하면 true로 (좌표계 뒤집기용)
    public bool flipX = false;            // 필요하면 true로 (좌표계 뒤집기용)
    public float worldScale = 1f;
    public Vector3 worldOffset = Vector3.zero;

    [Header("Agents")]
    public GameObject agentPrefab;        // 비워두면 Sphere로 생성
    [Min(0.01f)] public float agentSize = 0.15f;
    public bool applyYawFromAngle = false;
    public float yawOffsetDeg = 0f;

    [Header("Group Gizmos")]
    public bool drawGroupRays = true;
    public bool drawOnlyWhenBothVisible = true;
    public float rayY = 0.05f;            // 바닥에서 살짝 띄우기
    public bool alsoDrawAgentGizmos = true;
    public float gizmoAgentRadius = 0.05f;

    [Header("Animation (Blend Tree)")]
    public bool driveAnimatorBlend = true;
    public string blendParamName = "Blend";   // Animator float parameter name
    public float speedForFullWalk = 1.2f;     // 이 속도에서 Blend=1
    public float blendDampTime = 0.12f;       // 부드럽게 변화(0이면 즉시)
    public float speedEpsilon = 0.01f;        // 이하면 0으로 스냅

    readonly Dictionary<int, Animator> animById = new();
    readonly Dictionary<int, Vector3> prevPosById = new();
    readonly Dictionary<int, float> prevFrameById = new();

    public enum GroupLinkMode { AllPairs, Chain }
    public GroupLinkMode linkMode = GroupLinkMode.AllPairs;

    // ======= counter

    [Header("Physics / Trigger Collision")]
    public bool enableAgentTriggers = true;
    public bool colliderIsTrigger = true;
    public float triggerRadiusWorld = 0.25f;   // 월드 기준 반경
    public bool addKinematicRigidbody = true;

    // ---------- internal ----------
    struct Sample
    {
        public int frame;
        public Vector2 xy;     // raw x,y from file (or already mapped if you want)
        public float angleDeg; // 5th column
    }

    sealed class Trajectory
    {
        public int id;
        public int[] frames;
        public Vector2[] xy;
        public float[] angles;

        public int MinFrame => frames[0];
        public int MaxFrame => frames[frames.Length - 1];

        public bool TryEvaluate(float frameF, out Vector2 pos, out float angDeg)
        {
            pos = default;
            angDeg = 0f;
            if (frames == null || frames.Length == 0) return false;
            if (frameF < frames[0] || frameF > frames[frames.Length - 1]) return false;

            // BinarySearch on int frames
            int target = Mathf.FloorToInt(frameF);
            int idx = Array.BinarySearch(frames, target);
            if (idx >= 0)
            {
                // exact integer frame present
                pos = xy[idx];
                angDeg = angles[idx];
                return true;
            }

            // insertion point
            idx = ~idx;
            int hi = Mathf.Clamp(idx, 0, frames.Length - 1);
            int lo = Mathf.Clamp(hi - 1, 0, frames.Length - 1);

            // If we got clamped to an edge, just return edge
            if (lo == hi)
            {
                pos = xy[lo];
                angDeg = angles[lo];
                return true;
            }

            float f0 = frames[lo];
            float f1 = frames[hi];
            float t = (Mathf.Approximately(f1, f0)) ? 0f : (frameF - f0) / (f1 - f0);

            pos = Vector2.LerpUnclamped(xy[lo], xy[hi], t);
            angDeg = Mathf.LerpAngle(angles[lo], angles[hi], t);
            return true;
        }
    }

    Matrix3x3 H = Matrix3x3.Identity;
    readonly Dictionary<int, Trajectory> trajById = new();
    readonly Dictionary<int, GameObject> goById = new();

    readonly List<int[]> groups = new();
    readonly List<(int a, int b, int groupIndex)> groupEdges = new();
    readonly Dictionary<int, Vector3> currentWorldPos = new();

    int globalMinFrame = int.MaxValue;
    int globalMaxFrame = int.MinValue;

    float currentFrameF = 0f;
    bool isPlaying = false;

    // ---------- Unity ----------
    void Awake()
    {
        if (playOnStart) isPlaying = true;
        LoadAll();
        SpawnAgents();
        ResetPlayback();
    }

    void Update()
    {
        if (!Application.isPlaying) return;

        if (isPlaying)
        {
            currentFrameF += Time.deltaTime * fps * playbackSpeed;

            if (loop)
            {
                float len = Mathf.Max(1f, globalMaxFrame - globalMinFrame + 1);
                while (currentFrameF > globalMaxFrame) currentFrameF -= len;
                while (currentFrameF < globalMinFrame) currentFrameF += len;
            }
            else
            {
                // clamp + "끝에 도달했을 때만" 정지
                if (currentFrameF >= globalMaxFrame)
                {
                    currentFrameF = globalMaxFrame;
                    isPlaying = false;
                }
                else if (currentFrameF < globalMinFrame)
                {
                    currentFrameF = globalMinFrame;
                }
            }
        }

        EvaluateAndApplyTransforms(); // alive 받아도 되고 안 받아도 됨
    }

    void OnDrawGizmos()
    {
        if (!drawGroupRays) return;
        if (!Application.isPlaying) return; // 단순화: 플레이 중만 그림

        // Rays between group members
        foreach (var e in groupEdges)
        {
            if (drawOnlyWhenBothVisible)
            {
                if (!currentWorldPos.TryGetValue(e.a, out var pa)) continue;
                if (!currentWorldPos.TryGetValue(e.b, out var pb)) continue;

                pa.y = rayY;
                pb.y = rayY;

                Gizmos.color = GroupColor(e.groupIndex);
                Gizmos.DrawRay(pa, pb - pa);
            }
            else
            {
                // if you want even when missing, you could draw from last known etc.
            }
        }

        if (alsoDrawAgentGizmos)
        {
            Gizmos.color = Color.white;
            foreach (var kv in currentWorldPos)
            {
                var p = kv.Value;
                p.y = rayY;
                Gizmos.DrawSphere(p, gizmoAgentRadius);
            }
        }
    }

    // ---------- public controls ----------
    public void Play() => isPlaying = true;
    public void Pause() => isPlaying = false;
    public void ResetPlayback()
    {
        currentFrameF = globalMinFrame;
        EvaluateAndApplyTransforms();
    }

    // ---------- core ----------
    void LoadAll()
    {
        trajById.Clear();
        groups.Clear();
        groupEdges.Clear();
        globalMinFrame = int.MaxValue;
        globalMaxFrame = int.MinValue;

        if (homographyTxt != null)
            H = ParseHomography3x3(homographyTxt.text);

        if (trajectoriesTxt == null)
        {
            Debug.LogError("[ZaraGroupSimulator] trajectoriesTxt is null.");
            return;
        }

        ParseTrajectories(trajectoriesTxt.text);
        if (groupsTxt != null) ParseGroups(groupsTxt.text);
        BuildGroupEdges();
    }

    void ParseTrajectories(string text)
    {
        var inv = CultureInfo.InvariantCulture;

        // temp: id -> List<Sample>
        var tmp = new Dictionary<int, List<Sample>>(256);

        using var sr = new StringReader(text);
        string line;
        int lineNo = 0;

        while ((line = sr.ReadLine()) != null)
        {
            lineNo++;
            if (string.IsNullOrWhiteSpace(line)) continue;

            // expected: id, x, y, frame, angle
            var parts = line.Split(',');
            if (parts.Length < 5)
            {
                Debug.LogWarning($"[ZaraGroupSimulator] Skip bad line {lineNo}: {line}");
                continue;
            }

            try
            {
                int id = int.Parse(parts[0].Trim(), inv);
                float x = float.Parse(parts[1].Trim(), inv);
                float y = float.Parse(parts[2].Trim(), inv);
                int frame = int.Parse(parts[3].Trim(), inv);
                float ang = float.Parse(parts[4].Trim(), inv);

                if (!tmp.TryGetValue(id, out var list))
                {
                    list = new List<Sample>(256);
                    tmp[id] = list;
                }
                list.Add(new Sample { frame = frame, xy = new Vector2(x, y), angleDeg = ang });

                globalMinFrame = Mathf.Min(globalMinFrame, frame);
                globalMaxFrame = Mathf.Max(globalMaxFrame, frame);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[ZaraGroupSimulator] Parse error line {lineNo}: {line}\n{ex.Message}");
            }
        }

        // finalize
        foreach (var kv in tmp)
        {
            var id = kv.Key;
            var list = kv.Value;
            list.Sort((a, b) => a.frame.CompareTo(b.frame));

            int n = list.Count;
            var frames = new int[n];
            var xy = new Vector2[n];
            var ang = new float[n];

            for (int i = 0; i < n; i++)
            {
                frames[i] = list[i].frame;
                xy[i] = list[i].xy;
                ang[i] = list[i].angleDeg;
            }

            trajById[id] = new Trajectory
            {
                id = id,
                frames = frames,
                xy = xy,
                angles = ang
            };
        }

        if (globalMinFrame == int.MaxValue) globalMinFrame = 0;
        if (globalMaxFrame == int.MinValue) globalMaxFrame = 0;
    }

    void ParseGroups(string text)
    {
        using var sr = new StringReader(text);
        string line;
        int lineNo = 0;

        while ((line = sr.ReadLine()) != null)
        {
            lineNo++;
            line = line.Trim();
            if (line.Length == 0) continue;

            var toks = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            if (toks.Length < 2) continue;

            var ids = new int[toks.Length];
            bool ok = true;
            for (int i = 0; i < toks.Length; i++)
            {
                if (!int.TryParse(toks[i], NumberStyles.Integer, CultureInfo.InvariantCulture, out ids[i]))
                {
                    ok = false;
                    break;
                }
            }
            if (!ok)
            {
                Debug.LogWarning($"[ZaraGroupSimulator] Bad group line {lineNo}: {line}");
                continue;
            }
            groups.Add(ids);
        }
    }

    void BuildGroupEdges()
    {
        groupEdges.Clear();
        for (int gi = 0; gi < groups.Count; gi++)
        {
            var g = groups[gi];
            if (g.Length < 2) continue;

            if (linkMode == GroupLinkMode.Chain)
            {
                for (int i = 0; i < g.Length - 1; i++)
                    groupEdges.Add((g[i], g[i + 1], gi));
            }
            else // AllPairs
            {
                for (int i = 0; i < g.Length; i++)
                    for (int j = i + 1; j < g.Length; j++)
                        groupEdges.Add((g[i], g[j], gi));
            }
        }
    }

    void SpawnAgents()
    {
        // cleanup old
        animById.Clear();
        prevPosById.Clear();
        prevFrameById.Clear();
        foreach (var kv in goById)
            if (kv.Value != null) Destroy(kv.Value);
        goById.Clear();

        foreach (var kv in trajById)
        {
            int id = kv.Key;

            GameObject go;
            if (agentPrefab != null)
            {
                go = Instantiate(agentPrefab, transform);
            }
            else
            {
                go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                go.transform.SetParent(transform, worldPositionStays: true);
                var col = go.GetComponent<Collider>();
                if (col != null) Destroy(col); // 물리 필요 없으면 제거
            }

            go.name = $"ped_{id}";
            go.transform.localScale = Vector3.one * agentSize;
            goById[id] = go;

            var anim = go.GetComponentInChildren<Animator>();
            if (anim != null) animById[id] = anim;

        }
    }

    //int EvaluateAndApplyTransforms()
    //{
    //    currentWorldPos.Clear();
    //    int alive = 0;
    //    foreach (var kv in trajById)
    //    {
    //        int id = kv.Key;
    //        var traj = kv.Value;

    //        if (!traj.TryEvaluate(currentFrameF, out var rawXY, out var angDeg))
    //        {
    //            if (goById.TryGetValue(id, out var goMissing) && goMissing != null)
    //                goMissing.SetActive(false);

    //            prevPosById.Remove(id);
    //            prevFrameById.Remove(id);

    //            continue;
    //        }
    //        alive++;
    //        if (goById.TryGetValue(id, out var go) && go != null)
    //        {
    //            go.SetActive(true);

    //            Vector2 world2 = useHomography ? ApplyHomography(H, rawXY) : rawXY;
    //            var pos = ToUnityXZ(world2) * worldScale + worldOffset;

    //            go.transform.position = pos;

    //            // speed -> Blend(0..1)
    //            if (driveAnimatorBlend && animById.TryGetValue(id, out var anim) && anim != null)
    //            {
    //                float speed = 0f;

    //                if (prevPosById.TryGetValue(id, out var prevPos) &&
    //                    prevFrameById.TryGetValue(id, out var prevF))
    //                {
    //                    float dt = Mathf.Max(1e-6f, Time.deltaTime);
    //                    speed = Vector3.Distance(prevPos, pos) / dt;

    //                }

    //                prevPosById[id] = pos;
    //                prevFrameById[id] = currentFrameF;

    //                float target = (speedForFullWalk <= 1e-6f) ? 0f : Mathf.Clamp01(speed / speedForFullWalk);
    //                if (speed < speedEpsilon) target = 0f;

    //                if (blendDampTime > 0f) anim.SetFloat(blendParamName, target, blendDampTime, Time.deltaTime);
    //                else anim.SetFloat(blendParamName, target);
    //            }

    //            if (applyYawFromAngle)
    //            {
    //                float yaw = angDeg;

    //                // 좌표 미러링에 맞춰 각도도 미러링
    //                if (flipX) yaw = 180f - yaw;  // (dx,dy)->(-dx,dy)
    //                if (flipZ) yaw = -yaw;        // (dx,dy)->(dx,-dy)

    //                yaw += yawOffsetDeg;

    //                go.transform.rotation = Quaternion.Euler(0f, yaw, 0f);
    //            }


    //            currentWorldPos[id] = pos;
    //        }
    //    }

    //    return alive;
    //}


    int EvaluateAndApplyTransforms()
    {
        currentWorldPos.Clear();
        int alive = 0;
        foreach (var kv in trajById)
        {
            int id = kv.Key;
            var traj = kv.Value;

            // 1. 현재 프레임의 위치 계산
            if (!traj.TryEvaluate(currentFrameF, out var rawXY, out var angDeg))
            {
                if (goById.TryGetValue(id, out var goMissing) && goMissing != null)
                    goMissing.SetActive(false);

                prevPosById.Remove(id);
                prevFrameById.Remove(id);

                continue;
            }
            alive++;
            if (goById.TryGetValue(id, out var go) && go != null)
            {
                go.SetActive(true);

                // 2. 월드 좌표 변환
                Vector2 world2 = useHomography ? ApplyHomography(H, rawXY) : rawXY;
                var pos = ToUnityXZ(world2) * worldScale + worldOffset;

                // [중요] 이전 위치 가져오기 (이동 방향 계산용)
                Vector3 oldPos = pos;
                bool hasPrev = prevPosById.TryGetValue(id, out oldPos);

                // 3. 위치 적용
                go.transform.position = pos;

                // 4. 애니메이션 속도 계산 (옵션)
                if (driveAnimatorBlend && animById.TryGetValue(id, out var anim) && anim != null)
                {
                    float speed = 0f;
                    if (hasPrev && prevFrameById.TryGetValue(id, out var prevF))
                    {
                        float dt = Mathf.Max(1e-6f, Time.deltaTime);
                        speed = Vector3.Distance(oldPos, pos) / dt;
                    }

                    float target = (speedForFullWalk <= 1e-6f) ? 0f : Mathf.Clamp01(speed / speedForFullWalk);
                    if (speed < speedEpsilon) target = 0f;

                    if (blendDampTime > 0f) anim.SetFloat(blendParamName, target, blendDampTime, Time.deltaTime);
                    else anim.SetFloat(blendParamName, target);
                }

                // 5. [수정된 부분] 회전 처리: 걷는 방향 바라보기
                if (applyYawFromAngle)
                {
                    if (hasPrev) // 이전 위치가 있어야 방향을 알 수 있음
                    {
                        Vector3 dir = pos - oldPos;
                        dir.y = 0; // 높이 차이는 무시

                        // 아주 조금이라도 움직였을 때만 회전 (제자리 떨림 방지)
                        if (dir.sqrMagnitude > 0.00001f)
                        {
                            Quaternion lookRot = Quaternion.LookRotation(dir);
                            // 모델의 기본 방향 보정 (yawOffsetDeg) 적용
                            go.transform.rotation = lookRot * Quaternion.Euler(0, yawOffsetDeg, 0);
                        }
                    }
                }

                // [데이터 갱신] 현재 위치를 '이전 위치'로 저장
                prevPosById[id] = pos;
                prevFrameById[id] = currentFrameF;

                currentWorldPos[id] = pos;
            }
        }

        return alive;
    }
    Vector3 ToUnityXZ(Vector2 xy)
    {
        float x = flipX ? -xy.x : xy.x;
        float z = flipZ ? -xy.y : xy.y;
        return new Vector3(x, 0f, z);
    }



    static Color GroupColor(int groupIndex)
    {
        // group마다 색이 다르게 보이도록 (HSV)
        float h = Mathf.Repeat(groupIndex * 0.137f, 1f);
        return Color.HSVToRGB(h, 0.85f, 1f);
    }

    // ---------- homography helpers ----------
    static Vector2 ApplyHomography(Matrix3x3 h, Vector2 uv)
    {
        // [x;y;w] = H*[u;v;1]
        float u = uv.x, v = uv.y;

        float x = h.m00 * u + h.m01 * v + h.m02;
        float y = h.m10 * u + h.m11 * v + h.m12;
        float w = h.m20 * u + h.m21 * v + h.m22;

        if (!Mathf.Approximately(w, 0f))
        {
            x /= w;
            y /= w;
        }
        return new Vector2(x, y);
    }

    static Matrix3x3 ParseHomography3x3(string text)
    {
        var inv = CultureInfo.InvariantCulture;
        var rows = new List<float[]>(3);

        using var sr = new StringReader(text);
        string line;
        while ((line = sr.ReadLine()) != null)
        {
            line = line.Trim();
            if (line.Length == 0) continue;

            var toks = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            if (toks.Length < 3) continue;

            rows.Add(new[]
            {
                float.Parse(toks[0], inv),
                float.Parse(toks[1], inv),
                float.Parse(toks[2], inv),
            });

            if (rows.Count == 3) break;
        }

        if (rows.Count < 3)
        {
            Debug.LogWarning("[ZaraGroupSimulator] Homography parse failed. Using Identity.");
            return Matrix3x3.Identity;
        }

        return new Matrix3x3(
            rows[0][0], rows[0][1], rows[0][2],
            rows[1][0], rows[1][1], rows[1][2],
            rows[2][0], rows[2][1], rows[2][2]
        );
    }

    [Serializable]
    struct Matrix3x3
    {
        public float m00, m01, m02;
        public float m10, m11, m12;
        public float m20, m21, m22;

        public static Matrix3x3 Identity => new Matrix3x3(
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        );

        public Matrix3x3(
            float m00, float m01, float m02,
            float m10, float m11, float m12,
            float m20, float m21, float m22)
        {
            this.m00 = m00; this.m01 = m01; this.m02 = m02;
            this.m10 = m10; this.m11 = m11; this.m12 = m12;
            this.m20 = m20; this.m21 = m21; this.m22 = m22;
        }
    }



    public float CurrentFrameF => currentFrameF;                 // float frame (보간 포함)
    public int GlobalFrame => Mathf.FloorToInt(currentFrameF);    // int frame
    public int MinFrame => globalMinFrame;
    public int MaxFrame => globalMaxFrame;

}
