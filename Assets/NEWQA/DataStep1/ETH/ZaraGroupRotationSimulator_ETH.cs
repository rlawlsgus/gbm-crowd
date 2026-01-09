// ZaraGroupRotationSimulator_ETH.cs
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class ZaraGroupRotationSimulator_ETH : MonoBehaviour
{
    [Header("Input (TextAsset)")]
    public TextAsset trajectoriesTxt;   // Zara: id,x,y,frame,angle  OR  ETH: obsmat.txt
    public TextAsset groupsTxt;         // groups.txt
    public TextAsset homographyTxt;     // H.txt (optional)

    public enum TrajectoryInputFormat { Auto, ZaraCsv5, EthObsmat8 }

    [Header("Input Format")]
    public TrajectoryInputFormat inputFormat = TrajectoryInputFormat.Auto;

    [Tooltip("ETH obsmat.txt는 보통 pos/vel이 이미 meters(world) 좌표라서 homography를 다시 적용하면 안 됩니다.")]
    public bool ethPositionsAreWorldMeters = true;

    [Tooltip("ETH(obsmat)에서 v_x, v_y로 angle(heading)을 계산해 angles에 저장합니다.")]
    public bool ethComputeAngleFromVelocity = true;

    [Header("Playback")]
    [Min(1f)] public float fps = 25f;
    public float playbackSpeed = 1f;
    public bool loop = true;
    public bool playOnStart = true;

    [Header("World Mapping")]
    public bool useHomography = true;     // H.txt 적용 (Zara pixel->world 등)
    public bool flipZ = false;            // 필요하면 true로 (좌표계 뒤집기용)
    public bool flipX = false;            // 필요하면 true로 (좌표계 뒤집기용)
    public float worldScale = 1f;
    public Vector3 worldOffset = Vector3.zero;

    [Header("Augmentation / Rotation (WORLD)")]
    public bool rotateInWorld = false;
    public float rotateDeg = 0f;

    public enum RotationPivotMode { WorldOrigin, WorldOffset, DataCentroid, Custom }
    public RotationPivotMode pivotMode = RotationPivotMode.DataCentroid;
    public Vector3 customPivotWorld = Vector3.zero;

    [Header("Agents")]
    public GameObject agentPrefab;        // 비워두면 Sphere로 생성
    [Min(0.01f)] public float agentSize = 0.15f;
    public bool applyYawFromAngle = false; // (현재 구현은 "이동 방향을 바라보기" 토글로 사용)
    public float yawOffsetDeg = 0f;

    [Header("Yaw Smoothing (Move Direction -> Yaw)")]
    public bool smoothYaw = true;
    [Min(0.001f)] public float yawSmoothTime = 0.08f; // 작을수록 빨리 따라감
    [Min(0f)] public float dirEpsilon = 0.0005f;      // 너무 작으면 방향 업데이트 안 함(마지막 유효 dir 유지)

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

    readonly Dictionary<int, Animator> animById = new Dictionary<int, Animator>();
    readonly Dictionary<int, Vector3> prevPosById = new Dictionary<int, Vector3>();
    readonly Dictionary<int, float> prevFrameById = new Dictionary<int, float>();

    // yaw smoothing state
    readonly Dictionary<int, float> yawVelById = new Dictionary<int, float>();
    readonly Dictionary<int, Vector3> lastDirById = new Dictionary<int, Vector3>();

    public enum GroupLinkMode { AllPairs, Chain }
    public GroupLinkMode linkMode = GroupLinkMode.AllPairs;

    [Header("Physics / Trigger Collision")]
    public bool enableAgentTriggers = true;
    public bool colliderIsTrigger = true;
    public float triggerRadiusWorld = 0.25f;   // 월드 기준 반경
    public bool addKinematicRigidbody = true;

    // ---------- internal ----------
    struct Sample
    {
        public int frame;
        public Vector2 xy;         // raw ground (x,y) from file (Zara: x,y / ETH: pos_x,pos_y)
        public float angleDeg;     // Zara: 5th column / ETH: derived or 0
        public Vector2 velXY;      // ETH: (v_x, v_y) on ground
        public bool hasVel;
    }

    sealed class Trajectory
    {
        public int id;
        public int[] frames;
        public Vector2[] xy;
        public float[] angles;

        public Vector2[] velXY;    // optional
        public bool[] hasVel;

        public int MinFrame => frames[0];
        public int MaxFrame => frames[frames.Length - 1];

        public bool TryEvaluate(float frameF, out Vector2 pos, out float angDeg, out Vector2 velOut, out bool hasVelOut)
        {
            pos = default;
            angDeg = 0f;
            velOut = default;
            hasVelOut = false;

            if (frames == null || frames.Length == 0) return false;
            if (frameF < frames[0] || frameF > frames[frames.Length - 1]) return false;

            int target = Mathf.FloorToInt(frameF);
            int idx = Array.BinarySearch(frames, target);
            if (idx >= 0)
            {
                pos = xy[idx];
                angDeg = angles[idx];

                if (velXY != null && hasVel != null && idx < velXY.Length && idx < hasVel.Length)
                {
                    velOut = velXY[idx];
                    hasVelOut = hasVel[idx];
                }
                return true;
            }

            idx = ~idx;
            int hi = Mathf.Clamp(idx, 0, frames.Length - 1);
            int lo = Mathf.Clamp(hi - 1, 0, frames.Length - 1);

            if (lo == hi)
            {
                pos = xy[lo];
                angDeg = angles[lo];

                if (velXY != null && hasVel != null && lo < velXY.Length && lo < hasVel.Length)
                {
                    velOut = velXY[lo];
                    hasVelOut = hasVel[lo];
                }
                return true;
            }

            float f0 = frames[lo];
            float f1 = frames[hi];
            float t = (Mathf.Approximately(f1, f0)) ? 0f : (frameF - f0) / (f1 - f0);

            pos = Vector2.LerpUnclamped(xy[lo], xy[hi], t);
            angDeg = Mathf.LerpAngle(angles[lo], angles[hi], t);

            if (velXY != null && hasVel != null && lo < velXY.Length && hi < velXY.Length)
            {
                velOut = Vector2.LerpUnclamped(velXY[lo], velXY[hi], t);
                hasVelOut = (lo < hasVel.Length && hi < hasVel.Length) ? (hasVel[lo] || hasVel[hi]) : true;
            }

            return true;
        }
    }

    Matrix3x3 H = Matrix3x3.Identity;
    readonly Dictionary<int, Trajectory> trajById = new Dictionary<int, Trajectory>();
    readonly Dictionary<int, GameObject> goById = new Dictionary<int, GameObject>();

    readonly List<int[]> groups = new List<int[]>();
    readonly List<(int a, int b, int groupIndex)> groupEdges = new List<(int a, int b, int groupIndex)>();
    readonly Dictionary<int, Vector3> currentWorldPos = new Dictionary<int, Vector3>();

    int globalMinFrame = int.MaxValue;
    int globalMaxFrame = int.MinValue;

    float currentFrameF = 0f;
    bool isPlaying = false;

    Vector3 dataCentroidWorld = Vector3.zero;

    TrajectoryInputFormat formatInUse = TrajectoryInputFormat.ZaraCsv5;

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

        float before = currentFrameF;

        if (isPlaying)
        {
            currentFrameF += Time.deltaTime * fps * playbackSpeed;

            if (loop)
            {
                float len = Mathf.Max(1f, globalMaxFrame - globalMinFrame + 1);

                while (currentFrameF > globalMaxFrame) currentFrameF -= len;
                while (currentFrameF < globalMinFrame) currentFrameF += len;

                bool wrapped =
                    (playbackSpeed >= 0f && currentFrameF < before) ||
                    (playbackSpeed < 0f && currentFrameF > before);

                if (wrapped) ClearHistory();
            }
            else
            {
                if (currentFrameF >= globalMaxFrame)
                {
                    currentFrameF = globalMaxFrame;
                    StopPlaybackAndExitPlayMode();
                }
                else if (currentFrameF < globalMinFrame)
                {
                    currentFrameF = globalMinFrame;
                }
            }
        }

        EvaluateAndApplyTransforms();
    }

    void OnDrawGizmos()
    {
        bool wantTraj = drawAllTrajectories;
        bool wantGroup = drawGroupRays;

        if (!wantTraj && !wantGroup) return;

        if (!Application.isPlaying && wantTraj && !drawTrajectoriesInEditMode) return;

        if (!Application.isPlaying && wantTraj)
        {
            if (gizmoNeedsReload || trajById.Count == 0)
            {
                LoadAll();
                gizmoNeedsReload = false;
            }
        }

        if (wantTraj && trajById.Count > 0)
            DrawAllTrajectoriesGizmos();

        if (!Application.isPlaying) return;

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

    void DrawAllTrajectoriesGizmos()
    {
        int stride = Mathf.Max(1, trajStride);
        Vector3 pivot = GetPivotWorld();

        foreach (var kv in trajById)
        {
            int id = kv.Key;
            var t = kv.Value;
            if (t.frames == null || t.frames.Length < 2) continue;

            // ✅ NEW: Play 모드에서만 "활성 agent"만 trajectory 표시
            if (drawTrajectoriesOnlyForActiveAgents && Application.isPlaying)
            {
                if (!goById.TryGetValue(id, out var go) || go == null) continue;
                if (!go.activeInHierarchy) continue;
            }

            if (trajectoryUseGroupColors)
            {
                int gi = FindGroupIndexOfAgent(id);
                Gizmos.color = (gi >= 0) ? GroupColor(gi) : AgentColor(id);
            }
            else
            {
                Gizmos.color = AgentColor(id);
            }

            bool hasPrev = false;
            Vector3 prev = default;

            for (int i = 0; i < t.frames.Length; i += stride)
            {
                Vector3 p = MapRawToWorld(t.xy[i], pivot);
                p.y = trajY;

                if (hasPrev)
                    Gizmos.DrawLine(prev, p);

                if (drawTrajPoints)
                    Gizmos.DrawSphere(p, trajPointRadius);

                prev = p;
                hasPrev = true;
            }
        }
    }

    static Color AgentColor(int id)
    {
        float h = Mathf.Repeat(id * 0.1031f, 1f);
        return Color.HSVToRGB(h, 0.85f, 1f);
    }

    int FindGroupIndexOfAgent(int agentId)
    {
        for (int gi = 0; gi < groups.Count; gi++)
        {
            var g = groups[gi];
            for (int i = 0; i < g.Length; i++)
                if (g[i] == agentId) return gi;
        }
        return -1;
    }

    // ---------- public controls ----------
    public void Play() => isPlaying = true;
    public void Pause() => isPlaying = false;

    public void ResetPlayback()
    {
        currentFrameF = globalMinFrame;
        ClearHistory();
        EvaluateAndApplyTransforms();
    }

    void ClearHistory()
    {
        prevPosById.Clear();
        prevFrameById.Clear();
        yawVelById.Clear();
        lastDirById.Clear();
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
            Debug.LogError("[ZaraGroupRotationSimulator_ETH] trajectoriesTxt is null.");
            return;
        }

        ParseTrajectories(trajectoriesTxt.text);

        if (groupsTxt != null) ParseGroups(groupsTxt.text);
        BuildGroupEdges();

        ComputeDataCentroidWorld();
    }

    TrajectoryInputFormat DetectFormat(string text)
    {
        if (inputFormat != TrajectoryInputFormat.Auto) return inputFormat;

        using (var sr = new StringReader(text))
        {
            string line;
            while ((line = sr.ReadLine()) != null)
            {
                line = line.Trim();
                if (line.Length == 0) continue;

                // Zara original: comma separated
                if (line.Contains(",")) return TrajectoryInputFormat.ZaraCsv5;

                // ETH obsmat: whitespace separated, 8+ columns
                var toks = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                if (toks.Length >= 8) return TrajectoryInputFormat.EthObsmat8;
                if (toks.Length >= 5) return TrajectoryInputFormat.ZaraCsv5; // fallback
            }
        }
        return TrajectoryInputFormat.ZaraCsv5;
    }

    static int ParseIntFromFloatToken(string tok, CultureInfo inv)
    {
        // tokens like "8.2050000e+03"
        if (float.TryParse(tok, NumberStyles.Float, inv, out var f))
            return Mathf.RoundToInt(f);

        if (int.TryParse(tok, NumberStyles.Integer, inv, out var i))
            return i;

        return 0;
    }

    void ParseTrajectories(string text)
    {
        var inv = CultureInfo.InvariantCulture;

        formatInUse = DetectFormat(text);

        var tmp = new Dictionary<int, List<Sample>>(256);

        using (var sr = new StringReader(text))
        {
            string line;
            int lineNo = 0;

            while ((line = sr.ReadLine()) != null)
            {
                lineNo++;
                if (string.IsNullOrWhiteSpace(line)) continue;

                if (formatInUse == TrajectoryInputFormat.ZaraCsv5)
                {
                    var parts = line.Split(',');
                    if (parts.Length < 5) continue;

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

                        list.Add(new Sample
                        {
                            frame = frame,
                            xy = new Vector2(x, y),
                            angleDeg = ang,
                            velXY = Vector2.zero,
                            hasVel = false
                        });

                        globalMinFrame = Mathf.Min(globalMinFrame, frame);
                        globalMaxFrame = Mathf.Max(globalMaxFrame, frame);
                    }
                    catch (Exception ex)
                    {
                        Debug.LogWarning($"[ZaraGroupRotationSimulator_ETH] Parse error line {lineNo}: {line}\n{ex.Message}");
                    }
                }
                else // ETH obsmat
                {
                    // [frame_number pedestrian_ID pos_x pos_z pos_y v_x v_z v_y]
                    var toks = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                    if (toks.Length < 8) continue; // skip stray lines

                    try
                    {
                        int frame = ParseIntFromFloatToken(toks[0], inv);
                        int id = ParseIntFromFloatToken(toks[1], inv);

                        float posX = float.Parse(toks[2], NumberStyles.Float, inv);
                        float posY = float.Parse(toks[4], NumberStyles.Float, inv);

                        float vx = float.Parse(toks[5], NumberStyles.Float, inv);
                        float vy = float.Parse(toks[7], NumberStyles.Float, inv);

                        var vel = new Vector2(vx, vy);
                        float ang = 0f;
                        if (ethComputeAngleFromVelocity && vel.sqrMagnitude > 1e-10f)
                            ang = Mathf.Atan2(vel.x, vel.y) * Mathf.Rad2Deg;

                        if (!tmp.TryGetValue(id, out var list))
                        {
                            list = new List<Sample>(256);
                            tmp[id] = list;
                        }

                        list.Add(new Sample
                        {
                            frame = frame,
                            xy = new Vector2(posX, posY),
                            angleDeg = ang,
                            velXY = vel,
                            hasVel = true
                        });

                        globalMinFrame = Mathf.Min(globalMinFrame, frame);
                        globalMaxFrame = Mathf.Max(globalMaxFrame, frame);
                    }
                    catch (Exception ex)
                    {
                        Debug.LogWarning($"[ZaraGroupRotationSimulator_ETH] Parse error line {lineNo}: {line}\n{ex.Message}");
                    }
                }
            }
        }

        // build trajectories
        foreach (var kv in tmp)
        {
            int id = kv.Key;
            var list = kv.Value;
            list.Sort((a, b) => a.frame.CompareTo(b.frame));

            // compact duplicate frames
            var compact = new List<Sample>(list.Count);
            for (int i = 0; i < list.Count; i++)
            {
                var s = list[i];
                if (compact.Count == 0 || compact[compact.Count - 1].frame != s.frame)
                {
                    compact.Add(s);
                }
                else
                {
                    var prev = compact[compact.Count - 1];
                    float pv = prev.velXY.sqrMagnitude;
                    float sv = s.velXY.sqrMagnitude;
                    if (sv > pv) compact[compact.Count - 1] = s;
                    else compact[compact.Count - 1] = prev;
                }
            }

            int n = compact.Count;
            var frames = new int[n];
            var xy = new Vector2[n];
            var ang = new float[n];
            var vel = new Vector2[n];
            var hasVel = new bool[n];

            for (int i = 0; i < n; i++)
            {
                frames[i] = compact[i].frame;
                xy[i] = compact[i].xy;
                ang[i] = compact[i].angleDeg;
                vel[i] = compact[i].velXY;
                hasVel[i] = compact[i].hasVel;
            }

            trajById[id] = new Trajectory
            {
                id = id,
                frames = frames,
                xy = xy,
                angles = ang,
                velXY = vel,
                hasVel = hasVel
            };
        }

        if (globalMinFrame == int.MaxValue) globalMinFrame = 0;
        if (globalMaxFrame == int.MinValue) globalMaxFrame = 0;
    }

    void ParseGroups(string text)
    {
        using (var sr = new StringReader(text))
        {
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
                    Debug.LogWarning($"[ZaraGroupRotationSimulator_ETH] Bad group line {lineNo}: {line}");
                    continue;
                }

                groups.Add(ids);
            }
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
            else
            {
                for (int i = 0; i < g.Length; i++)
                    for (int j = i + 1; j < g.Length; j++)
                        groupEdges.Add((g[i], g[j], gi));
            }
        }
    }

    void SpawnAgents()
    {
        animById.Clear();
        prevPosById.Clear();
        prevFrameById.Clear();
        yawVelById.Clear();
        lastDirById.Clear();

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
                if (col != null) Destroy(col);
            }

            go.name = $"ped_{id}";
            go.transform.localScale = new Vector3(agentSize, go.transform.localScale.y, go.transform.localScale.z);

            int start = 0;
            int count = 6;
            int endExclusive = Mathf.Min(go.transform.childCount, start + count);

            for (int i = start; i < endExclusive; i++)
                go.transform.GetChild(i).gameObject.SetActive(false);

            if (endExclusive > start)
            {
                int idx = UnityEngine.Random.Range(start, endExclusive);
                go.transform.GetChild(idx).gameObject.SetActive(true);
            }

            if (enableAgentTriggers)
            {
                var sc = go.GetComponent<SphereCollider>();
                if (sc == null) sc = go.AddComponent<SphereCollider>();
                sc.isTrigger = colliderIsTrigger;

                float scale = Mathf.Max(1e-6f, go.transform.lossyScale.x);
                sc.radius = triggerRadiusWorld / scale;

                if (addKinematicRigidbody)
                {
                    var rb = go.GetComponent<Rigidbody>();
                    if (rb == null) rb = go.AddComponent<Rigidbody>();
                    rb.isKinematic = true;
                    rb.useGravity = false;
                }
            }

            go.SetActive(false);

            goById[id] = go;

            var anim = go.GetComponentInChildren<Animator>();
            if (anim != null) animById[id] = anim;
        }
    }

    bool IsETHWorld()
    {
        return formatInUse == TrajectoryInputFormat.EthObsmat8 && ethPositionsAreWorldMeters;
    }

    bool ShouldApplyHomography()
    {
        if (IsETHWorld()) return false;
        return useHomography;
    }

    Vector2 MapRawTo2D(Vector2 rawXY)
    {
        return ShouldApplyHomography() ? ApplyHomography(H, rawXY) : rawXY;
    }

    Vector3 Map2DToUnityWorld(Vector2 mappedXY)
    {
        return ToUnityXZ(mappedXY) * worldScale + worldOffset;
    }

    Vector3 MapRawToWorld(Vector2 rawXY, Vector3 pivot)
    {
        Vector2 mapped2 = MapRawTo2D(rawXY);
        var pos = Map2DToUnityWorld(mapped2);

        if (rotateInWorld)
            pos = RotateAroundY(pos, pivot, rotateDeg);

        return pos;
    }

    Vector3 MapVelToWorldDir(Vector2 velXY)
    {
        Vector3 d = ToUnityXZ(velXY) * worldScale;
        d.y = 0f;

        if (rotateInWorld)
        {
            var q = Quaternion.Euler(0f, rotateDeg, 0f);
            d = q * d;
            d.y = 0f;
        }

        return d;
    }

    bool TryGetInitialDirFromTrajectory(Trajectory traj, float frameF, Vector3 pivot, out Vector3 dirWorld)
    {
        dirWorld = Vector3.zero;

        if (traj.TryEvaluate(frameF, out _, out _, out var velNow, out var hasVelNow) && hasVelNow)
        {
            var d = MapVelToWorldDir(velNow);
            if (d.sqrMagnitude > dirEpsilon * dirEpsilon)
            {
                dirWorld = d;
                return true;
            }
        }

        float step = Mathf.Sign(playbackSpeed);
        if (Mathf.Approximately(step, 0f)) step = 1f;

        if (traj.TryEvaluate(frameF + step, out var rawNext, out _, out _, out _))
        {
            if (traj.TryEvaluate(frameF, out var rawNow, out _, out _, out _))
            {
                Vector3 p0 = MapRawToWorld(rawNow, pivot);
                Vector3 p1 = MapRawToWorld(rawNext, pivot);
                dirWorld = p1 - p0;
                dirWorld.y = 0f;
                return true;
            }
        }

        if (traj.TryEvaluate(frameF - step, out var rawPrev, out _, out _, out _))
        {
            if (traj.TryEvaluate(frameF, out var rawNow, out _, out _, out _))
            {
                Vector3 p0 = MapRawToWorld(rawPrev, pivot);
                Vector3 p1 = MapRawToWorld(rawNow, pivot);
                dirWorld = p1 - p0;
                dirWorld.y = 0f;
                return true;
            }
        }

        return false;
    }

    int EvaluateAndApplyTransforms()
    {
        currentWorldPos.Clear();
        int alive = 0;

        Vector3 pivot = GetPivotWorld();

        foreach (var kv in trajById)
        {
            int id = kv.Key;
            var traj = kv.Value;

            if (!goById.TryGetValue(id, out var go) || go == null)
                continue;

            bool wasActive = go.activeSelf;

            if (!traj.TryEvaluate(currentFrameF, out var rawXY, out var angDeg, out var rawVel, out var hasVel))
            {
                if (wasActive) go.SetActive(false);

                prevPosById.Remove(id);
                prevFrameById.Remove(id);
                yawVelById.Remove(id);
                lastDirById.Remove(id);
                continue;
            }

            alive++;

            Vector3 pos = MapRawToWorld(rawXY, pivot);

            bool hasPrev = prevPosById.TryGetValue(id, out var oldPos);
            go.transform.position = pos;

            if (driveAnimatorBlend && animById.TryGetValue(id, out var anim) && anim != null)
            {
                float speed = 0f;

                if (hasPrev && prevFrameById.TryGetValue(id, out _))
                {
                    float dt = Mathf.Max(1e-6f, Time.deltaTime);
                    speed = Vector3.Distance(oldPos, pos) / dt;
                }

                float target = (speedForFullWalk <= 1e-6f) ? 0f : Mathf.Clamp01(speed / speedForFullWalk);
                if (speed < speedEpsilon) target = 0f;

                if (blendDampTime > 0f) anim.SetFloat(blendParamName, target, blendDampTime, Time.deltaTime);
                else anim.SetFloat(blendParamName, target);
            }

            bool freshActivation = (!wasActive);

            if (applyYawFromAngle)
            {
                if (hasPrev)
                {
                    Vector3 dir = pos - oldPos;
                    dir.y = 0f;

                    if (dir.sqrMagnitude > dirEpsilon * dirEpsilon)
                        lastDirById[id] = dir.normalized;
                }
                else
                {
                    if (hasVel)
                    {
                        Vector3 d = MapVelToWorldDir(rawVel);
                        if (d.sqrMagnitude > dirEpsilon * dirEpsilon)
                            lastDirById[id] = d.normalized;
                    }
                    else
                    {
                        if (TryGetInitialDirFromTrajectory(traj, currentFrameF, pivot, out var initDir))
                        {
                            if (initDir.sqrMagnitude > dirEpsilon * dirEpsilon)
                                lastDirById[id] = initDir.normalized;
                        }
                    }
                }

                if (!lastDirById.TryGetValue(id, out var useDir) || useDir.sqrMagnitude < 1e-8f)
                    useDir = go.transform.forward;

                float targetYaw = Mathf.Atan2(useDir.x, useDir.z) * Mathf.Rad2Deg + yawOffsetDeg;

                if (freshActivation || !hasPrev || !smoothYaw)
                {
                    go.transform.rotation = Quaternion.Euler(0f, targetYaw, 0f);
                    yawVelById[id] = 0f;
                }
                else
                {
                    float curYaw = go.transform.eulerAngles.y;
                    float vel = 0f;
                    yawVelById.TryGetValue(id, out vel);

                    float newYaw = Mathf.SmoothDampAngle(
                        curYaw, targetYaw, ref vel,
                        yawSmoothTime, Mathf.Infinity, Time.deltaTime
                    );

                    yawVelById[id] = vel;
                    go.transform.rotation = Quaternion.Euler(0f, newYaw, 0f);
                }
            }

            if (!wasActive) go.SetActive(true);

            prevPosById[id] = pos;
            prevFrameById[id] = currentFrameF;

            currentWorldPos[id] = pos;
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
        float h = Mathf.Repeat(groupIndex * 0.137f, 1f);
        return Color.HSVToRGB(h, 0.85f, 1f);
    }

    static Vector3 RotateAroundY(Vector3 p, Vector3 pivot, float deg)
    {
        var q = Quaternion.Euler(0f, deg, 0f);
        return pivot + q * (p - pivot);
    }

    Vector3 GetPivotWorld()
    {
        switch (pivotMode)
        {
            case RotationPivotMode.WorldOrigin: return Vector3.zero;
            case RotationPivotMode.WorldOffset: return worldOffset;
            case RotationPivotMode.DataCentroid: return dataCentroidWorld;
            case RotationPivotMode.Custom: return customPivotWorld;
            default: return worldOffset;
        }
    }

    void ComputeDataCentroidWorld()
    {
        double sx = 0, sz = 0;
        long cnt = 0;

        foreach (var kv in trajById)
        {
            var t = kv.Value;
            if (t.frames == null) continue;

            for (int i = 0; i < t.frames.Length; i++)
            {
                Vector2 mapped2 = MapRawTo2D(t.xy[i]);
                var p = Map2DToUnityWorld(mapped2);
                sx += p.x;
                sz += p.z;
                cnt++;
            }
        }

        dataCentroidWorld = (cnt > 0)
            ? new Vector3((float)(sx / cnt), 0f, (float)(sz / cnt))
            : worldOffset;
    }

    static Vector2 ApplyHomography(Matrix3x3 h, Vector2 uv)
    {
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

        using (var sr = new StringReader(text))
        {
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
        }

        if (rows.Count < 3)
        {
            Debug.LogWarning("[ZaraGroupRotationSimulator_ETH] Homography parse failed. Using Identity.");
            return Matrix3x3.Identity;
        }

        return new Matrix3x3(
            rows[0][0], rows[0][1], rows[0][2],
            rows[1][0], rows[1][1], rows[1][2],
            rows[2][0], rows[2][1], rows[2][2]
        );
    }

    [Serializable]
    public struct Matrix3x3
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

    void StopPlaybackAndExitPlayMode()
    {
        isPlaying = false;

#if UNITY_EDITOR
        if (EditorApplication.isPlaying)
            EditorApplication.isPlaying = false;
#endif
    }

    public void ExportWorldTrajectories(string outPath, bool exportAngleFromMotion = true)
    {
        var inv = CultureInfo.InvariantCulture;
        Vector3 pivot = GetPivotWorld();

        using (var sw = new StreamWriter(outPath))
        {
            foreach (var kv in trajById)
            {
                int id = kv.Key;
                var t = kv.Value;
                if (t.frames == null || t.frames.Length == 0) continue;

                var wp = new Vector3[t.frames.Length];
                for (int i = 0; i < t.frames.Length; i++)
                {
                    Vector3 p = MapRawToWorld(t.xy[i], pivot);
                    wp[i] = p;
                }

                for (int i = 0; i < t.frames.Length; i++)
                {
                    int frame = t.frames[i];
                    float wx = wp[i].x;
                    float wz = wp[i].z;

                    float outAngDeg;
                    if (exportAngleFromMotion)
                    {
                        Vector3 dir;
                        if (i < wp.Length - 1) dir = wp[i + 1] - wp[i];
                        else dir = wp[i] - wp[Mathf.Max(0, i - 1)];

                        dir.y = 0f;
                        outAngDeg = (dir.sqrMagnitude < 1e-8f)
                            ? 0f
                            : Mathf.Atan2(dir.x, dir.z) * Mathf.Rad2Deg;
                    }
                    else
                    {
                        outAngDeg = t.angles[i] + (rotateInWorld ? rotateDeg : 0f);
                    }

                    sw.WriteLine(
                        $"{id}," +
                        $"{wx.ToString(inv)}," +
                        $"{wz.ToString(inv)}," +
                        $"{frame}," +
                        $"{outAngDeg.ToString(inv)}"
                    );
                }
            }
        }

        Debug.Log($"[ZaraGroupRotationSimulator_ETH] Exported WORLD trajectories: {outPath}");
    }



    [ContextMenu("Export WORLD Trajectories (persistentDataPath)")]
    void ExportWorldToPersistent()
    {
        string file = $"zara_world_rot{rotateDeg:0}.txt";
        string path = Path.Combine(Application.persistentDataPath, file);
        ExportWorldTrajectories(path, exportAngleFromMotion: true);
    }

    [ContextMenu("Export WORLD Trajectories (S:\\home\\iiixr\\Documents)")]
    void ExportWorldToSDrive()
    {
        string dir = @"S:\home\iiixr\Documents";
        Directory.CreateDirectory(dir);

        string file = $"zara_world_rot{rotateDeg:0}.txt";
        string path = Path.Combine(dir, file);

        ExportWorldTrajectories(path, exportAngleFromMotion: true);
    }


    // ---------- getters ----------
    public float CurrentFrameF => currentFrameF;
    public int GlobalFrame => Mathf.FloorToInt(currentFrameF);
    public int MinFrame => globalMinFrame;
    public int MaxFrame => globalMaxFrame;

    [Header("Trajectory Gizmos (ALL Agents)")]
    public bool drawAllTrajectories = false;
    public bool drawTrajectoriesInEditMode = true;

    [Tooltip("Play 모드에서만 적용: 현재 activeInHierarchy인 agent의 trajectory만 그립니다.")]
    public bool drawTrajectoriesOnlyForActiveAgents = false; // ✅ NEW

    public float trajY = 0.02f;
    [Min(1)] public int trajStride = 2;
    public bool drawTrajPoints = false;
    public float trajPointRadius = 0.02f;
    public bool trajectoryUseGroupColors = false;
    bool gizmoNeedsReload = true;

    void OnValidate()
    {
        gizmoNeedsReload = true;
    }
}
