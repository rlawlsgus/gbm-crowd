using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

[DefaultExecutionOrder(100)] // CollisionDebugGizmos(-100)보다 나중에
public class ZaraTelemetryRecorder : MonoBehaviour
{
    // ===================== Source selection =====================
    public enum SourceMode
    {
        Auto,
        UseSimulator,
        UseRotationSimulator,
        UseRotationSimulator_ETH, // ✅ added (append only)
    }

    [Header("Source")]
    public SourceMode sourceMode = SourceMode.Auto;

    public ZaraGroupSimulator simulator;
    public ZaraGroupRotationSimulator rotationSimlator; // (typo 유지: 기존 인스펙터 연결 깨지지 않게)

    // ✅ added: ETH rotation simulator
    public ZaraGroupRotationSimulator_ETH rotationSimlator_ETH;

    enum ActiveSourceKind { None, Simulator, Rotation, RotationETH }
    ActiveSourceKind activeKind = ActiveSourceKind.None;
    Transform activeRoot;

    // cached mapping config (active source 기준)
    bool srcUseHomography;
    bool srcFlipX;
    bool srcFlipZ;
    float srcWorldScale;
    Vector3 srcWorldOffset;
    Matrix3x3 srcH = Matrix3x3.Identity;

    // rotation-only config (rotation sim 선택시)
    bool srcRotateInWorld;
    float srcRotateDeg;
    int srcPivotMode;                 // ZaraGroupRotationSimulator.RotationPivotMode 값(0..)
    Vector3 srcCustomPivotWorld;

    // centroid/pivot cache (rotation sim의 DataCentroid 피벗 지원)
    Vector3 cachedDataCentroidWorld = Vector3.zero;
    bool cachedCentroidValid = false;

    // ===================== Output =====================
    [Header("Output")]
    public bool record = true;
    public string runName = "zara_run";
    public bool writeCombinedJsonl = false;   // frames.jsonl
    public bool writePerAgentJsonl = true;    // agents/agent_{id}.jsonl
    public bool imagesPerAgentFolder = true;

    public bool writeAgentSummaries = true;

    // ===================== Sampling =====================
    [Header("Sampling")]
    public int logEveryNGlobalFrames = 1;
    public int captureEveryNGlobalFrames = 10; // 0이면 캡쳐 안 함
    public bool captureOnlyWhenAgentActive = true;

    // ===================== Speed (dataset-time) =====================
    [Header("Speed (dataset-time)")]
    public float moveSpeedThreshold = 0.05f;
    public bool speedUseDatasetTime = true;

    // ===================== Group =====================
    [Header("Group")]
    public bool includeGroupInfo = true;

    // ===================== Camera Capture =====================
    [Header("Camera Capture")]
    public int captureWidth = 512;
    public int captureHeight = 512;
    public string cameraPathHint = "Rig 1/HeadAim/Camera";

    // ===================== Debug =====================
    [Header("Debug")]
    public bool warnIfCollisionNotPublished = true;



    readonly Dictionary<int, StreamWriter> agentWriters = new Dictionary<int, StreamWriter>(256);
    string agentsDir;

    string rootDir;
    StreamWriter framesWriter;
    StreamWriter summaryWriter;

    readonly Dictionary<int, AgentTrack> tracks = new Dictionary<int, AgentTrack>(256);

    readonly List<int[]> groups = new List<int[]>();
    readonly Dictionary<int, int> idToGroupIndex = new Dictionary<int, int>();
    readonly Dictionary<int, int[]> idToGroupMembers = new Dictionary<int, int[]>();

    int lastLoggedGlobalFrame = int.MinValue;
    bool warnedCollisionMissing;

    sealed class AgentTrack
    {
        public int id;
        public Transform t;
        public GameObject go;
        public Camera cam;

        public bool started;
        public int firstGlobalFrame;
        public Vector3 startWorld;
        public int lastGlobalFrame;
        public Vector3 lastWorld;

        public bool hasPrev;
        public int prevGlobalFrame;
        public Vector3 prevWorld;

        public int localFrame;
        public int movedFrames;
        public float movedTime;

        public RenderTexture rt;
        public Texture2D tex;

        public Vector3 trajStartWorld;
        public Vector3 trajEndWorld;

        public bool isActiveThisFrame;
        public Vector3 posThisFrame;
        public Vector3 velThisFrame;
        public float speedThisFrame;

        // ----- padding cache for group -----
        public bool everObserved;            // "관측" = active 상태로 프레임을 한 번이라도 기록한 적
        public Vector3 firstObservedWorld;   // 첫 관측 위치(참고용)
        public Vector3 lastObservedWorld;    // 마지막 관측 위치 (inactive 시 padding에 사용)
    }

    // ---------- endpoints precompute ----------
    struct Endpoints
    {
        public bool has;
        public int minF, maxF;
        public Vector2 minXY, maxXY;
    }

    readonly Dictionary<int, Vector3> trajStartById = new Dictionary<int, Vector3>(256);
    readonly Dictionary<int, Vector3> trajEndById = new Dictionary<int, Vector3>(256);

    // ---------- trajectory format detect ----------
    enum TrajTextFormat { ZaraCsv5, EthObsmat8 }

    void Awake()
    {
        // 기본 ref 찾기 (둘 중 하나라도)
        if (simulator == null) simulator = GetComponent<ZaraGroupSimulator>();
        if (simulator == null) simulator = FindAny<ZaraGroupSimulator>();

        if (rotationSimlator == null) rotationSimlator = GetComponent<ZaraGroupRotationSimulator>();
        if (rotationSimlator == null) rotationSimlator = FindAny<ZaraGroupRotationSimulator>();

        // ✅ ETH rotation simulator find
        if (rotationSimlator_ETH == null) rotationSimlator_ETH = GetComponent<ZaraGroupRotationSimulator_ETH>();
        if (rotationSimlator_ETH == null) rotationSimlator_ETH = FindAny<ZaraGroupRotationSimulator_ETH>();

        ResolveActiveSource();

        SetupOutput();
        ParseGroupsFromSource();

        PrecomputeTrajectoryEndpoints(); // active source 기준(회전 포함 가능)

        IndexAgentsUnderSource(lightRefresh: false);

        foreach (var kv in tracks)
        {
            var tr = kv.Value;
            if (tr == null) continue;

            if (trajStartById.TryGetValue(tr.id, out var ws)) tr.trajStartWorld = ws;
            if (trajEndById.TryGetValue(tr.id, out var we)) tr.trajEndWorld = we;
        }

        if (record && Application.isPlaying && activeKind != ActiveSourceKind.None)
        {
            Time.captureFramerate = Mathf.RoundToInt(GetSrcFps());
            Debug.Log($"[ZaraTelemetryRecorder] Time.captureFramerate set to {Time.captureFramerate} for deterministic recording.");
        }
    }

    void OnDestroy()
    {
        Time.captureFramerate = 0;

        try { framesWriter?.Flush(); framesWriter?.Dispose(); } catch { }
        try { summaryWriter?.Flush(); summaryWriter?.Dispose(); } catch { }

        foreach (var kv in tracks)
            if (kv.Value.rt != null) kv.Value.rt.Release();

        foreach (var w in agentWriters.Values)
        {
            try { w?.Flush(); w?.Dispose(); } catch { }
        }
        agentWriters.Clear();
    }

    // ===================== Source resolving =====================

    void ResolveActiveSource()
    {
        activeKind = ActiveSourceKind.None;
        activeRoot = null;

        bool simOk = simulator != null;
        bool rotOk = rotationSimlator != null;
        bool rotEthOk = rotationSimlator_ETH != null;

        if (sourceMode == SourceMode.UseRotationSimulator_ETH)
        {
            if (rotEthOk) activeKind = ActiveSourceKind.RotationETH;
            else if (rotOk) activeKind = ActiveSourceKind.Rotation;
            else if (simOk) activeKind = ActiveSourceKind.Simulator;
        }
        else if (sourceMode == SourceMode.UseRotationSimulator)
        {
            if (rotOk) activeKind = ActiveSourceKind.Rotation;
            else if (rotEthOk) activeKind = ActiveSourceKind.RotationETH;
            else if (simOk) activeKind = ActiveSourceKind.Simulator;
        }
        else if (sourceMode == SourceMode.UseSimulator)
        {
            if (simOk) activeKind = ActiveSourceKind.Simulator;
            else if (rotEthOk) activeKind = ActiveSourceKind.RotationETH;
            else if (rotOk) activeKind = ActiveSourceKind.Rotation;
        }
        else // Auto
        {
            // ✅ Auto priority: ETH Rotation -> Rotation -> Simulator
            if (rotEthOk) activeKind = ActiveSourceKind.RotationETH;
            else if (rotOk) activeKind = ActiveSourceKind.Rotation;
            else if (simOk) activeKind = ActiveSourceKind.Simulator;
        }

        if (activeKind == ActiveSourceKind.Simulator)
            activeRoot = simulator != null ? simulator.transform : null;
        else if (activeKind == ActiveSourceKind.Rotation)
            activeRoot = rotationSimlator != null ? rotationSimlator.transform : null;
        else if (activeKind == ActiveSourceKind.RotationETH)
            activeRoot = rotationSimlator_ETH != null ? rotationSimlator_ETH.transform : null;

        CacheSourceMappingConfig();

        if (activeKind == ActiveSourceKind.None)
            Debug.LogWarning("[ZaraTelemetryRecorder] No active source. Assign ZaraGroupSimulator / ZaraGroupRotationSimulator / ZaraGroupRotationSimulator_ETH.");
        else
            Debug.Log($"[ZaraTelemetryRecorder] Active source = {activeKind}");
    }

    void CacheSourceMappingConfig()
    {
        cachedCentroidValid = false;
        cachedDataCentroidWorld = Vector3.zero;

        srcUseHomography = false;
        srcFlipX = false;
        srcFlipZ = false;
        srcWorldScale = 1f;
        srcWorldOffset = Vector3.zero;
        srcH = Matrix3x3.Identity;

        srcRotateInWorld = false;
        srcRotateDeg = 0f;
        srcPivotMode = 0;
        srcCustomPivotWorld = Vector3.zero;

        if (activeKind == ActiveSourceKind.Simulator && simulator != null)
        {
            srcUseHomography = simulator.useHomography;
            srcFlipX = simulator.flipX;
            srcFlipZ = simulator.flipZ;
            srcWorldScale = simulator.worldScale;
            srcWorldOffset = simulator.worldOffset;

            if (srcUseHomography && simulator.homographyTxt != null)
                srcH = ParseHomography3x3(simulator.homographyTxt.text);
        }
        else if (activeKind == ActiveSourceKind.Rotation && rotationSimlator != null)
        {
            srcUseHomography = rotationSimlator.useHomography;
            srcFlipX = rotationSimlator.flipX;
            srcFlipZ = rotationSimlator.flipZ;
            srcWorldScale = rotationSimlator.worldScale;
            srcWorldOffset = rotationSimlator.worldOffset;

            if (srcUseHomography && rotationSimlator.homographyTxt != null)
                srcH = ParseHomography3x3(rotationSimlator.homographyTxt.text);

            srcRotateInWorld = rotationSimlator.rotateInWorld;
            srcRotateDeg = rotationSimlator.rotateDeg;
            srcCustomPivotWorld = rotationSimlator.customPivotWorld;
            srcPivotMode = (int)rotationSimlator.pivotMode;
        }
        else if (activeKind == ActiveSourceKind.RotationETH && rotationSimlator_ETH != null)
        {
            // ✅ ETH: world meters면 homography 재적용 금지
            srcUseHomography = rotationSimlator_ETH.useHomography && !rotationSimlator_ETH.ethPositionsAreWorldMeters;
            srcFlipX = rotationSimlator_ETH.flipX;
            srcFlipZ = rotationSimlator_ETH.flipZ;
            srcWorldScale = rotationSimlator_ETH.worldScale;
            srcWorldOffset = rotationSimlator_ETH.worldOffset;

            if (srcUseHomography && rotationSimlator_ETH.homographyTxt != null)
                srcH = ParseHomography3x3(rotationSimlator_ETH.homographyTxt.text);

            srcRotateInWorld = rotationSimlator_ETH.rotateInWorld;
            srcRotateDeg = rotationSimlator_ETH.rotateDeg;
            srcCustomPivotWorld = rotationSimlator_ETH.customPivotWorld;
            srcPivotMode = (int)rotationSimlator_ETH.pivotMode;
        }
    }

    float GetSrcFps()
    {
        if (activeKind == ActiveSourceKind.RotationETH && rotationSimlator_ETH != null) return rotationSimlator_ETH.fps;
        if (activeKind == ActiveSourceKind.Rotation && rotationSimlator != null) return rotationSimlator.fps;
        if (activeKind == ActiveSourceKind.Simulator && simulator != null) return simulator.fps;
        return 25f;
    }

    int GetSrcGlobalFrame()
    {
        if (activeKind == ActiveSourceKind.RotationETH && rotationSimlator_ETH != null) return rotationSimlator_ETH.GlobalFrame;
        if (activeKind == ActiveSourceKind.Rotation && rotationSimlator != null) return rotationSimlator.GlobalFrame;
        if (activeKind == ActiveSourceKind.Simulator && simulator != null) return simulator.GlobalFrame;
        return 0;
    }

    int GetSrcMinFrame()
    {
        if (activeKind == ActiveSourceKind.RotationETH && rotationSimlator_ETH != null) return rotationSimlator_ETH.MinFrame;
        if (activeKind == ActiveSourceKind.Rotation && rotationSimlator != null) return rotationSimlator.MinFrame;
        if (activeKind == ActiveSourceKind.Simulator && simulator != null) return simulator.MinFrame;
        return 0;
    }

    int GetSrcMaxFrame()
    {
        if (activeKind == ActiveSourceKind.RotationETH && rotationSimlator_ETH != null) return rotationSimlator_ETH.MaxFrame;
        if (activeKind == ActiveSourceKind.Rotation && rotationSimlator != null) return rotationSimlator.MaxFrame;
        if (activeKind == ActiveSourceKind.Simulator && simulator != null) return simulator.MaxFrame;
        return 0;
    }

    TextAsset GetSrcTrajectoriesTxt()
    {
        if (activeKind == ActiveSourceKind.RotationETH && rotationSimlator_ETH != null) return rotationSimlator_ETH.trajectoriesTxt;
        if (activeKind == ActiveSourceKind.Rotation && rotationSimlator != null) return rotationSimlator.trajectoriesTxt;
        if (activeKind == ActiveSourceKind.Simulator && simulator != null) return simulator.trajectoriesTxt;
        return null;
    }

    TextAsset GetSrcGroupsTxt()
    {
        if (activeKind == ActiveSourceKind.RotationETH && rotationSimlator_ETH != null) return rotationSimlator_ETH.groupsTxt;
        if (activeKind == ActiveSourceKind.Rotation && rotationSimlator != null) return rotationSimlator.groupsTxt;
        if (activeKind == ActiveSourceKind.Simulator && simulator != null) return simulator.groupsTxt;
        return null;
    }

    // ===================== Runtime loop =====================

    void LateUpdate()
    {
        if (!record) return;
        if (!Application.isPlaying) return;
        if (activeKind == ActiveSourceKind.None) return;

        int gFrame = GetSrcGlobalFrame();

        if (gFrame == lastLoggedGlobalFrame) return;
        lastLoggedGlobalFrame = gFrame;

        if (logEveryNGlobalFrames > 1 && (gFrame % logEveryNGlobalFrames) != 0) return;

        IndexAgentsUnderSource(lightRefresh: true);

        // PASS 1: pos/vel (+ observe cache)
        foreach (var kv in tracks)
        {
            var tr = kv.Value;
            if (tr == null || tr.go == null || tr.t == null) continue;

            bool active = tr.go.activeInHierarchy;
            tr.isActiveThisFrame = active;

            if (!active)
            {
                tr.posThisFrame = tr.t.position;
                tr.velThisFrame = Vector3.zero;
                tr.speedThisFrame = 0f;
                continue;
            }

            Vector3 pos = tr.t.position;
            tr.posThisFrame = pos;

            if (!tr.everObserved)
            {
                tr.everObserved = true;
                tr.firstObservedWorld = pos;
            }
            tr.lastObservedWorld = pos;

            if (!tr.started)
            {
                tr.started = true;
                tr.firstGlobalFrame = gFrame;
                tr.startWorld = pos;
                tr.lastGlobalFrame = gFrame;
                tr.lastWorld = pos;

                tr.hasPrev = false;
                tr.localFrame = 0;
                tr.movedFrames = 0;
                tr.movedTime = 0f;
            }

            tr.localFrame = gFrame - tr.firstGlobalFrame;

            float speed = 0f;
            Vector3 vel = Vector3.zero;

            if (tr.hasPrev)
            {
                int df = FrameDelta(tr.prevGlobalFrame, gFrame, GetSrcMinFrame(), GetSrcMaxFrame());
                float dt = speedUseDatasetTime
                    ? Mathf.Abs(df) / Mathf.Max(1f, GetSrcFps())
                    : Mathf.Max(1e-6f, Time.deltaTime);

                if (dt > 1e-6f)
                {
                    vel = (pos - tr.prevWorld) / dt;
                    speed = vel.magnitude;
                }

                float dtDataset = Mathf.Abs(df) / Mathf.Max(1f, GetSrcFps());
                if (speed > moveSpeedThreshold)
                {
                    tr.movedFrames += Mathf.Max(1, Mathf.Abs(df));
                    tr.movedTime += dtDataset;
                }
            }

            tr.prevGlobalFrame = gFrame;
            tr.prevWorld = pos;
            tr.hasPrev = true;

            tr.lastGlobalFrame = gFrame;
            tr.lastWorld = pos;

            tr.velThisFrame = vel;
            tr.speedThisFrame = speed;
        }

        // PASS 2: group/capture/collision(bus)/write
        foreach (var kv in tracks)
        {
            var tr = kv.Value;
            if (tr == null || tr.go == null || tr.t == null) continue;
            if (!tr.isActiveThisFrame) continue;

            int groupIndex = -1;
            int[] members = Array.Empty<int>();

            if (includeGroupInfo && idToGroupIndex.TryGetValue(tr.id, out var gi))
            {
                groupIndex = gi;
                members = idToGroupMembers[tr.id];
            }

            // collision: bus만 사용
            bool collisionAhead = false;
            int collisionOtherId = -1;
            float collisionT = -1f;
            float collisionMinDist = -1f;
            float collisionNowDist = -1f;

            ZaraCollisionBus.CollisionResult cr;
            if (ZaraCollisionBus.TryGet(gFrame, tr.id, out cr))
            {
                collisionAhead = cr.ahead;
                collisionOtherId = cr.otherId;
                collisionT = cr.timeToMinDistSec;
                collisionMinDist = cr.minDist;
                collisionNowDist = cr.nowDist;
            }
            else if (warnIfCollisionNotPublished && !warnedCollisionMissing)
            {
                warnedCollisionMissing = true;
                Debug.LogWarning("[ZaraTelemetryRecorder] CollisionBus has no data for globalFrame="
                                 + gFrame.ToString(CultureInfo.InvariantCulture)
                                 + " (LastPublishedFrame=" + ZaraCollisionBus.LastPublishedFrame.ToString(CultureInfo.InvariantCulture)
                                 + "). Check that ZaraCollisionDebugGizmos is enabled and execution order.");
            }

            // capture
            string imagePath = null;
            if (captureEveryNGlobalFrames > 0 &&
                (gFrame % captureEveryNGlobalFrames) == 0 &&
                (!captureOnlyWhenAgentActive || tr.isActiveThisFrame))
            {
                imagePath = CaptureAgentCamera(tr, gFrame);
            }

            WriteFrameRecord(
                tr,
                gFrame,
                tr.speedThisFrame,
                groupIndex,
                members,
                imagePath,
                collisionAhead,
                collisionOtherId,
                collisionT,
                collisionMinDist,
                collisionNowDist
            );
        }
    }

    // ===================== group padding helper =====================

    Vector3 GetPaddedWorldForMember(int memberId)
    {
        if (tracks.TryGetValue(memberId, out var mt) && mt != null)
        {
            if (mt.everObserved) return mt.lastObservedWorld;
        }

        if (trajStartById.TryGetValue(memberId, out var ws)) return ws;

        return Vector3.zero;
    }

    // ===================== indexing / parsing =====================

    void ParseGroupsFromSource()
    {
        groups.Clear();
        idToGroupIndex.Clear();
        idToGroupMembers.Clear();

        var gt = GetSrcGroupsTxt();
        if (gt == null) return;

        using (var sr = new StringReader(gt.text))
        {
            string line;
            while ((line = sr.ReadLine()) != null)
            {
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
                if (!ok) continue;

                int groupIdx = groups.Count;
                groups.Add(ids);

                foreach (var id in ids)
                {
                    if (!idToGroupIndex.ContainsKey(id))
                    {
                        idToGroupIndex[id] = groupIdx;
                        idToGroupMembers[id] = ids;
                    }
                }
            }
        }
    }

    void IndexAgentsUnderSource(bool lightRefresh = false)
    {
        if (activeRoot == null) return;

        foreach (Transform child in activeRoot)
        {
            if (child == null) continue;
            var go = child.gameObject;
            if (go == null) continue;
            if (!go.name.StartsWith("ped_", StringComparison.Ordinal)) continue;

            if (!TryParsePedId(go.name, out int id)) continue;

            if (!tracks.TryGetValue(id, out var tr) || tr == null)
            {
                tr = new AgentTrack
                {
                    id = id,
                    t = child,
                    go = go,
                    cam = FindAgentCamera(child),
                    trajStartWorld = trajStartById.TryGetValue(id, out var ws) ? ws : child.position,
                    trajEndWorld = trajEndById.TryGetValue(id, out var we) ? we : child.position,

                    everObserved = false,
                    firstObservedWorld = Vector3.zero,
                    lastObservedWorld = Vector3.zero
                };
                tracks[id] = tr;
            }
            else if (!lightRefresh)
            {
                tr.t = child;
                tr.go = go;
                tr.cam = FindAgentCamera(child);
            }
            else
            {
                tr.t = child;
                tr.go = go;
            }
        }
    }

    static bool TryParsePedId(string name, out int id)
    {
        id = -1;
        int idx = name.IndexOf('_');
        if (idx < 0 || idx + 1 >= name.Length) return false;
        string s = name.Substring(idx + 1);
        return int.TryParse(s, NumberStyles.Integer, CultureInfo.InvariantCulture, out id);
    }

    Camera FindAgentCamera(Transform pedRoot)
    {
        if (pedRoot == null) return null;

        if (!string.IsNullOrWhiteSpace(cameraPathHint))
        {
            var t = pedRoot.Find(cameraPathHint);
            if (t != null && t.TryGetComponent<Camera>(out var cam0)) return cam0;
        }

        var cams = pedRoot.GetComponentsInChildren<Camera>(true);
        return (cams != null && cams.Length > 0) ? cams[0] : null;
    }

    // ===================== output =====================

    void SetupOutput()
    {
        // 원하는 저장 루트 (반드시 @"" 또는 \\ 이스케이프)
        string baseDir = @"S:\home\iiixr\Documents";

        Debug.Log($"[ZaraTelemetryRecorder] Platform={Application.platform}");
        Debug.Log($"[ZaraTelemetryRecorder] baseDir={baseDir} exists? {Directory.Exists(baseDir)}");
        Debug.Log($"[ZaraTelemetryRecorder] S:\\ exists? {Directory.Exists(@"S:\")}");

        try
        {
            // 쓰기 테스트(권한/경로/매핑 확인)
            Directory.CreateDirectory(baseDir);
            File.WriteAllText(Path.Combine(baseDir, "_write_test.txt"), "ok");

            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            rootDir = Path.Combine(baseDir, runName + "_" + stamp);

            Directory.CreateDirectory(rootDir);
            Directory.CreateDirectory(Path.Combine(rootDir, "images"));

            agentsDir = Path.Combine(rootDir, "agents");
            Directory.CreateDirectory(agentsDir);

            if (writeCombinedJsonl)
                framesWriter = new StreamWriter(Path.Combine(rootDir, "frames.jsonl"), false, Encoding.UTF8) { AutoFlush = false };

            if (writeAgentSummaries)
                summaryWriter = new StreamWriter(Path.Combine(rootDir, "agents_summary.jsonl"), false, Encoding.UTF8) { AutoFlush = false };

            Debug.Log("[ZaraTelemetryRecorder] Output: " + rootDir);
        }
        catch (Exception e)
        {
            Debug.LogError("[ZaraTelemetryRecorder] SetupOutput failed:\n" + e);
            // 필요하면 fallback
            // rootDir = Path.Combine(Application.persistentDataPath, runName + "_" + stamp);
        }
    }


    StreamWriter GetAgentWriter(int agentId)
    {
        if (!writePerAgentJsonl) return null;

        if (agentWriters.TryGetValue(agentId, out var w) && w != null) return w;

        string path = Path.Combine(agentsDir, "agent_" + agentId.ToString(CultureInfo.InvariantCulture) + ".jsonl");
        w = new StreamWriter(path, false, Encoding.UTF8) { AutoFlush = false };
        agentWriters[agentId] = w;
        return w;
    }

    void WriteFrameRecord(
        AgentTrack tr,
        int globalFrame,
        float speed,
        int groupIndex,
        int[] members,
        string imagePath,
        bool collisionAhead,
        int collisionOtherId,
        float collisionTime,
        float collisionMinDist,
        float collisionNowDist)
    {
        if ((!writeCombinedJsonl || framesWriter == null) && !writePerAgentJsonl)
            return;

        Vector3 worldStart = tr.trajStartWorld;
        Vector3 worldEnd = tr.trajEndWorld;
        Vector3 worldCurrent = tr.posThisFrame;

        Vector3 localStart = Vector3.zero;
        Vector3 localEnd = worldEnd - worldStart;
        Vector3 localCurrent = worldCurrent - worldStart;

        var sb = new StringBuilder(1024);
        sb.Append('{');

        AppendKV(sb, "agentId", tr.id); sb.Append(',');
        AppendKV(sb, "globalFrame", globalFrame); sb.Append(',');
        AppendKV(sb, "localFrame", tr.localFrame); sb.Append(',');

        AppendKV(sb, "speed", speed); sb.Append(',');

        AppendVec3(sb, "worldStart", worldStart); sb.Append(',');
        AppendVec3(sb, "worldCurrent", worldCurrent); sb.Append(',');
        AppendVec3(sb, "worldEnd", worldEnd); sb.Append(',');

        AppendVec3(sb, "localStart", localStart); sb.Append(',');
        AppendVec3(sb, "localCurrent", localCurrent); sb.Append(',');
        AppendVec3(sb, "localEnd", localEnd); sb.Append(',');

        AppendKV(sb, "movedFrames", tr.movedFrames); sb.Append(',');
        AppendKV(sb, "movedTimeSec", tr.movedTime); sb.Append(',');

        // group: self 제외, abs/rel 항상 기록(패딩 포함), active 여부 무관
        sb.Append("\"group\":{");
        AppendKV(sb, "index", groupIndex); sb.Append(',');
        sb.Append("\"members\":[");

        if (includeGroupInfo && groupIndex >= 0 && members != null && members.Length > 0)
        {
            bool first = true;
            Vector3 myPos = tr.posThisFrame;

            foreach (var mid in members)
            {
                if (mid == tr.id) continue; // self 제외

                Vector3 abs = GetPaddedWorldForMember(mid);
                Vector3 rel = abs - myPos;

                if (!first) sb.Append(',');
                first = false;

                sb.Append('{');
                AppendKV(sb, "id", mid); sb.Append(',');
                AppendVec3(sb, "abs", abs); sb.Append(',');
                AppendVec3(sb, "rel", rel);
                sb.Append('}');
            }
        }

        sb.Append("]}");
        sb.Append(',');

        // image
        sb.Append("\"image\":{");
        if (tr.cam != null)
        {
            AppendKV(sb, "hasCamera", 1); sb.Append(',');
            AppendKV(sb, "width", captureWidth); sb.Append(',');
            AppendKV(sb, "height", captureHeight); sb.Append(',');
            AppendKV(sb, "fov", tr.cam.fieldOfView);
        }
        else
        {
            AppendKV(sb, "hasCamera", 0);
        }
        sb.Append("},");
        AppendKS(sb, "imagePath", imagePath ?? "");
        sb.Append(',');

        // collision
        sb.Append("\"collision\":{");
        AppendKV(sb, "ahead", collisionAhead ? 1 : 0); sb.Append(',');
        AppendKV(sb, "otherId", collisionOtherId); sb.Append(',');
        AppendKV(sb, "timeToMinDistSec", collisionTime); sb.Append(',');
        AppendKV(sb, "minDist", collisionMinDist); sb.Append(',');
        AppendKV(sb, "nowDist", collisionNowDist);
        sb.Append('}');

        sb.Append('}');
        string line = sb.ToString();

        if (writeCombinedJsonl && framesWriter != null)
            framesWriter.WriteLine(line);

        var aw = GetAgentWriter(tr.id);
        if (aw != null) aw.WriteLine(line);
    }

    // ===================== camera capture =====================

    string CaptureAgentCamera(AgentTrack tr, int globalFrame)
    {
        if (tr == null || tr.cam == null) return null;

        if (tr.rt == null || tr.rt.width != captureWidth || tr.rt.height != captureHeight)
        {
            if (tr.rt != null) tr.rt.Release();
            tr.rt = new RenderTexture(captureWidth, captureHeight, 24, RenderTextureFormat.ARGB32);
            tr.tex = new Texture2D(captureWidth, captureHeight, TextureFormat.RGB24, false);
        }

        string imgDir = imagesPerAgentFolder
            ? Path.Combine(rootDir, "images", "agent_" + tr.id.ToString(CultureInfo.InvariantCulture))
            : Path.Combine(rootDir, "images");

        Directory.CreateDirectory(imgDir);

        string fname = "agent_" + tr.id.ToString(CultureInfo.InvariantCulture) + "_f_" +
                       globalFrame.ToString(CultureInfo.InvariantCulture) + ".png";
        string full = Path.Combine(imgDir, fname);

        var prevTarget = tr.cam.targetTexture;
        var prevActive = RenderTexture.active;

        try
        {
            tr.cam.targetTexture = tr.rt;
            RenderTexture.active = tr.rt;

            tr.cam.Render();

            tr.tex.ReadPixels(new Rect(0, 0, captureWidth, captureHeight), 0, 0);
            tr.tex.Apply(false, false);

            byte[] png = tr.tex.EncodeToPNG();
            File.WriteAllBytes(full, png);
        }
        catch (Exception e)
        {
            Debug.LogWarning("[ZaraTelemetryRecorder] Capture failed for agent " + tr.id + ": " + e.Message);
            return null;
        }
        finally
        {
            tr.cam.targetTexture = prevTarget;
            RenderTexture.active = prevActive;
        }

        return full;
    }

    // ===================== endpoint precompute (active source 기준) =====================

    static TrajTextFormat DetectTrajFormat(string text)
    {
        using (var sr = new StringReader(text))
        {
            string line;
            while ((line = sr.ReadLine()) != null)
            {
                line = line.Trim();
                if (line.Length == 0) continue;

                if (line.Contains(",")) return TrajTextFormat.ZaraCsv5;

                var toks = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                if (toks.Length >= 8) return TrajTextFormat.EthObsmat8;

                // fallback
                return TrajTextFormat.ZaraCsv5;
            }
        }
        return TrajTextFormat.ZaraCsv5;
    }

    static int ParseIntFromFloatToken(string tok, CultureInfo inv)
    {
        if (float.TryParse(tok, NumberStyles.Float, inv, out var f))
            return Mathf.RoundToInt(f);

        if (int.TryParse(tok, NumberStyles.Integer, inv, out var i))
            return i;

        return 0;
    }

    void PrecomputeTrajectoryEndpoints()
    {
        trajStartById.Clear();
        trajEndById.Clear();

        cachedCentroidValid = false;
        cachedDataCentroidWorld = Vector3.zero;

        var trajTxt = GetSrcTrajectoriesTxt();
        if (trajTxt == null) return;

        var inv = CultureInfo.InvariantCulture;
        var ep = new Dictionary<int, Endpoints>(256);

        var fmt = DetectTrajFormat(trajTxt.text);

        // centroid accumulator (rotationSim의 DataCentroid 피벗 대응)
        double cx = 0, cz = 0;
        long ccnt = 0;

        using (var sr = new StringReader(trajTxt.text))
        {
            string line;
            while ((line = sr.ReadLine()) != null)
            {
                if (string.IsNullOrWhiteSpace(line)) continue;

                int id, frame;
                Vector2 rawXY;

                if (fmt == TrajTextFormat.ZaraCsv5)
                {
                    var parts = line.Split(',');
                    if (parts.Length < 5) continue;

                    id = int.Parse(parts[0].Trim(), inv);
                    float x = float.Parse(parts[1].Trim(), inv);
                    float y = float.Parse(parts[2].Trim(), inv);
                    frame = int.Parse(parts[3].Trim(), inv);

                    rawXY = new Vector2(x, y);
                }
                else
                {
                    // ETH: [frame_number pedestrian_ID pos_x pos_z pos_y v_x v_z v_y]
                    var toks = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                    if (toks.Length < 8) continue;

                    frame = ParseIntFromFloatToken(toks[0], inv);
                    id = ParseIntFromFloatToken(toks[1], inv);

                    float posX = float.Parse(toks[2], NumberStyles.Float, inv);
                    float posY = float.Parse(toks[4], NumberStyles.Float, inv);

                    rawXY = new Vector2(posX, posY);
                }

                if (!ep.TryGetValue(id, out var e) || !e.has)
                {
                    e = new Endpoints { has = true, minF = frame, maxF = frame, minXY = rawXY, maxXY = rawXY };
                }
                else
                {
                    if (frame < e.minF) { e.minF = frame; e.minXY = rawXY; }
                    if (frame > e.maxF) { e.maxF = frame; e.maxXY = rawXY; }
                }
                ep[id] = e;

                Vector3 pBase = MapRawToWorldBase(rawXY);
                cx += pBase.x;
                cz += pBase.z;
                ccnt++;
            }
        }

        if (ccnt > 0)
        {
            cachedDataCentroidWorld = new Vector3((float)(cx / ccnt), 0f, (float)(cz / ccnt));
            cachedCentroidValid = true;
        }

        foreach (var kv in ep)
        {
            int id = kv.Key;
            var e = kv.Value;

            Vector3 ws = MapRawToWorld(e.minXY);
            Vector3 we = MapRawToWorld(e.maxXY);

            trajStartById[id] = ws;
            trajEndById[id] = we;
        }
    }

    Vector3 MapRawToWorldBase(Vector2 rawXY)
    {
        Vector2 w2 = srcUseHomography ? ApplyHomography(srcH, rawXY) : rawXY;

        float x = srcFlipX ? -w2.x : w2.x;
        float z = srcFlipZ ? -w2.y : w2.y;

        Vector3 p = new Vector3(x, 0f, z);
        return p * srcWorldScale + srcWorldOffset;
    }

    Vector3 GetRotationPivotWorld()
    {
        // 0 WorldOrigin, 1 WorldOffset, 2 DataCentroid, 3 Custom
        switch (srcPivotMode)
        {
            case 0: return Vector3.zero;
            case 1: return srcWorldOffset;
            case 2: return cachedCentroidValid ? cachedDataCentroidWorld : srcWorldOffset;
            case 3: return srcCustomPivotWorld;
            default: return srcWorldOffset;
        }
    }

    Vector3 MapRawToWorld(Vector2 rawXY)
    {
        Vector3 p = MapRawToWorldBase(rawXY);

        // ✅ Rotation / RotationETH 둘 다 여기서 처리
        if ((activeKind == ActiveSourceKind.Rotation || activeKind == ActiveSourceKind.RotationETH) && srcRotateInWorld)
        {
            Vector3 pivot = GetRotationPivotWorld();
            p = RotateAroundY(p, pivot, srcRotateDeg);
        }

        return p;
    }

    static Vector3 RotateAroundY(Vector3 p, Vector3 pivot, float deg)
    {
        var q = Quaternion.Euler(0f, deg, 0f);
        return pivot + q * (p - pivot);
    }

    // ===================== math / json helpers =====================

    static int FrameDelta(int prev, int cur, int minF, int maxF)
    {
        int df = cur - prev;
        int len = (maxF - minF + 1);
        if (len <= 1) return df;

        if (df > len / 2) df -= len;
        else if (df < -len / 2) df += len;
        return df;
    }

    static void AppendKV(StringBuilder sb, string k, int v)
    {
        sb.Append('\"').Append(k).Append("\":").Append(v.ToString(CultureInfo.InvariantCulture));
    }

    static void AppendKV(StringBuilder sb, string k, float v)
    {
        sb.Append('\"').Append(k).Append("\":").Append(v.ToString("0.######", CultureInfo.InvariantCulture));
    }

    static void AppendKS(StringBuilder sb, string k, string s)
    {
        sb.Append('\"').Append(k).Append("\":\"").Append(Escape(s)).Append('\"');
    }

    static void AppendVec3(StringBuilder sb, string k, Vector3 v)
    {
        sb.Append('\"').Append(k).Append("\":[")
          .Append(v.x.ToString("0.######", CultureInfo.InvariantCulture)).Append(',')
          .Append(v.y.ToString("0.######", CultureInfo.InvariantCulture)).Append(',')
          .Append(v.z.ToString("0.######", CultureInfo.InvariantCulture)).Append(']');
    }

    static string Escape(string s)
    {
        if (string.IsNullOrEmpty(s)) return "";
        return s.Replace("\\", "\\\\").Replace("\"", "\\\"");
    }

    // ===================== homography helpers =====================

    static Vector2 ApplyHomography(Matrix3x3 h, Vector2 uv)
    {
        float u = uv.x, v = uv.y;
        float x = h.m00 * u + h.m01 * v + h.m02;
        float y = h.m10 * u + h.m11 * v + h.m12;
        float w = h.m20 * u + h.m21 * v + h.m22;
        if (!Mathf.Approximately(w, 0f)) { x /= w; y /= w; }
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

        if (rows.Count < 3) return Matrix3x3.Identity;

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

        public static Matrix3x3 Identity => new Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);

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

    // ===================== misc =====================

    static T FindAny<T>() where T : UnityEngine.Object
    {
#if UNITY_2023_1_OR_NEWER
        return UnityEngine.Object.FindFirstObjectByType<T>();
#else
        return UnityEngine.Object.FindObjectOfType<T>();
#endif
    }
}
