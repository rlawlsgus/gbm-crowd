using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;

// Requires Newtonsoft JSON (com.unity.nuget.newtonsoft-json)
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System.Xml.Linq;
using System.Xml;

public class ZaraTelemetryLabeler : MonoBehaviour
{
    [Header("Input")]
    [Tooltip("비워두면 Application.persistentDataPath에서 runName_* 중 최신 폴더를 자동 선택")]
    public string runFolderPath = "";
    public string runName = "zara_run";

    [Tooltip("agents 폴더를 입력으로 사용 (recorder의 per-agent jsonl)")]
    public bool inputIsPerAgentFiles = true;

    [Header("Timing")]
    [Tooltip("dataset fps (recorder가 speed를 dataset-time으로 계산한 값과 맞춰야 함)")]
    public float datasetFps = 25f;

    [Header("Motion thresholds")]
    public float stopSpeed = 0.05f;          // 정지
    public float moveSpeed = 0.10f;          // 이동으로 보는 최소
    public float accelThr = 0.5f;            // 가속/감속 임계 (m/s^2)
    public float hardBrakeThr = 1.5f;        // 급감속 임계 (m/s^2)
    public float suddenStopPrevSpeed = 0.6f; // 이전에 이 정도 이상 달리다가
    public float suddenStopNowSpeed = 0.05f; // 지금 거의 정지면 급정지

    [Header("Turning (motion)")]
    [Tooltip("이 속도 미만이면 turn 판정 안 함")]
    public float minSpeedForTurn = 0.10f;
    [Tooltip("약회전 임계각(도)")]
    public float slightTurnDegThr = 15f;
    [Tooltip("강회전 임계각(도)")]
    public float hardTurnDegThr = 45f;

    [Header("Collision avoidance thresholds")]
    public float turnDegThr = 45f;           // (collision) 방향 급변
    public float lateralMetersThr = 0.25f;   // 횡이동 최소
    public float lateralRatioThr = 0.45f;    // (횡이동 / 총이동) 비율로도 판정

    [Header("Loitering (서성거림)")]
    public float loiterMinDurationSec = 10f; // 최소 지속시간
    public float loiterMaxRadius = 1.0f;     // 작은 영역 반경
    public float wanderRatioThr = 0.25f;     // 순이동/총이동 비율(낮을수록 목적성 약함)
    public float gridCell = 0.35f;           // 반복방문 체크(격자)
    public float revisitRateThr = 0.45f;     // (재방문 스텝 비율)

    [Header("Output")]
    public string outputSubFolder = "labeled";     // runFolder/labeled/agents/*.jsonl
    public bool overwriteIfExists = true;


    [ContextMenu("Label Latest Run (or runFolderPath)")]
    public void Label()
    {
        try
        {
            string run = ResolveRunFolder();
            if (string.IsNullOrEmpty(run))
            {
                Debug.LogError("[ZaraTelemetryLabeler] Run folder not found.");
                return;
            }

            if (datasetFps <= 0f) datasetFps = 25f;

            string inAgentsDir = inputIsPerAgentFiles
                ? Path.Combine(run, "agents")
                : run;

            if (!Directory.Exists(inAgentsDir))
            {
                Debug.LogError($"[ZaraTelemetryLabeler] Input directory not found: {inAgentsDir}");
                return;
            }

            string outAgentsDir = Path.Combine(run, outputSubFolder, "agents");
            if (Directory.Exists(outAgentsDir) && overwriteIfExists)
                Directory.Delete(outAgentsDir, recursive: true);
            Directory.CreateDirectory(outAgentsDir);

            // 1) load all agents (for cross-agent lookup e.g., otherId position)
            var db = LoadAllAgents(inAgentsDir);

            // 2) compute per-agent summary labels (loitering, totals)
            var agentSummary = ComputeAgentSummaries(db);

            // 3) per-agent: compute per-frame labels (motion/group/collision) and write
            foreach (var kv in db.byAgent)
            {
                int agentId = kv.Key;
                var frames = kv.Value;

                ApplyDerivedKinematics(frames, datasetFps);
                ApplyGroupLabels(frames);
                ApplyCollisionLabels(frames, db, datasetFps);
                ApplyLoiteringLabelToFrames(frames, agentSummary[agentId]);

                // write
                string outPath = Path.Combine(outAgentsDir, $"agent_{agentId}.jsonl");
                using var sw = new StreamWriter(outPath, append: false, System.Text.Encoding.UTF8);

                foreach (var fr in frames)
                {
                    sw.WriteLine(fr.json.ToString(Newtonsoft.Json.Formatting.None));
                }
            }

            Debug.Log($"[ZaraTelemetryLabeler] Done. Output: {Path.Combine(run, outputSubFolder)}");
        }
        catch (Exception e)
        {
            Debug.LogError("[ZaraTelemetryLabeler] Failed: " + e);
        }
    }

    // ----------------------------
    // Data models
    // ----------------------------
    sealed class FrameRec
    {
        public int agentId;
        public int globalFrame;

        public Vector3 worldCurrent;
        public int groupIndex;
        public int membersCount; // includes self in your recorder (so others = max(0, membersCount-1))

        public int collisionAhead;
        public int collisionOtherId;
        public float collisionTimeToMin;
        public float collisionMinDist;
        public float collisionNowDist;

        public JObject json;

        // derived
        public bool discontinuity;
        public float dt;
        public Vector3 vel;
        public float speed;
        public float accel;

        public Vector3 deltaPos;      // current - prev
        public float signedYawDeg;    // +left, -right
        public string turnState;      // Straight / TurnLeft / TurnRight / TurnSlightLeft / TurnSlightRight / None
    }

    sealed class AgentDB
    {
        public readonly Dictionary<int, List<FrameRec>> byAgent = new();
        public readonly Dictionary<int, Dictionary<int, FrameRec>> byAgentFrame = new(); // agentId -> globalFrame -> rec
    }

    struct AgentSummary
    {
        public bool isLoiter;
        public string loiterType; // SmallArea | Wandering | Repeating | None
        public float durationSec;
        public float totalDist;
        public float netDist;
        public float netToTotalRatio;
        public float radius; // max distance from centroid
        public float revisitRate;
    }

    // ----------------------------
    // Loading
    // ----------------------------
    AgentDB LoadAllAgents(string agentsDir)
    {
        var db = new AgentDB();

        string[] files = Directory.GetFiles(agentsDir, "agent_*.jsonl", SearchOption.TopDirectoryOnly);
        if (files.Length == 0)
        {
            // fallback: maybe user wants to process combined frames.jsonl
            string combined = Path.Combine(agentsDir, "frames.jsonl");
            if (File.Exists(combined))
                files = new[] { combined };
        }

        foreach (var path in files)
        {
            foreach (var line in File.ReadLines(path))
            {
                if (string.IsNullOrWhiteSpace(line)) continue;

                JObject jo;
                try { jo = JObject.Parse(line); }
                catch { continue; }

                int agentId = jo.Value<int?>("agentId") ?? -1;
                int gFrame = jo.Value<int?>("globalFrame") ?? int.MinValue;
                if (agentId < 0 || gFrame == int.MinValue) continue;

                Vector3 wc = ReadVec3(jo, "worldCurrent");

                var group = jo["group"] as JObject;
                int gi = group?.Value<int?>("index") ?? -1;
                int mcount = (group?["members"] as JArray)?.Count ?? 0;

                var col = jo["collision"] as JObject;
                int ahead = col?.Value<int?>("ahead") ?? 0;
                int other = col?.Value<int?>("otherId") ?? -1;
                float tmin = col?.Value<float?>("timeToMinDistSec") ?? -1f;
                float mind = col?.Value<float?>("minDist") ?? -1f;
                float nowd = col?.Value<float?>("nowDist") ?? -1f;

                var fr = new FrameRec
                {
                    agentId = agentId,
                    globalFrame = gFrame,
                    worldCurrent = wc,
                    groupIndex = gi,
                    membersCount = mcount,
                    collisionAhead = ahead,
                    collisionOtherId = other,
                    collisionTimeToMin = tmin,
                    collisionMinDist = mind,
                    collisionNowDist = nowd,
                    json = jo
                };

                if (!db.byAgent.TryGetValue(agentId, out var list))
                {
                    list = new List<FrameRec>(4096);
                    db.byAgent[agentId] = list;
                    db.byAgentFrame[agentId] = new Dictionary<int, FrameRec>(4096);
                }

                list.Add(fr);
                // 같은 globalFrame이 중복이면 마지막을 유지
                db.byAgentFrame[agentId][gFrame] = fr;
            }
        }

        // sort per agent by globalFrame (단, loop로 globalFrame이 떨어지면 discontinuity로 처리)
        foreach (var kv in db.byAgent)
            kv.Value.Sort((a, b) => a.globalFrame.CompareTo(b.globalFrame));

        return db;
    }

    // ----------------------------
    // Kinematics (pos -> vel/accel -> motion labels + delta + turning)
    // ----------------------------
    void ApplyDerivedKinematics(List<FrameRec> frames, float fps)
    {
        if (frames.Count == 0) return;

        FrameRec prev = null;
        float prevSpeed = 0f;

        for (int i = 0; i < frames.Count; i++)
        {
            var cur = frames[i];

            if (prev == null)
            {
                cur.discontinuity = true;
                cur.dt = 0f;
                cur.vel = Vector3.zero;
                cur.speed = 0f;
                cur.accel = 0f;
                cur.deltaPos = Vector3.zero;
                cur.signedYawDeg = 0f;
                cur.turnState = "None";

                WriteMotionLabel(cur, "Unknown", 0f, 0f, Vector3.zero, Vector3.zero);
                WriteTurnLabel(cur, "None", 0f);

                prev = cur;
                prevSpeed = 0f;
                continue;
            }

            int df = cur.globalFrame - prev.globalFrame;
            if (df <= 0)
            {
                // loop or disorder -> 끊김으로 보고 reset
                cur.discontinuity = true;
                cur.dt = 0f;
                cur.vel = Vector3.zero;
                cur.speed = 0f;
                cur.accel = 0f;
                cur.deltaPos = Vector3.zero;
                cur.signedYawDeg = 0f;
                cur.turnState = "None";

                WriteMotionLabel(cur, "Unknown", 0f, 0f, Vector3.zero, Vector3.zero);
                WriteTurnLabel(cur, "None", 0f);

                prev = cur;
                prevSpeed = 0f;
                continue;
            }

            float dt = df / Mathf.Max(1e-6f, fps);
            Vector3 delta = (cur.worldCurrent - prev.worldCurrent);
            Vector3 vel = delta / Mathf.Max(1e-6f, dt);

            float speed = vel.magnitude;
            float accel = (speed - prevSpeed) / Mathf.Max(1e-6f, dt);

            cur.discontinuity = false;
            cur.dt = dt;
            cur.deltaPos = delta;
            cur.vel = vel;
            cur.speed = speed;
            cur.accel = accel;

            // motion state
            string motion;
            if (prevSpeed >= suddenStopPrevSpeed && speed <= suddenStopNowSpeed && accel <= -hardBrakeThr)
                motion = "SuddenStop";
            else if (speed <= stopSpeed)
                motion = "Stop";
            else if (accel >= accelThr)
                motion = "Accelerate";
            else if (accel <= -accelThr)
                motion = "Decelerate";
            else if (speed >= moveSpeed)
                motion = "Move";
            else
                motion = "SlowMove";

            WriteMotionLabel(cur, motion, speed, accel, vel, delta);

            // turning label (local: prev frame dir -> current frame dir)
            float yawDeg = 0f;
            string turnState = "None";
            if (!prev.discontinuity &&
                prevSpeed >= minSpeedForTurn &&
                speed >= minSpeedForTurn)
            {
                yawDeg = SignedAngleOnXZ(prev.vel, vel);
                turnState = ClassifyTurn(yawDeg, slightTurnDegThr, hardTurnDegThr);
            }
            else if (speed >= minSpeedForTurn)
            {
                // moving but no reliable previous direction
                yawDeg = 0f;
                turnState = "None";
            }

            cur.signedYawDeg = yawDeg;
            cur.turnState = turnState;
            WriteTurnLabel(cur, turnState, yawDeg);

            prev = cur;
            prevSpeed = speed;
        }
    }

    void WriteMotionLabel(FrameRec fr, string state, float speed, float accel, Vector3 vel, Vector3 delta)
    {
        var labels = EnsureObj(fr.json, "labels");
        var motion = EnsureObj(labels, "motion");

        motion["state"] = state;
        motion["speed"] = Round6(speed);
        motion["accel"] = Round6(accel);
        motion["velocity"] = new JArray(Round6(vel.x), Round6(vel.y), Round6(vel.z));

        // ★ 이전 프레임 대비 이동량 기록
        motion["deltaWorld"] = new JArray(Round6(delta.x), Round6(delta.y), Round6(delta.z));
        motion["deltaXZ"] = new JArray(Round6(delta.x), Round6(delta.z));
    }

    void WriteTurnLabel(FrameRec fr, string state, float yawDeg)
    {
        var labels = EnsureObj(fr.json, "labels");
        var turn = EnsureObj(labels, "turning");
        turn["state"] = state;
        turn["yawDeg"] = Round6(yawDeg);
    }

    static float SignedAngleOnXZ(Vector3 from, Vector3 to)
    {
        from.y = 0f; to.y = 0f;
        if (from.sqrMagnitude < 1e-6f || to.sqrMagnitude < 1e-6f) return 0f;
        from.Normalize(); to.Normalize();
        return Vector3.SignedAngle(from, to, Vector3.up); // +면 좌, -면 우(일반적)
    }

    static string ClassifyTurn(float yawDeg, float slightThr, float hardThr)
    {
        float a = Mathf.Abs(yawDeg);
        if (a < slightThr) return "Straight";
        if (yawDeg >= hardThr) return "TurnLeft";
        if (yawDeg <= -hardThr) return "TurnRight";
        return (yawDeg > 0f) ? "TurnSlightLeft" : "TurnSlightRight";
    }

    // ----------------------------
    // Group labels (in-group / join / leave)
    // ----------------------------
    void ApplyGroupLabels(List<FrameRec> frames)
    {
        bool prevInGroup = false;

        foreach (var fr in frames)
        {
            int others = Mathf.Max(0, fr.membersCount - 1); // self 포함 기록이면 -1

            bool hasGroup = (fr.groupIndex >= 0);
            bool inGroup = hasGroup && (others >= 1);

            string gstate;

            if (!hasGroup)
            {
                gstate = "Solo";
            }
            else
            {
                if (!prevInGroup && inGroup) gstate = "Join";
                else if (prevInGroup && !inGroup) gstate = "Leave";
                else if (inGroup) gstate = "InGroup";
                else gstate = "GroupApart"; // 그룹은 있는데(>=0) 지금 others==0
            }

            var labels = EnsureObj(fr.json, "labels");
            var group = EnsureObj(labels, "groupBehavior");
            group["state"] = gstate;
            group["groupIndex"] = fr.groupIndex;
            group["othersCount"] = others;

            prevInGroup = inGroup;
        }
    }

    // ----------------------------
    // Collision (Risk / Avoided + avoidance type)
    // ----------------------------
    void ApplyCollisionLabels(List<FrameRec> frames, AgentDB db, float fps)
    {
        bool inRisk = false;
        int riskStartIdx = -1;
        int riskOtherId = -1;

        for (int i = 0; i < frames.Count; i++)
        {
            var fr = frames[i];

            if (fr.collisionAhead == 1)
            {
                if (!inRisk)
                {
                    inRisk = true;
                    riskStartIdx = i;
                    riskOtherId = fr.collisionOtherId;
                }

                WriteCollisionLabel(fr, "Risk", fr.collisionOtherId, fr.collisionTimeToMin, fr.collisionMinDist, fr.collisionNowDist, new string[0]);
                continue;
            }

            // fr.collisionAhead == 0
            if (inRisk)
            {
                int clearIdx = i;
                int startIdx = riskStartIdx;
                int endRiskIdx = i - 1;

                var avoidTypes = ClassifyAvoidance(frames, startIdx, clearIdx, riskOtherId, db);

                WriteCollisionLabel(frames[clearIdx], "Avoided", riskOtherId,
                    frames[endRiskIdx].collisionTimeToMin,
                    frames[endRiskIdx].collisionMinDist,
                    frames[clearIdx].collisionNowDist,
                    avoidTypes);

                inRisk = false;
                riskStartIdx = -1;
                riskOtherId = -1;
            }
            else
            {
                WriteCollisionLabel(fr, "None", -1, -1f, -1f, -1f, new string[0]);
            }
        }

        // 끝까지 risk가 유지된 경우: 마지막 프레임은 Risk로 이미 찍혀있음
    }

    string[] ClassifyAvoidance(List<FrameRec> frames, int startIdx, int clearIdx, int otherId, AgentDB db)
    {
        if (startIdx < 0 || clearIdx <= startIdx || clearIdx >= frames.Count)
            return new[] { "Unknown" };

        var s = frames[startIdx];
        var c = frames[clearIdx];

        // 내 움직임 기반
        Vector3 v0 = s.vel;
        Vector3 v1 = c.vel;
        float sp0 = s.speed;
        float sp1 = c.speed;

        // 방향 급변 (XZ)
        float turnDeg = AngleOnXZ(v0, v1);
        bool turn = turnDeg >= turnDegThr;

        // 감속/정지
        bool stop = (sp0 >= suddenStopPrevSpeed && sp1 <= stopSpeed);
        bool brake = false;

        // start~clear 구간 중 가장 큰 negative accel 확인
        float minAccel = float.PositiveInfinity;
        for (int k = startIdx; k <= clearIdx; k++)
            minAccel = Mathf.Min(minAccel, frames[k].accel);

        brake = (minAccel <= -hardBrakeThr) || (sp1 < sp0 && (sp0 - sp1) > 0.4f);

        // 횡이동: other 위치를 통해 line-of-sight 기준으로 분해
        bool lateral = false;
        if (otherId >= 0 &&
            TryGetPos(db, otherId, s.globalFrame, out var otherStart))
        {
            Vector3 myStart = s.worldCurrent;
            Vector3 myClear = c.worldCurrent;

            Vector3 los = otherStart - myStart;
            los.y = 0f;
            if (los.sqrMagnitude > 1e-6f)
            {
                Vector3 losN = los.normalized;
                Vector3 disp = myClear - myStart;
                disp.y = 0f;

                float forward = Vector3.Dot(disp, losN);
                Vector3 lat = disp - losN * forward;
                float latMag = lat.magnitude;
                float dispMag = disp.magnitude;

                if (latMag >= lateralMetersThr &&
                    (dispMag > 1e-6f && (latMag / dispMag) >= lateralRatioThr))
                    lateral = true;
            }
        }

        var list = new List<string>(3);
        if (turn) list.Add("Turn");
        if (stop || brake) list.Add("BrakeOrStop");
        if (lateral) list.Add("Lateral");

        if (list.Count == 0)
            list.Add("Resolved_NoEvasion");

        return list.ToArray();
    }

    void WriteCollisionLabel(FrameRec fr, string state, int otherId, float tMin, float minDist, float nowDist, string[] avoidTypes)
    {
        var labels = EnsureObj(fr.json, "labels");
        var col = EnsureObj(labels, "collisionBehavior");

        col["state"] = state;
        col["otherId"] = otherId;
        col["timeToMinDistSec"] = Round6(tMin);
        col["minDist"] = Round6(minDist);
        col["nowDist"] = Round6(nowDist);

        var arr = new JArray();
        if (avoidTypes != null)
            foreach (var s in avoidTypes) arr.Add(s);
        col["avoidTypes"] = arr;
    }

    // ----------------------------
    // Loitering / wandering / repeating (agent-level)
    // ----------------------------
    Dictionary<int, AgentSummary> ComputeAgentSummaries(AgentDB db)
    {
        var result = new Dictionary<int, AgentSummary>(db.byAgent.Count);

        foreach (var kv in db.byAgent)
        {
            int id = kv.Key;
            var frames = kv.Value;
            if (frames.Count < 2)
            {
                result[id] = new AgentSummary { isLoiter = false, loiterType = "None" };
                continue;
            }

            float totalDist = 0f;
            float duration = 0f;

            var positions = new List<Vector3>(frames.Count);
            positions.Add(frames[0].worldCurrent);

            for (int i = 1; i < frames.Count; i++)
            {
                int df = frames[i].globalFrame - frames[i - 1].globalFrame;
                if (df <= 0) continue; // loop/끊김 무시

                float dt = df / Mathf.Max(1e-6f, datasetFps);
                duration += dt;

                float d = Vector3.Distance(frames[i - 1].worldCurrent, frames[i].worldCurrent);
                totalDist += d;

                positions.Add(frames[i].worldCurrent);
            }

            Vector3 start = positions.First();
            Vector3 end = positions.Last();
            float netDist = Vector3.Distance(start, end);
            float ratio = (totalDist > 1e-6f) ? (netDist / totalDist) : 1f;

            // centroid + radius
            Vector3 centroid = Vector3.zero;
            foreach (var p in positions) centroid += p;
            centroid /= Mathf.Max(1, positions.Count);

            float radius = 0f;
            foreach (var p in positions)
                radius = Mathf.Max(radius, Vector3.Distance(p, centroid));

            // revisit rate (grid hashing)
            int revisits = 0;
            var visited = new HashSet<long>();
            long prevKey = long.MinValue;
            foreach (var p in positions)
            {
                long key = GridKey(p, gridCell);
                if (key == prevKey) continue;

                if (visited.Contains(key)) revisits++;
                else visited.Add(key);

                prevKey = key;
            }
            float revisitRate = (positions.Count > 1) ? (revisits / (float)(positions.Count - 1)) : 0f;

            bool smallArea = duration >= loiterMinDurationSec && radius <= loiterMaxRadius;
            bool wandering = duration >= loiterMinDurationSec && totalDist >= 1.0f && ratio <= wanderRatioThr;
            bool repeating = duration >= loiterMinDurationSec && revisitRate >= revisitRateThr;

            bool isLoiter = smallArea || wandering || repeating;
            string type = "None";
            if (smallArea) type = "SmallArea";
            else if (wandering) type = "Wandering";
            else if (repeating) type = "Repeating";

            result[id] = new AgentSummary
            {
                isLoiter = isLoiter,
                loiterType = type,
                durationSec = duration,
                totalDist = totalDist,
                netDist = netDist,
                netToTotalRatio = ratio,
                radius = radius,
                revisitRate = revisitRate
            };
        }

        return result;
    }

    void ApplyLoiteringLabelToFrames(List<FrameRec> frames, AgentSummary s)
    {
        foreach (var fr in frames)
        {
            var labels = EnsureObj(fr.json, "labels");
            var lo = EnsureObj(labels, "loitering");

            lo["isLoiter"] = s.isLoiter ? 1 : 0;
            lo["type"] = s.loiterType;
            lo["durationSec"] = Round6(s.durationSec);
            lo["totalDist"] = Round6(s.totalDist);
            lo["netDist"] = Round6(s.netDist);
            lo["netToTotalRatio"] = Round6(s.netToTotalRatio);
            lo["radius"] = Round6(s.radius);
            lo["revisitRate"] = Round6(s.revisitRate);
        }
    }

    // ----------------------------
    // Utilities
    // ----------------------------
    string ResolveRunFolder()
    {
        if (!string.IsNullOrWhiteSpace(runFolderPath))
            return runFolderPath;

        string baseDir = Application.persistentDataPath;
        if (!Directory.Exists(baseDir)) return "";

        var dirs = Directory.GetDirectories(baseDir, runName + "_*", SearchOption.TopDirectoryOnly);
        if (dirs.Length == 0) return "";

        // latest by last write time
        string latest = dirs
            .Select(d => new { dir = d, t = Directory.GetLastWriteTimeUtc(d) })
            .OrderByDescending(x => x.t)
            .First().dir;
        return latest;
    }

    static Vector3 ReadVec3(JObject jo, string key)
    {
        var arr = jo[key] as JArray;
        if (arr == null || arr.Count < 3) return Vector3.zero;
        return new Vector3(
            (float)arr[0],
            (float)arr[1],
            (float)arr[2]
        );
    }

    static JObject EnsureObj(JObject parent, string key)
    {
        if (parent[key] is JObject o) return o;
        var n = new JObject();
        parent[key] = n;
        return n;
    }

    static double Round6(double v) => Math.Round(v, 6, MidpointRounding.AwayFromZero);

    static float AngleOnXZ(Vector3 a, Vector3 b)
    {
        a.y = 0f; b.y = 0f;
        if (a.sqrMagnitude < 1e-6f || b.sqrMagnitude < 1e-6f) return 0f;
        return Vector3.Angle(a, b);
    }

    static bool TryGetPos(AgentDB db, int agentId, int globalFrame, out Vector3 pos)
    {
        pos = default;
        if (!db.byAgentFrame.TryGetValue(agentId, out var map)) return false;
        if (!map.TryGetValue(globalFrame, out var fr)) return false;
        pos = fr.worldCurrent;
        return true;
    }

    static long GridKey(Vector3 p, float cell)
    {
        int gx = Mathf.RoundToInt(p.x / Mathf.Max(1e-6f, cell));
        int gz = Mathf.RoundToInt(p.z / Mathf.Max(1e-6f, cell));
        // pack two 32-bit ints into 64-bit
        return ((long)gx << 32) ^ (uint)gz;
    }
}
