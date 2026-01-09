using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

using Newtonsoft.Json.Linq;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class ZaraBehaviorGizmoDebug : MonoBehaviour
{
    [Header("Refs")]
    public ZaraGroupSimulator simulator;   // ped_*들이 이 아래에 있음

    [Header("Input (labeled output)")]
    [Tooltip("비워두면 Application.persistentDataPath에서 runName_* 중 최신 폴더를 자동 선택")]
    public string runFolderPath = "";
    public string runName = "zara_run";
    public string labeledSubFolder = "labeled"; // labeler에서 outputSubFolder
    public bool inputIsPerAgent = true;          // labeled/agents/agent_{id}.jsonl 사용

    [Header("Display")]
    public bool drawLabels = true;
    public float labelHeight = 2.0f;
    public Vector3 labelOffset = new Vector3(0, 0, 0);
    public float maxWorldDistanceToDraw = 80f;   // 멀면 안 그림(가볍게)
    public int maxFrameLookback = 30;            // 해당 globalFrame이 없으면 최대 몇 프레임 뒤로 찾아볼지

    [Header("Colors (optional)")]
    public bool colorByCollision = true;
    public Color colorNone = Color.white;
    public Color colorRisk = new Color(1f, 0.35f, 0.35f, 1f);
    public Color colorAvoided = new Color(0.35f, 1f, 0.5f, 1f);

    // ---- internal ----
    sealed class LabelRec
    {
        public int globalFrame;
        public string motionState;
        public float speed;
        public float accel;

        public string groupState;
        public int groupIndex;
        public int othersCount;

        public string collisionState; // None/Risk/Avoided
        public int collisionOtherId;
        public float timeToMin;
        public float minDist;
        public float nowDist;
        public string[] avoidTypes;

        public int isLoiter;
        public string loiterType;
    }

    // agentId -> (globalFrame -> LabelRec) (중복 frame은 마지막이 덮어씀)
    readonly Dictionary<int, Dictionary<int, LabelRec>> byAgentFrame = new();
    // agentId -> sorted unique frames
    readonly Dictionary<int, List<int>> framesByAgent = new();

    // runtime ped cache
    readonly Dictionary<int, Transform> pedById = new();
    float nextRefreshTime = 0f;

    void Awake()
    {
        if (simulator == null) simulator = FindFirstObjectByType<ZaraGroupSimulator>();
        Reload();
    }

    [ContextMenu("Reload JSON")]
    public void Reload()
    {
        byAgentFrame.Clear();
        framesByAgent.Clear();

        string run = ResolveRunFolder();
        if (string.IsNullOrEmpty(run))
        {
            Debug.LogError("[ZaraBehaviorGizmoDebug] Run folder not found.");
            return;
        }

        string baseDir = Path.Combine(run, labeledSubFolder);
        string agentsDir = inputIsPerAgent ? Path.Combine(baseDir, "agents") : baseDir;

        if (!Directory.Exists(agentsDir))
        {
            Debug.LogError($"[ZaraBehaviorGizmoDebug] Labeled agents directory not found: {agentsDir}");
            return;
        }

        string[] files = inputIsPerAgent
            ? Directory.GetFiles(agentsDir, "agent_*.jsonl", SearchOption.TopDirectoryOnly)
            : new[] { Path.Combine(agentsDir, "frames.jsonl") };

        int lines = 0;
        foreach (var path in files)
        {
            if (!File.Exists(path)) continue;

            foreach (var line in File.ReadLines(path))
            {
                if (string.IsNullOrWhiteSpace(line)) continue;

                JObject jo;
                try { jo = JObject.Parse(line); }
                catch { continue; }

                int agentId = jo.Value<int?>("agentId") ?? -1;
                int gFrame = jo.Value<int?>("globalFrame") ?? int.MinValue;
                if (agentId < 0 || gFrame == int.MinValue) continue;

                var labels = jo["labels"] as JObject;
                if (labels == null) continue;

                var rec = new LabelRec { globalFrame = gFrame };

                // motion
                var motion = labels["motion"] as JObject;
                rec.motionState = motion?.Value<string>("state") ?? "Unknown";
                rec.speed = motion?.Value<float?>("speed") ?? float.NaN;
                rec.accel = motion?.Value<float?>("accel") ?? float.NaN;

                // group
                var gb = labels["groupBehavior"] as JObject;
                rec.groupState = gb?.Value<string>("state") ?? "Unknown";
                rec.groupIndex = gb?.Value<int?>("groupIndex") ?? -1;
                rec.othersCount = gb?.Value<int?>("othersCount") ?? 0;

                // collision
                var cb = labels["collisionBehavior"] as JObject;
                rec.collisionState = cb?.Value<string>("state") ?? "None";
                rec.collisionOtherId = cb?.Value<int?>("otherId") ?? -1;
                rec.timeToMin = cb?.Value<float?>("timeToMinDistSec") ?? -1f;
                rec.minDist = cb?.Value<float?>("minDist") ?? -1f;
                rec.nowDist = cb?.Value<float?>("nowDist") ?? -1f;

                var avoidArr = cb?["avoidTypes"] as JArray;
                if (avoidArr != null)
                {
                    rec.avoidTypes = new string[avoidArr.Count];
                    for (int i = 0; i < avoidArr.Count; i++)
                        rec.avoidTypes[i] = (string)avoidArr[i];
                }
                else rec.avoidTypes = Array.Empty<string>();

                // loiter
                var lo = labels["loitering"] as JObject;
                rec.isLoiter = lo?.Value<int?>("isLoiter") ?? 0;
                rec.loiterType = lo?.Value<string>("type") ?? "None";

                if (!byAgentFrame.TryGetValue(agentId, out var map))
                {
                    map = new Dictionary<int, LabelRec>(4096);
                    byAgentFrame[agentId] = map;
                    framesByAgent[agentId] = new List<int>(4096);
                }

                // overwrite duplicates with latest
                bool existed = map.ContainsKey(gFrame);
                map[gFrame] = rec;
                if (!existed) framesByAgent[agentId].Add(gFrame);

                lines++;
            }
        }

        // sort frames list
        foreach (var kv in framesByAgent)
            kv.Value.Sort();

        Debug.Log($"[ZaraBehaviorGizmoDebug] Loaded labeled json. agents={byAgentFrame.Count}, lines={lines}");
    }

    void Update()
    {
        // ped 캐시 주기적 갱신(생성/파괴 대비)
        if (Time.time >= nextRefreshTime)
        {
            RefreshPedCache();
            nextRefreshTime = Time.time + 0.5f;
        }
    }

    void RefreshPedCache()
    {
        pedById.Clear();
        if (simulator == null) return;

        foreach (Transform child in simulator.transform)
        {
            if (child == null) continue;
            var go = child.gameObject;
            if (go == null || !go.name.StartsWith("ped_", StringComparison.Ordinal)) continue;

            int idx = go.name.IndexOf('_');
            if (idx < 0) continue;
            if (!int.TryParse(go.name[(idx + 1)..], NumberStyles.Integer, CultureInfo.InvariantCulture, out int id))
                continue;

            pedById[id] = child;
        }
    }

#if UNITY_EDITOR
    void OnDrawGizmos()
    {
        if (!drawLabels) return;
        if (!Application.isPlaying) return;
        if (simulator == null) return;

        int gFrame = simulator.GlobalFrame;

        // 카메라 기준 너무 멀면 안 그리기
        var sceneCam = SceneView.lastActiveSceneView ? SceneView.lastActiveSceneView.camera : null;
        Vector3 camPos = sceneCam ? sceneCam.transform.position : Vector3.zero;

        foreach (var kv in pedById)
        {
            int agentId = kv.Key;
            var t = kv.Value;
            if (t == null) continue;
            if (!t.gameObject.activeInHierarchy) continue;

            if (sceneCam && Vector3.Distance(camPos, t.position) > maxWorldDistanceToDraw) continue;

            if (!TryGetLabel(agentId, gFrame, out var rec)) continue;

            string txt = BuildLabelText(agentId, gFrame, rec);

            var style = new GUIStyle(EditorStyles.boldLabel);
            style.fontSize = 11;

            if (colorByCollision)
            {
                style.normal.textColor = rec.collisionState switch
                {
                    "Risk" => colorRisk,
                    "Avoided" => colorAvoided,
                    _ => colorNone
                };
            }
            else
            {
                style.normal.textColor = Color.white;
            }

            Vector3 p = t.position + Vector3.up * labelHeight + labelOffset;
            Handles.Label(p, txt, style);

            // 선택적으로 작은 표시도
            Gizmos.color = style.normal.textColor;
            Gizmos.DrawSphere(t.position + Vector3.up * 0.1f, 0.05f);
        }
    }
#endif

    bool TryGetLabel(int agentId, int targetFrame, out LabelRec rec)
    {
        rec = null;
        if (!byAgentFrame.TryGetValue(agentId, out var map)) return false;
        if (!framesByAgent.TryGetValue(agentId, out var frames) || frames.Count == 0) return false;

        // exact first
        if (map.TryGetValue(targetFrame, out rec)) return true;

        // find nearest <= target using binary search
        int idx = frames.BinarySearch(targetFrame);
        if (idx >= 0)
        {
            rec = map[frames[idx]];
            return true;
        }

        idx = ~idx - 1; // predecessor
        if (idx < 0) return false;

        int foundFrame = frames[idx];
        if ((targetFrame - foundFrame) > maxFrameLookback) return false;

        rec = map[foundFrame];
        return true;
    }

    string BuildLabelText(int agentId, int gFrame, LabelRec r)
    {
        string sp = float.IsNaN(r.speed) ? "?" : r.speed.ToString("0.###", CultureInfo.InvariantCulture);
        string ac = float.IsNaN(r.accel) ? "?" : r.accel.ToString("0.###", CultureInfo.InvariantCulture);

        string avoid = (r.avoidTypes != null && r.avoidTypes.Length > 0)
            ? string.Join(",", r.avoidTypes)
            : "-";

        string lo = (r.isLoiter == 1) ? $"Loiter:{r.loiterType}" : "Loiter:None";

        string col = r.collisionState switch
        {
            "Risk" => $"C:Risk -> {r.collisionOtherId}  t:{r.timeToMin:0.##}  now:{r.nowDist:0.##}  min:{r.minDist:0.##}",
            "Avoided" => $"C:Avoided -> {r.collisionOtherId}  avoid:{avoid}",
            _ => "C:None"
        };

        return
            $"#{agentId}  f:{gFrame}\n" +
            $"M:{r.motionState}  v:{sp}  a:{ac}\n" +
            $"G:{r.groupState} (idx:{r.groupIndex}, others:{r.othersCount})\n" +
            $"{col}\n" +
            $"{lo}";
    }

    string ResolveRunFolder()
    {
        if (!string.IsNullOrWhiteSpace(runFolderPath))
            return runFolderPath;

        string baseDir = Application.persistentDataPath;
        if (!Directory.Exists(baseDir)) return "";

        var dirs = Directory.GetDirectories(baseDir, runName + "_*", SearchOption.TopDirectoryOnly);
        if (dirs.Length == 0) return "";

        string latest = dirs[0];
        DateTime best = Directory.GetLastWriteTimeUtc(latest);

        for (int i = 1; i < dirs.Length; i++)
        {
            var t = Directory.GetLastWriteTimeUtc(dirs[i]);
            if (t > best) { best = t; latest = dirs[i]; }
        }
        return latest;
    }
}
