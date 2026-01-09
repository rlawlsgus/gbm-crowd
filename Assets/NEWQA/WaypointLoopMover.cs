using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

// 파일 1개로 끝: 인스펙터에서 Vector3로 조절 + Scene에서 점 드래그(핸들) + 1->2->3->4 반복 이동
public class WaypointLoopMoverVector3 : MonoBehaviour
{
    [Header("Waypoints (Vector3)")]
    [Tooltip("체크하면 이 오브젝트 기준 Local 좌표로 저장/이동, 아니면 World 좌표")]
    [SerializeField] private bool useLocalSpace = false;

    [SerializeField] private Vector3[] waypoints = new Vector3[2];

    [Header("Move")]
    [SerializeField] private float speed = 3f;
    [SerializeField] private float reachDistance = 0.1f;

    [Header("Rotate (look at waypoint direction)")]
    [Tooltip("이동 방향(현재->타겟)으로 바라보게 할지")]
    [SerializeField] private bool faceMoveDirection = true;

    [Tooltip("회전 속도 (deg/sec). 0이면 즉시 회전")]
    [SerializeField] private float turnSpeedDeg = 720f;

    [Tooltip("수평(y축)만 회전할지")]
    [SerializeField] private bool yawOnly = true;

    private int currentIndex = 0;

    public bool UseLocalSpace => useLocalSpace;
    public Vector3[] Waypoints => waypoints;

    private void OnValidate()
    {
        // 4개 고정
        if (waypoints == null) waypoints = new Vector3[2];
        if (waypoints.Length != 4) System.Array.Resize(ref waypoints, 2);
        if (reachDistance < 0f) reachDistance = 0f;
        if (speed < 0f) speed = 0f;

        if (turnSpeedDeg < 0f) turnSpeedDeg = 0f;
    }

    private void Update()
    {
        if (waypoints == null || waypoints.Length == 0) return;

        Vector3 target = GetWorldPoint(currentIndex);

        // 이동
        Vector3 prevPos = transform.position;
        transform.position = Vector3.MoveTowards(
            transform.position,
            target,
            speed * Time.deltaTime
        );

        // 회전: waypoint 방향(현재 위치 -> target) 바라보기
        if (faceMoveDirection)
            RotateToward(target);

        // 도착 체크
        if ((transform.position - target).sqrMagnitude <= reachDistance * reachDistance)
        {
            currentIndex = (currentIndex + 1) % waypoints.Length; // 0->1->2->3->0...
        }
    }

    private void RotateToward(Vector3 targetWorld)
    {
        Vector3 dir = targetWorld - transform.position;
        if (yawOnly)
            dir.y = 0f;

        if (dir.sqrMagnitude < 1e-8f) return;

        Quaternion targetRot = Quaternion.LookRotation(dir.normalized, Vector3.up);

        if (turnSpeedDeg <= 0f)
        {
            transform.rotation = targetRot; // 즉시
        }
        else
        {
            transform.rotation = Quaternion.RotateTowards(
                transform.rotation,
                targetRot,
                turnSpeedDeg * Time.deltaTime
            );
        }
    }

    private Vector3 GetWorldPoint(int i)
    {
        return useLocalSpace ? transform.TransformPoint(waypoints[i]) : waypoints[i];
    }

    private void OnDrawGizmos()
    {
        if (waypoints == null || waypoints.Length == 0) return;

        // 점 + 루프 라인
        for (int i = 0; i < waypoints.Length; i++)
        {
            Vector3 p = GetWorldPoint(i);
            Gizmos.DrawSphere(p, 0.12f);

            int next = (i + 1) % waypoints.Length;
            Vector3 q = GetWorldPoint(next);
            Gizmos.DrawLine(p, q);
        }
    }
}

#if UNITY_EDITOR
[CustomEditor(typeof(WaypointLoopMoverVector3))]
public class WaypointLoopMoverVector3Editor : Editor
{
    private SerializedProperty useLocalSpaceProp;
    private SerializedProperty waypointsProp;

    private void OnEnable()
    {
        useLocalSpaceProp = serializedObject.FindProperty("useLocalSpace");
        waypointsProp = serializedObject.FindProperty("waypoints");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        EditorGUILayout.PropertyField(useLocalSpaceProp);
        EditorGUILayout.PropertyField(waypointsProp, true);

        // 나머지 Rotate 옵션도 같이 노출(기본 Inspector가 아니므로 명시적으로)
        EditorGUILayout.Space(8);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("faceMoveDirection"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("turnSpeedDeg"));
        EditorGUILayout.PropertyField(serializedObject.FindProperty("yawOnly"));

        EditorGUILayout.Space(8);
        using (new EditorGUILayout.HorizontalScope())
        {
            if (GUILayout.Button("현재 위치로 1번(0) 설정"))
            {
                SetWaypointWorld(0, ((WaypointLoopMoverVector3)target).transform.position);
            }

            if (GUILayout.Button("현재 위치로 1~4 전부 덮어쓰기"))
            {
                var mover = (WaypointLoopMoverVector3)target;
                Vector3 pos = mover.transform.position;
                for (int i = 0; i < mover.Waypoints.Length; i++)
                    SetWaypointWorld(i, pos);
            }
        }

        serializedObject.ApplyModifiedProperties();
    }

    private void OnSceneGUI()
    {
        serializedObject.Update();

        var mover = (WaypointLoopMoverVector3)target;
        Transform t = mover.transform;
        bool local = useLocalSpaceProp.boolValue;

        if (waypointsProp == null || !waypointsProp.isArray) return;

        for (int i = 0; i < waypointsProp.arraySize; i++)
        {
            SerializedProperty wp = waypointsProp.GetArrayElementAtIndex(i);
            Vector3 stored = wp.vector3Value;
            Vector3 world = local ? t.TransformPoint(stored) : stored;

            Handles.Label(world + Vector3.up * 0.2f, $"{i + 1}");

            EditorGUI.BeginChangeCheck();
            Vector3 newWorld = Handles.PositionHandle(world, Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(mover, "Move Waypoint");
                wp.vector3Value = local ? t.InverseTransformPoint(newWorld) : newWorld;
                EditorUtility.SetDirty(mover);
            }
        }

        serializedObject.ApplyModifiedProperties();
    }

    private void SetWaypointWorld(int index, Vector3 worldPos)
    {
        serializedObject.Update();

        var mover = (WaypointLoopMoverVector3)target;
        Transform t = mover.transform;
        bool local = useLocalSpaceProp.boolValue;

        if (waypointsProp == null || !waypointsProp.isArray) return;
        if (index < 0 || index >= waypointsProp.arraySize) return;

        SerializedProperty wp = waypointsProp.GetArrayElementAtIndex(index);

        Undo.RecordObject(mover, "Set Waypoint");
        wp.vector3Value = local ? t.InverseTransformPoint(worldPos) : worldPos;
        EditorUtility.SetDirty(mover);

        serializedObject.ApplyModifiedProperties();
    }
}
#endif
