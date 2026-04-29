using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEditor.SceneManagement;
using System.Collections.Generic;

public class MissingScriptRemover : EditorWindow
{
    [MenuItem("Tools/Missing Script Remover")]
    public static void ShowWindow()
    {
        GetWindow<MissingScriptRemover>("Script Remover");
    }

    private void OnGUI()
    {
        GUILayout.Label("Missing Script Remover", EditorStyles.boldLabel);
        EditorGUILayout.Space();

        if (GUILayout.Button("Remove from Current Scene (All Objects)", GUILayout.Height(30)))
        {
            RemoveFromScene();
        }

        EditorGUILayout.Space();

        if (GUILayout.Button("Remove from Selected Objects (Include Children)", GUILayout.Height(30)))
        {
            RemoveFromSelected();
        }

        EditorGUILayout.HelpBox("이 작업은 되돌릴 수 없으므로 실행 전 씬을 저장하는 것을 권장합니다.", MessageType.Info);
    }

    private static void RemoveFromScene()
    {
        GameObject[] allObjects = SceneManager.GetActiveScene().GetRootGameObjects();
        int count = 0;

        foreach (GameObject go in allObjects)
        {
            count += RemoveInGameObjectAndChildren(go);
        }

        Debug.Log($"[MissingScriptRemover] 씬 전체에서 {count}개의 Missing Script를 제거했습니다.");
        
        if (count > 0)
        {
            EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
        }
    }

    private static void RemoveFromSelected()
    {
        GameObject[] selectedObjects = Selection.gameObjects;
        int count = 0;

        foreach (GameObject go in selectedObjects)
        {
            count += RemoveInGameObjectAndChildren(go);
        }

        Debug.Log($"[MissingScriptRemover] 선택한 오브젝트들에서 {count}개의 Missing Script를 제거했습니다.");
        
        if (count > 0)
        {
            EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
        }
    }

    private static int RemoveInGameObjectAndChildren(GameObject go)
    {
        int count = GameObjectUtility.RemoveMonoBehavioursWithMissingScript(go);

        foreach (Transform child in go.transform)
        {
            count += RemoveInGameObjectAndChildren(child.gameObject);
        }

        return count;
    }
}
