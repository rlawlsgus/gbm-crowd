using UnityEngine;
using UnityEngine.Splines;
using System.Collections.Generic;

public class CarPoolManager : MonoBehaviour
{
    [Header("Settings")]
    public SplineContainer splineContainer;
    public int splineIndex = 0;
    public float checkRadius = 5f;
    public LayerMask carLayer;

    [Header("Pooling Settings")]
    [Tooltip("다양한 자동차 스킨 프리팹들을 등록하세요.")]
    public List<GameObject> carPrefabs;
    public int poolSizePerPrefab = 5;

    [Header("Bus Settings")]
    public List<GameObject> busPrefabs;
    public int busSplineIndex = 0;
    public int busPoolSize = 3;

    [Header("Safety Settings")]
    public float minSpawnCooldown = 2.0f;
    public float maxSpawnCooldown = 20.0f;
    private float currentCooldown = 2.0f; 
    private float lastSpawnTime = -100f;

    private List<CarController> carPool = new List<CarController>();
    private Queue<CarController> waitingCars = new Queue<CarController>();
    private Queue<CarController> waitingBuses = new Queue<CarController>();

    private void Start()
    {
        if (splineContainer == null)
        {
            Debug.LogError("[CarPoolManager] SplineContainer is not assigned!");
            return;
        }

        // 일반 자동차 풀 생성
        foreach (var prefab in carPrefabs)
        {
            if (prefab == null) continue;
            for (int i = 0; i < poolSizePerPrefab; i++)
                CreateAndEnqueue(prefab, waitingCars);
        }

        // 버스 전용 풀 생성
        foreach (var prefab in busPrefabs)
        {
            if (prefab == null) continue;
            for (int i = 0; i < busPoolSize; i++)
                CreateAndEnqueue(prefab, waitingBuses);
        }
        
        // 대기 큐 섞기
        waitingCars = ShuffleQueue(waitingCars);
        waitingBuses = ShuffleQueue(waitingBuses);
    }

    void CreateAndEnqueue(GameObject prefab, Queue<CarController> queue)
    {
        // 1. 생성 시 부모 객체의 위치에 관계없이 멀리 떨어진 곳에서 생성 (겹침 방지)
        GameObject go = Instantiate(prefab, new Vector3(0, -100, 0), Quaternion.identity, transform);
        CarController car = go.GetComponent<CarController>();
        if (car != null)
        {
            // 2. 레이어 강제 설정 (CanSpawn 감지용)
            // carLayer가 단일 레이어인 경우 그 레이어로, 마스크인 경우 가장 낮은 비트의 레이어로 설정
            int layer = 0;
            int layerValue = carLayer.value;
            while (layerValue > 1) { layerValue >>= 1; layer++; }
            go.layer = (layerValue > 0) ? layer : go.layer;

            car.gameObject.SetActive(false);
            car.OnReachEnd = () => ReturnToPool(car);
            carPool.Add(car);
            queue.Enqueue(car);
        }
    }

    private Queue<CarController> ShuffleQueue(Queue<CarController> q)
    {
        List<CarController> temp = new List<CarController>(q);
        q.Clear();
        while (temp.Count > 0)
        {
            int index = Random.Range(0, temp.Count);
            q.Enqueue(temp[index]);
            temp.RemoveAt(index);
        }
        return q;
    }

    private void Update()
    {
        // 1. 랜덤 쿨타임 체크 (물리 감지 지연 보완용)
        if (Time.time < lastSpawnTime + currentCooldown) return;

        // 2. 물리적 공간 체크
        if (CanSpawn())
        {
            if (splineIndex == busSplineIndex)
            {
                if (waitingBuses.Count > 0 || waitingCars.Count > 0)
                {
                    bool spawnBus = waitingBuses.Count > 0 && Random.value < 0.3f;
                    ActivateCar(spawnBus ? waitingBuses : waitingCars);
                    lastSpawnTime = Time.time; 
                    currentCooldown = Random.Range(minSpawnCooldown, maxSpawnCooldown); // 다음 쿨타임 랜덤 결정
                }
            }
            else if (waitingCars.Count > 0)
            {
                ActivateCar(waitingCars);
                lastSpawnTime = Time.time; 
                currentCooldown = Random.Range(minSpawnCooldown, maxSpawnCooldown); // 다음 쿨타임 랜덤 결정
            }
        }
    }

    bool CanSpawn()
    {
        if (splineContainer == null || splineContainer.Splines.Count <= splineIndex) return false;
        
        // Spline의 첫 번째 매듭(Knot) 위치를 월드 좌표로 직접 가져오기
        var spline = splineContainer.Splines[splineIndex];
        Vector3 spawnWorldPos = splineContainer.transform.TransformPoint((Vector3)spline[0].Position);
        
        // 주변에 Car Layer 물체가 있는지 확인
        Collider[] colliders = Physics.OverlapSphere(spawnWorldPos, checkRadius, carLayer);
        return colliders.Length == 0;
    }

    void ActivateCar(Queue<CarController> queue)
    {
        if (queue.Count == 0) return;

        CarController car = queue.Dequeue();
        
        car.splineContainer = splineContainer;
        car.splineIndex = splineIndex;
        car.ResetPath();
        
        // 월드 좌표 및 탄젠트 계산
        var spline = splineContainer.Splines[splineIndex];
        Unity.Mathematics.float3 pos, tangent, up;
        spline.Evaluate(0f, out pos, out tangent, out up);
        
        Vector3 spawnWorldPos = splineContainer.transform.TransformPoint((Vector3)pos);
        Vector3 worldTangent = splineContainer.transform.TransformDirection((Vector3)tangent);
        Vector3 worldUp = splineContainer.transform.TransformDirection((Vector3)up);

        car.transform.position = spawnWorldPos;
        if (worldTangent != Vector3.zero)
        {
            car.transform.rotation = Quaternion.LookRotation(worldTangent, worldUp);
        }
        
        car.gameObject.SetActive(true);
    }

    void ReturnToPool(CarController car)
    {
        car.gameObject.SetActive(false);
        // 비활성화 시 즉시 멀리 치워서 감지에서 제외
        car.transform.position = new Vector3(0, -100, 0);
        
        if (car.vehicleType == VehicleType.Bus) waitingBuses.Enqueue(car);
        else waitingCars.Enqueue(car);
    }

    private void OnDrawGizmos()
    {
        if (splineContainer != null && splineContainer.Splines.Count > splineIndex)
        {
            Gizmos.color = Color.cyan;
            var spline = splineContainer.Splines[splineIndex];
            Vector3 spawnPos = splineContainer.transform.TransformPoint((Vector3)spline[0].Position);
            Gizmos.DrawWireSphere(spawnPos, checkRadius);
        }
    }
}
