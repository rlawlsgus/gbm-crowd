using System.Collections.Generic;

public static class ZaraCollisionBus
{
    public struct CollisionResult
    {
        public bool ahead;
        public int otherId;
        public float timeToMinDistSec;
        public float minDist;
        public float nowDist;
    }

    static int _frame = int.MinValue;
    static readonly Dictionary<int, CollisionResult> _byAgent = new Dictionary<int, CollisionResult>(256);

    public static void Publish(int globalFrame, Dictionary<int, CollisionResult> results)
    {
        _frame = globalFrame;
        _byAgent.Clear();
        foreach (var kv in results)
            _byAgent[kv.Key] = kv.Value;
    }

    public static bool TryGet(int globalFrame, int agentId, out CollisionResult r)
    {
        if (globalFrame != _frame)
        {
            r = default;
            return false;
        }
        return _byAgent.TryGetValue(agentId, out r);
    }

    public static int LastPublishedFrame => _frame;
}
