using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading;

namespace AssettoServer.Server.Ai.Splines;

public class JunctionEvaluator
{
    private readonly ConcurrentDictionary<int, bool>? _evaluated;
    private readonly AiSpline? _spline;
    private readonly Func<int, float> _getProbability;
    private IReadOnlyDictionary<int, bool>? _explicitDecisions;

    public JunctionEvaluator(AiSpline spline, bool savesState = true)
    {
        _spline = spline;
        _getProbability = junctionId => spline.Junctions[junctionId].Probability;

        if (savesState)
            _evaluated = new ConcurrentDictionary<int, bool>();
    }

    internal JunctionEvaluator(Func<int, float> getProbability, bool savesState = true)
    {
        _getProbability = getProbability;

        if (savesState)
            _evaluated = new ConcurrentDictionary<int, bool>();
    }

    public void Clear()
    {
        _evaluated?.Clear();
    }

    public void SetExplicitDecisions(IReadOnlyDictionary<int, bool>? decisions)
    {
        Volatile.Write(ref _explicitDecisions,
            decisions == null ? null : new Dictionary<int, bool>(decisions));
    }

    public bool WillTakeJunction(int junctionId)
    {
        var explicitDecisions = Volatile.Read(ref _explicitDecisions);
        if (explicitDecisions != null
            && explicitDecisions.TryGetValue(junctionId, out var explicitDecision))
        {
            return explicitDecision;
        }

        bool result = Random.Shared.NextDouble() < _getProbability(junctionId);
        return _evaluated?.GetOrAdd(junctionId, result) ?? result;
    }

    public int Traverse(int pointId, int count = 1)
    {
        return count > 0 ? Next(pointId, count) : Previous(pointId, -count);
    }

    public int Next(int pointId, int count = 1)
    {
        var spline = _spline ?? throw new InvalidOperationException("Spline traversal is unavailable");
        var points = spline.Points;
        var junctions = spline.Junctions;

        for (int i = 0; i < count && pointId >= 0; i++)
        {
            ref readonly var point = ref points[pointId];
            if (point.JunctionStartId >= 0)
            {
                var junctionId = point.JunctionStartId;
                bool result = WillTakeJunction(junctionId);
                pointId = result ? junctions[junctionId].EndPointId : point.NextId;
            }
            else
            {
                pointId = point.NextId;
            }
        }

        return pointId;
    }

    public bool TryNext(int pointId, out int nextPointId, int count = 1)
    {
        nextPointId = Next(pointId, count);
        return nextPointId >= 0;
    }

    public int Previous(int pointId, int count = 1)
    {
        var spline = _spline ?? throw new InvalidOperationException("Spline traversal is unavailable");
        var points = spline.Points;
        var junctions = spline.Junctions;

        for (int i = 0; i < count && pointId >= 0; i++)
        {
            ref readonly var point = ref points[pointId];
            if (point.JunctionEndId >= 0)
            {
                var junctionId = point.JunctionEndId;
                ref readonly var junction = ref junctions[junctionId];
                bool result = false;
                if (_evaluated != null && _evaluated.TryGetValue(junctionId, out var value))
                {
                    result = value;
                }
                else if (point.PreviousId < 0)
                {
                    result = Random.Shared.NextDouble() < junction.Probability;
                    _evaluated?.TryAdd(junctionId, result);
                }

                pointId = result ? junction.StartPointId : point.PreviousId;
            }
            else
            {
                pointId = point.PreviousId;
            }
        }

        return pointId;
    }

    public bool TryPrevious(int pointId, out int nextPointId, int count = 1)
    {
        nextPointId = Previous(pointId, count);
        return nextPointId >= 0;
    }
}
