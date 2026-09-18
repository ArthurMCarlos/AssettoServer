using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;
using AssettoServer.Server.Ai.Splines;

namespace AssettoServer.Server.Ai.Routing;

public sealed record AiRoutePlan(
    float DistanceMeters,
    IReadOnlyDictionary<int, bool> JunctionDecisions);

public sealed class AiRoutePlanner
{
    private readonly Lazy<AiRouteGraph> _graph;

    public AiRoutePlanner(AiSpline spline) : this(() => CreateGraph(spline))
    {
    }

    internal AiRoutePlanner(AiRouteGraph graph) : this(() => graph)
    {
    }

    internal AiRoutePlanner(Func<AiRouteGraph> graphFactory)
    {
        _graph = new Lazy<AiRouteGraph>(
            graphFactory,
            LazyThreadSafetyMode.ExecutionAndPublication);
    }

    public AiRoutePlan? TryPlan(
        int startPointId,
        IReadOnlySet<int> targetPointIds,
        float maxDistanceMeters)
    {
        ArgumentNullException.ThrowIfNull(targetPointIds);
        var graph = _graph.Value;
        if (!graph.ContainsPoint(startPointId)
            || targetPointIds.Count == 0
            || !float.IsFinite(maxDistanceMeters)
            || maxDistanceMeters < 0)
        {
            return null;
        }

        if (targetPointIds.Contains(startPointId))
            return new AiRoutePlan(0, new Dictionary<int, bool>());

        var distances = new Dictionary<int, float> { [startPointId] = 0 };
        var predecessors = new Dictionary<int, (int PreviousPointId, AiRouteEdge Edge)>();
        var queue = new PriorityQueue<int, float>();
        queue.Enqueue(startPointId, 0);

        while (queue.TryDequeue(out var pointId, out var queuedDistance))
        {
            if (!distances.TryGetValue(pointId, out var knownDistance)
                || queuedDistance > knownDistance)
            {
                continue;
            }

            if (targetPointIds.Contains(pointId))
                return Reconstruct(startPointId, pointId, knownDistance, predecessors);

            foreach (var edge in graph.GetEdges(pointId))
            {
                if (edge.LengthMeters < 0 || !float.IsFinite(edge.LengthMeters))
                    continue;

                var candidateDistance = knownDistance + edge.LengthMeters;
                if (candidateDistance > maxDistanceMeters)
                    continue;

                if (distances.TryGetValue(edge.ToPointId, out var previousDistance)
                    && previousDistance <= candidateDistance)
                {
                    continue;
                }

                distances[edge.ToPointId] = candidateDistance;
                predecessors[edge.ToPointId] = (pointId, edge);
                queue.Enqueue(edge.ToPointId, candidateDistance);
            }
        }

        return null;
    }

    private static AiRoutePlan Reconstruct(
        int startPointId,
        int targetPointId,
        float distanceMeters,
        IReadOnlyDictionary<int, (int PreviousPointId, AiRouteEdge Edge)> predecessors)
    {
        var decisions = new Dictionary<int, bool>();
        var pointId = targetPointId;
        while (pointId != startPointId)
        {
            var predecessor = predecessors[pointId];
            if (predecessor.Edge.JunctionId.HasValue
                && predecessor.Edge.TakeBranch.HasValue)
            {
                decisions[predecessor.Edge.JunctionId.Value] =
                    predecessor.Edge.TakeBranch.Value;
            }

            pointId = predecessor.PreviousPointId;
        }

        return new AiRoutePlan(distanceMeters, decisions);
    }

    private static AiRouteGraph CreateGraph(AiSpline spline)
    {
        var points = spline.Points;
        var junctions = spline.Junctions;
        var edges = new Dictionary<int, AiRouteEdge[]>(points.Length);

        for (var i = 0; i < points.Length; i++)
        {
            ref readonly var point = ref points[i];
            var outgoing = new List<AiRouteEdge>(2);

            if (point.NextId >= 0)
            {
                outgoing.Add(new AiRouteEdge(
                    point.NextId,
                    Vector3.Distance(point.Position, points[point.NextId].Position),
                    point.JunctionStartId >= 0 ? point.JunctionStartId : null,
                    point.JunctionStartId >= 0 ? false : null));
            }

            if (point.JunctionStartId >= 0)
            {
                ref readonly var junction = ref junctions[point.JunctionStartId];
                if (junction.EndPointId >= 0 && junction.EndPointId != point.NextId)
                {
                    outgoing.Add(new AiRouteEdge(
                        junction.EndPointId,
                        Vector3.Distance(point.Position, points[junction.EndPointId].Position),
                        point.JunctionStartId,
                        true));
                }
            }

            edges[point.Id] = outgoing.ToArray();
        }

        return new AiRouteGraph(edges);
    }
}
