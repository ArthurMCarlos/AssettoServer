using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;
using AssettoServer.Server.Ai.Splines;

namespace AssettoServer.Server.Ai.Routing;

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
        float maxDistanceMeters) =>
        TryPlan(
            startPointId,
            targetPointIds,
            new AiRouteSearchLimits(maxDistanceMeters, int.MaxValue)).Plan;

    public AiRouteSearchResult TryPlan(
        int startPointId,
        IReadOnlySet<int> targetPointIds,
        AiRouteSearchLimits limits)
    {
        ArgumentNullException.ThrowIfNull(targetPointIds);
        ArgumentNullException.ThrowIfNull(limits);
        if (targetPointIds.Count == 0
            || !float.IsFinite(limits.MaxDistanceMeters)
            || limits.MaxDistanceMeters <= 0
            || limits.MaxVisitedNodes <= 0)
        {
            return new AiRouteSearchResult(null, AiRouteSearchFailure.InvalidRequest, 0);
        }

        var graph = _graph.Value;
        if (!graph.ContainsPoint(startPointId))
            return new AiRouteSearchResult(null, AiRouteSearchFailure.InvalidRequest, 0);

        if (targetPointIds.Contains(startPointId))
        {
            return new AiRouteSearchResult(
                new AiRoutePlan(
                    0,
                    [new AiRouteNode(startPointId, 0)],
                    new Dictionary<int, bool>()),
                AiRouteSearchFailure.None,
                1);
        }

        var distances = new Dictionary<int, float> { [startPointId] = 0 };
        var predecessors = new Dictionary<int, (int PreviousPointId, AiRouteEdge Edge)>();
        var queue = new PriorityQueue<int, float>();
        queue.Enqueue(startPointId, 0);
        var visitedNodes = 0;
        var distanceLimited = false;

        while (queue.TryDequeue(out var pointId, out var queuedDistance))
        {
            if (!distances.TryGetValue(pointId, out var knownDistance)
                || queuedDistance > knownDistance)
            {
                continue;
            }

            if (visitedNodes >= limits.MaxVisitedNodes)
            {
                return new AiRouteSearchResult(
                    null,
                    AiRouteSearchFailure.NodeLimit,
                    visitedNodes);
            }

            visitedNodes++;

            if (targetPointIds.Contains(pointId))
            {
                return new AiRouteSearchResult(
                    Reconstruct(
                        startPointId,
                        pointId,
                        knownDistance,
                        predecessors,
                        distances),
                    AiRouteSearchFailure.None,
                    visitedNodes);
            }

            foreach (var edge in graph.GetEdges(pointId))
            {
                if (edge.LengthMeters < 0 || !float.IsFinite(edge.LengthMeters))
                    continue;

                var candidateDistance = knownDistance + edge.LengthMeters;
                if (candidateDistance > limits.MaxDistanceMeters)
                {
                    distanceLimited = true;
                    continue;
                }

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

        return new AiRouteSearchResult(
            null,
            distanceLimited
                ? AiRouteSearchFailure.DistanceLimit
                : AiRouteSearchFailure.Unreachable,
            visitedNodes);
    }

    private static AiRoutePlan Reconstruct(
        int startPointId,
        int targetPointId,
        float distanceMeters,
        IReadOnlyDictionary<int, (int PreviousPointId, AiRouteEdge Edge)> predecessors,
        IReadOnlyDictionary<int, float> distances)
    {
        var decisions = new Dictionary<int, bool>();
        var nodes = new List<AiRouteNode>();
        var pointId = targetPointId;
        nodes.Add(new AiRouteNode(pointId, distances[pointId]));
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
            nodes.Add(new AiRouteNode(pointId, distances[pointId]));
        }

        nodes.Reverse();
        return new AiRoutePlan(distanceMeters, nodes, decisions);
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
