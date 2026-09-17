using System.Collections.Generic;

namespace AssettoServer.Server.Ai.Routing;

public readonly record struct AiRouteEdge(
    int ToPointId,
    float LengthMeters,
    int? JunctionId,
    bool? TakeBranch);

public sealed class AiRouteGraph
{
    private static readonly IReadOnlyList<AiRouteEdge> EmptyEdges = [];
    private readonly IReadOnlyDictionary<int, AiRouteEdge[]> _edges;

    public AiRouteGraph(IReadOnlyDictionary<int, AiRouteEdge[]> edges)
    {
        _edges = edges;
    }

    public bool ContainsPoint(int pointId) => _edges.ContainsKey(pointId);

    public IReadOnlyList<AiRouteEdge> GetEdges(int pointId) =>
        _edges.TryGetValue(pointId, out var edges) ? edges : EmptyEdges;
}
