using System.Collections.Generic;

namespace AssettoServer.Server.Ai.Routing;

public sealed record AiRouteSearchLimits(
    float MaxDistanceMeters,
    int MaxVisitedNodes);

public enum AiRouteSearchFailure
{
    None,
    InvalidRequest,
    DistanceLimit,
    NodeLimit,
    Unreachable
}

public sealed record AiRouteNode(
    int PointId,
    float DistanceFromStartMeters);

public sealed record AiRoutePlan(
    float DistanceMeters,
    IReadOnlyList<AiRouteNode> Nodes,
    IReadOnlyDictionary<int, bool> JunctionDecisions);

public sealed record AiRouteSearchResult(
    AiRoutePlan? Plan,
    AiRouteSearchFailure Failure,
    int VisitedNodes,
    float MaximumExploredDistanceMeters = 0,
    int JunctionEdgesExamined = 0);
