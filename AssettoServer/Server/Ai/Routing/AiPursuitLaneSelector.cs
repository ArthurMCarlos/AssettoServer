using System;
using System.Collections.Generic;
using System.Linq;
using AssettoServer.Server.Ai.Splines;

namespace AssettoServer.Server.Ai.Routing;

public enum AiLaneChangeDirection
{
    Left,
    Right
}

public enum AiPursuitLaneSelectionKind
{
    Stay,
    Change,
    Unreachable
}

public enum AiPursuitLaneRejectionReason
{
    MissingNeighbor,
    OppositeDirection,
    NoForwardRoute,
    NoRealJunction,
    BeyondLookahead,
    InsufficientPreparationDistance
}

public enum AiPursuitLaneEvaluationReason
{
    CurrentLaneValid,
    NoAdjacentLane,
    OppositeDirection,
    NoForwardRoute,
    NoRealJunction,
    BeyondLookahead,
    InsufficientPreparationDistance,
    RoutePreparation
}

public readonly record struct AiAdjacentLanePoints(int LeftPointId, int RightPointId);
public readonly record struct AiPursuitLaneDecision(int JunctionId, float DistanceMeters);

public sealed record AiPursuitLaneSelection(
    int FromPointId,
    int ToPointId,
    AiLaneChangeDirection Direction,
    AiRoutePlan DestinationPlan,
    float DistanceToDecisionMeters)
{
    public int JunctionId { get; init; }
}

public sealed record AiPursuitLaneCandidateDiagnostic(
    int? PointId,
    AiLaneChangeDirection Direction,
    AiPursuitLaneRejectionReason? Rejection);

public sealed record AiPursuitLaneRouteDiagnostic(
    int? PointId,
    AiLaneChangeDirection? Direction,
    AiRouteSearchFailure SearchFailure,
    float? RouteDistanceMeters,
    float MaximumExploredDistanceMeters,
    int JunctionEdgesExamined,
    int? JunctionId,
    float? DistanceToDecisionMeters,
    AiPursuitLaneEvaluationReason Reason);

public sealed record AiPursuitLaneSelectionResult(
    AiPursuitLaneSelectionKind Kind,
    AiPursuitLaneSelection? Selection,
    IReadOnlyList<AiPursuitLaneCandidateDiagnostic> Candidates)
{
    public AiPursuitLaneEvaluationReason Reason { get; init; }
    public AiPursuitLaneRouteDiagnostic CurrentLaneRoute { get; init; } = null!;
    public IReadOnlyList<AiPursuitLaneRouteDiagnostic> CandidateLaneRoutes { get; init; } = [];
}

public sealed class AiPursuitLaneSelector
{
    private readonly Func<int, AiAdjacentLanePoints> _getAdjacent;
    private readonly Func<int, int, bool> _isSameDirection;
    private readonly Func<int, IReadOnlySet<int>, AiRouteSearchLimits, AiRouteSearchResult> _tryPlan;
    private readonly Func<AiRoutePlan, AiPursuitLaneDecision?> _getDecision;

    public AiPursuitLaneSelector(AiSpline spline, AiRoutePlanner planner)
        : this(
            pointId => new AiAdjacentLanePoints(
                spline.Points[pointId].LeftId,
                spline.Points[pointId].RightId),
            (sourcePointId, targetPointId) =>
                spline.Operations.IsSameDirection(sourcePointId, targetPointId),
            planner.TryPlan,
            plan => GetDecision(spline, plan))
    {
    }

    internal AiPursuitLaneSelector(
        Func<int, AiAdjacentLanePoints> getAdjacent,
        Func<int, int, bool> isSameDirection,
        Func<int, IReadOnlySet<int>, AiRouteSearchLimits, AiRouteSearchResult> tryPlan)
        : this(
            getAdjacent,
            isSameDirection,
            tryPlan,
            plan => plan.JunctionDecisions.Count == 0
                ? null
                : new AiPursuitLaneDecision(
                    plan.JunctionDecisions.Keys.OrderBy(id => id).First(),
                    plan.DistanceMeters))
    {
    }

    internal AiPursuitLaneSelector(
        Func<int, AiAdjacentLanePoints> getAdjacent,
        Func<int, int, bool> isSameDirection,
        Func<int, IReadOnlySet<int>, AiRouteSearchLimits, AiRouteSearchResult> tryPlan,
        Func<AiRoutePlan, AiPursuitLaneDecision?> getDecision)
    {
        _getAdjacent = getAdjacent;
        _isSameDirection = isSameDirection;
        _tryPlan = tryPlan;
        _getDecision = getDecision;
    }

    public AiPursuitLaneSelectionResult Select(
        int currentPointId,
        IReadOnlySet<int> targetPointIds,
        AiRouteSearchLimits limits,
        float maneuverDistanceMeters) =>
        Select(
            currentPointId,
            targetPointIds,
            limits,
            maneuverDistanceMeters,
            float.MaxValue);

    public AiPursuitLaneSelectionResult Select(
        int currentPointId,
        IReadOnlySet<int> targetPointIds,
        AiRouteSearchLimits limits,
        float maneuverDistanceMeters,
        float lookaheadMeters)
    {
        ArgumentNullException.ThrowIfNull(targetPointIds);
        if (!float.IsFinite(maneuverDistanceMeters) || maneuverDistanceMeters <= 0)
            throw new ArgumentOutOfRangeException(nameof(maneuverDistanceMeters));
        if (!float.IsFinite(lookaheadMeters) || lookaheadMeters < maneuverDistanceMeters)
            throw new ArgumentOutOfRangeException(nameof(lookaheadMeters));

        var currentRoute = _tryPlan(currentPointId, targetPointIds, limits);
        var currentDiagnostic = CreateRouteDiagnostic(
            currentPointId,
            null,
            currentRoute,
            null,
            currentRoute.Plan == null
                ? AiPursuitLaneEvaluationReason.NoForwardRoute
                : AiPursuitLaneEvaluationReason.CurrentLaneValid);
        if (currentRoute.Plan != null)
        {
            return new AiPursuitLaneSelectionResult(
                AiPursuitLaneSelectionKind.Stay,
                null,
                [])
            {
                Reason = AiPursuitLaneEvaluationReason.CurrentLaneValid,
                CurrentLaneRoute = currentDiagnostic
            };
        }

        var adjacent = _getAdjacent(currentPointId);
        var diagnostics = new List<AiPursuitLaneCandidateDiagnostic>(2);
        var routeDiagnostics = new List<AiPursuitLaneRouteDiagnostic>(2);
        var selections = new List<AiPursuitLaneSelection>(2);
        Evaluate(
            currentPointId,
            adjacent.LeftPointId,
            AiLaneChangeDirection.Left,
            targetPointIds,
            limits,
            maneuverDistanceMeters,
            lookaheadMeters,
            diagnostics,
            routeDiagnostics,
            selections);
        Evaluate(
            currentPointId,
            adjacent.RightPointId,
            AiLaneChangeDirection.Right,
            targetPointIds,
            limits,
            maneuverDistanceMeters,
            lookaheadMeters,
            diagnostics,
            routeDiagnostics,
            selections);

        var orderedSelections = selections
            .OrderBy(selection => selection.DestinationPlan.DistanceMeters)
            .ThenBy(selection => selection.ToPointId)
            .ToArray();
        var selected = orderedSelections.FirstOrDefault();
        if (orderedSelections.Length > 1
            && Math.Abs(
                orderedSelections[0].DestinationPlan.DistanceMeters
                - orderedSelections[1].DestinationPlan.DistanceMeters) < 0.001f)
        {
            selected = null;
        }
        var reason = selected == null
            ? ResolveFailureReason(diagnostics)
            : AiPursuitLaneEvaluationReason.RoutePreparation;
        return selected == null
            ? new AiPursuitLaneSelectionResult(
                AiPursuitLaneSelectionKind.Unreachable,
                null,
                diagnostics)
            {
                Reason = reason,
                CurrentLaneRoute = currentDiagnostic,
                CandidateLaneRoutes = routeDiagnostics
            }
            : new AiPursuitLaneSelectionResult(
                AiPursuitLaneSelectionKind.Change,
                selected,
                diagnostics)
            {
                Reason = reason,
                CurrentLaneRoute = currentDiagnostic,
                CandidateLaneRoutes = routeDiagnostics
            };
    }

    private void Evaluate(
        int currentPointId,
        int adjacentPointId,
        AiLaneChangeDirection direction,
        IReadOnlySet<int> targetPointIds,
        AiRouteSearchLimits limits,
        float maneuverDistanceMeters,
        float lookaheadMeters,
        ICollection<AiPursuitLaneCandidateDiagnostic> diagnostics,
        ICollection<AiPursuitLaneRouteDiagnostic> routeDiagnostics,
        ICollection<AiPursuitLaneSelection> selections)
    {
        if (adjacentPointId < 0)
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                null,
                direction,
                AiPursuitLaneRejectionReason.MissingNeighbor));
            routeDiagnostics.Add(new AiPursuitLaneRouteDiagnostic(
                null,
                direction,
                AiRouteSearchFailure.InvalidRequest,
                null,
                0,
                0,
                null,
                null,
                AiPursuitLaneEvaluationReason.NoAdjacentLane));
            return;
        }

        if (!_isSameDirection(currentPointId, adjacentPointId))
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId,
                direction,
                AiPursuitLaneRejectionReason.OppositeDirection));
            routeDiagnostics.Add(new AiPursuitLaneRouteDiagnostic(
                adjacentPointId,
                direction,
                AiRouteSearchFailure.InvalidRequest,
                null,
                0,
                0,
                null,
                null,
                AiPursuitLaneEvaluationReason.OppositeDirection));
            return;
        }

        var route = _tryPlan(adjacentPointId, targetPointIds, limits);
        if (route.Plan == null)
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId,
                direction,
                AiPursuitLaneRejectionReason.NoForwardRoute));
            routeDiagnostics.Add(CreateRouteDiagnostic(
                adjacentPointId,
                direction,
                route,
                null,
                AiPursuitLaneEvaluationReason.NoForwardRoute));
            return;
        }

        var decision = _getDecision(route.Plan);
        if (!decision.HasValue)
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId,
                direction,
                AiPursuitLaneRejectionReason.NoRealJunction));
            routeDiagnostics.Add(CreateRouteDiagnostic(
                adjacentPointId,
                direction,
                route,
                null,
                AiPursuitLaneEvaluationReason.NoRealJunction));
            return;
        }

        var distanceToDecision = decision.Value.DistanceMeters;
        if (distanceToDecision < maneuverDistanceMeters)
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId,
                direction,
                AiPursuitLaneRejectionReason.InsufficientPreparationDistance));
            routeDiagnostics.Add(CreateRouteDiagnostic(
                adjacentPointId,
                direction,
                route,
                decision,
                AiPursuitLaneEvaluationReason.InsufficientPreparationDistance));
            return;
        }
        if (distanceToDecision > lookaheadMeters)
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId,
                direction,
                AiPursuitLaneRejectionReason.BeyondLookahead));
            routeDiagnostics.Add(CreateRouteDiagnostic(
                adjacentPointId,
                direction,
                route,
                decision,
                AiPursuitLaneEvaluationReason.BeyondLookahead));
            return;
        }

        diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
            adjacentPointId,
            direction,
            null));
        routeDiagnostics.Add(CreateRouteDiagnostic(
            adjacentPointId,
            direction,
            route,
            decision,
            AiPursuitLaneEvaluationReason.RoutePreparation));
        selections.Add(new AiPursuitLaneSelection(
            currentPointId,
            adjacentPointId,
            direction,
            route.Plan,
            distanceToDecision)
        {
            JunctionId = decision.Value.JunctionId
        });
    }

    private static AiPursuitLaneDecision? GetDecision(AiSpline spline, AiRoutePlan plan)
    {
        foreach (var node in plan.Nodes)
        {
            var junctionId = spline.Points[node.PointId].JunctionStartId;
            if (junctionId >= 0 && plan.JunctionDecisions.ContainsKey(junctionId))
                return new AiPursuitLaneDecision(junctionId, node.DistanceFromStartMeters);
        }

        return null;
    }

    private static AiPursuitLaneRouteDiagnostic CreateRouteDiagnostic(
        int pointId,
        AiLaneChangeDirection? direction,
        AiRouteSearchResult route,
        AiPursuitLaneDecision? decision,
        AiPursuitLaneEvaluationReason reason) =>
        new(
            pointId,
            direction,
            route.Failure,
            route.Plan?.DistanceMeters,
            route.MaximumExploredDistanceMeters,
            route.JunctionEdgesExamined,
            decision?.JunctionId,
            decision?.DistanceMeters,
            reason);

    private static AiPursuitLaneEvaluationReason ResolveFailureReason(
        IReadOnlyCollection<AiPursuitLaneCandidateDiagnostic> diagnostics)
    {
        if (diagnostics.Any(candidate =>
                candidate.Rejection == AiPursuitLaneRejectionReason.InsufficientPreparationDistance))
        {
            return AiPursuitLaneEvaluationReason.InsufficientPreparationDistance;
        }
        if (diagnostics.Any(candidate =>
                candidate.Rejection == AiPursuitLaneRejectionReason.BeyondLookahead))
        {
            return AiPursuitLaneEvaluationReason.BeyondLookahead;
        }
        if (diagnostics.Any(candidate =>
                candidate.Rejection == AiPursuitLaneRejectionReason.NoRealJunction))
        {
            return AiPursuitLaneEvaluationReason.NoRealJunction;
        }
        if (diagnostics.Any(candidate =>
                candidate.Rejection == AiPursuitLaneRejectionReason.NoForwardRoute))
        {
            return AiPursuitLaneEvaluationReason.NoForwardRoute;
        }
        if (diagnostics.Any(candidate =>
                candidate.Rejection == AiPursuitLaneRejectionReason.OppositeDirection))
        {
            return AiPursuitLaneEvaluationReason.OppositeDirection;
        }
        return AiPursuitLaneEvaluationReason.NoAdjacentLane;
    }
}
