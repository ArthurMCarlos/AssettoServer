using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using AssettoServer.Server.Ai.Splines;

namespace AssettoServer.Server.Ai.Routing;

public enum AiLaneChangeDirection
{
    Left,
    Right
}

public enum AiPursuitLaneMotivation
{
    FutureJunction,
    TargetLaneAlignment
}

public enum AiPursuitLanePhysicalRelation
{
    SameLane,
    ImmediateLeft,
    ImmediateRight,
    NonAdjacent,
    OppositeDirection,
    InvalidGeometry
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
    NonAdjacent,
    OppositeDirection,
    InvalidGeometry,
    NoForwardRoute,
    NoRealJunction,
    BeyondLookahead,
    InsufficientPreparationDistance
}

public enum AiPursuitLaneEvaluationReason
{
    CurrentLaneValid,
    NoAdjacentLane,
    NonAdjacent,
    OppositeDirection,
    InvalidGeometry,
    NoForwardRoute,
    NoRealJunction,
    BeyondLookahead,
    InsufficientPreparationDistance,
    RoutePreparation,
    Cooldown
}

public readonly record struct AiAdjacentLanePoints(int LeftPointId, int RightPointId);
public readonly record struct AiPursuitLaneDecision(int JunctionId, float DistanceMeters);

public sealed record AiPursuitLaneSelection(
    int FromPointId,
    int ToPointId,
    AiLaneChangeDirection Direction,
    AiRoutePlan DestinationPlan,
    float? DistanceToDecisionMeters)
{
    public int? JunctionId { get; init; }
    public AiPursuitLaneMotivation Motivation { get; init; } = AiPursuitLaneMotivation.FutureJunction;
    public AiPursuitLanePhysicalRelation PhysicalRelation { get; init; }
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
    AiPursuitLaneEvaluationReason Reason)
{
    public AiPursuitLanePhysicalRelation? PhysicalRelation { get; init; }
}

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
    private readonly Func<int, int, AiLaneChangeDirection, AiPursuitLanePhysicalRelation> _getPhysicalRelation;
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
            (sourcePointId, targetPointId, direction) =>
                GetPhysicalRelation(spline, sourcePointId, targetPointId, direction),
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
            (_, _, direction) => Relation(direction),
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
        : this(
            getAdjacent,
            isSameDirection,
            (_, _, direction) => Relation(direction),
            tryPlan,
            getDecision)
    {
    }

    internal AiPursuitLaneSelector(
        Func<int, AiAdjacentLanePoints> getAdjacent,
        Func<int, int, bool> isSameDirection,
        Func<int, int, AiLaneChangeDirection, AiPursuitLanePhysicalRelation> getPhysicalRelation,
        Func<int, IReadOnlySet<int>, AiRouteSearchLimits, AiRouteSearchResult> tryPlan,
        Func<AiRoutePlan, AiPursuitLaneDecision?> getDecision)
    {
        _getAdjacent = getAdjacent;
        _isSameDirection = isSameDirection;
        _getPhysicalRelation = getPhysicalRelation;
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

    public AiPursuitLaneSelectionResult SelectForRoutePreparation(
        int currentPointId,
        IReadOnlySet<int> targetPointIds,
        AiRouteSearchLimits limits,
        float maneuverDistanceMeters,
        float lookaheadMeters) =>
        Select(
            currentPointId,
            targetPointIds,
            limits,
            maneuverDistanceMeters,
            lookaheadMeters);

    public AiPursuitLaneSelectionResult SelectForAlignment(
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
        var adjacent = _getAdjacent(currentPointId);
        var diagnostics = new List<AiPursuitLaneCandidateDiagnostic>(2);
        var routeDiagnostics = new List<AiPursuitLaneRouteDiagnostic>(2);
        var selections = new List<AiPursuitLaneSelection>(2);
        EvaluateAlignment(
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
        EvaluateAlignment(
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

        if (selected != null
            && currentRoute.Plan != null
            && currentRoute.Plan.DistanceMeters
                <= selected.DestinationPlan.DistanceMeters + 0.001f)
        {
            selected = null;
        }

        if (selected != null)
        {
            return new AiPursuitLaneSelectionResult(
                AiPursuitLaneSelectionKind.Change,
                selected,
                diagnostics)
            {
                Reason = AiPursuitLaneEvaluationReason.RoutePreparation,
                CurrentLaneRoute = currentDiagnostic,
                CandidateLaneRoutes = routeDiagnostics
            };
        }

        return new AiPursuitLaneSelectionResult(
            currentRoute.Plan == null
                ? AiPursuitLaneSelectionKind.Unreachable
                : AiPursuitLaneSelectionKind.Stay,
            null,
            diagnostics)
        {
            Reason = currentRoute.Plan == null
                ? ResolveFailureReason(diagnostics)
                : AiPursuitLaneEvaluationReason.CurrentLaneValid,
            CurrentLaneRoute = currentDiagnostic,
            CandidateLaneRoutes = routeDiagnostics
        };
    }

    private void EvaluateAlignment(
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
                null, direction, AiPursuitLaneRejectionReason.MissingNeighbor));
            routeDiagnostics.Add(new AiPursuitLaneRouteDiagnostic(
                null, direction, AiRouteSearchFailure.InvalidRequest, null, 0, 0,
                null, null, AiPursuitLaneEvaluationReason.NoAdjacentLane));
            return;
        }

        if (!_isSameDirection(currentPointId, adjacentPointId))
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId, direction, AiPursuitLaneRejectionReason.OppositeDirection));
            routeDiagnostics.Add(new AiPursuitLaneRouteDiagnostic(
                adjacentPointId, direction, AiRouteSearchFailure.InvalidRequest, null, 0, 0,
                null, null, AiPursuitLaneEvaluationReason.OppositeDirection));
            return;
        }

        var physicalRelation = _getPhysicalRelation(currentPointId, adjacentPointId, direction);
        if (physicalRelation is AiPursuitLanePhysicalRelation.NonAdjacent
            or AiPursuitLanePhysicalRelation.InvalidGeometry)
        {
            var rejection = physicalRelation == AiPursuitLanePhysicalRelation.NonAdjacent
                ? AiPursuitLaneRejectionReason.NonAdjacent
                : AiPursuitLaneRejectionReason.InvalidGeometry;
            var reason = physicalRelation == AiPursuitLanePhysicalRelation.NonAdjacent
                ? AiPursuitLaneEvaluationReason.NonAdjacent
                : AiPursuitLaneEvaluationReason.InvalidGeometry;
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId, direction, rejection));
            routeDiagnostics.Add(new AiPursuitLaneRouteDiagnostic(
                adjacentPointId, direction, AiRouteSearchFailure.InvalidRequest, null, 0, 0,
                null, null, reason)
            {
                PhysicalRelation = physicalRelation
            });
            return;
        }

        var route = _tryPlan(adjacentPointId, targetPointIds, limits);
        if (route.Plan == null)
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId, direction, AiPursuitLaneRejectionReason.NoForwardRoute));
            routeDiagnostics.Add(CreateRouteDiagnostic(
                adjacentPointId, direction, route, null,
                AiPursuitLaneEvaluationReason.NoForwardRoute));
            return;
        }

        var distanceToTarget = route.Plan.DistanceMeters;
        if (distanceToTarget > lookaheadMeters)
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId, direction, AiPursuitLaneRejectionReason.BeyondLookahead));
            routeDiagnostics.Add(CreateRouteDiagnostic(
                adjacentPointId, direction, route, null,
                AiPursuitLaneEvaluationReason.BeyondLookahead));
            return;
        }

        diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(adjacentPointId, direction, null));
        routeDiagnostics.Add(CreateRouteDiagnostic(
            adjacentPointId, direction, route, null,
            AiPursuitLaneEvaluationReason.RoutePreparation));
        selections.Add(new AiPursuitLaneSelection(
            currentPointId,
            adjacentPointId,
            direction,
            route.Plan,
            null)
        {
            Motivation = AiPursuitLaneMotivation.TargetLaneAlignment,
            PhysicalRelation = physicalRelation
        });
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

        var physicalRelation = _getPhysicalRelation(currentPointId, adjacentPointId, direction);
        if (physicalRelation is AiPursuitLanePhysicalRelation.NonAdjacent
            or AiPursuitLanePhysicalRelation.InvalidGeometry)
        {
            var rejection = physicalRelation == AiPursuitLanePhysicalRelation.NonAdjacent
                ? AiPursuitLaneRejectionReason.NonAdjacent
                : AiPursuitLaneRejectionReason.InvalidGeometry;
            var reason = physicalRelation == AiPursuitLanePhysicalRelation.NonAdjacent
                ? AiPursuitLaneEvaluationReason.NonAdjacent
                : AiPursuitLaneEvaluationReason.InvalidGeometry;
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId, direction, rejection));
            routeDiagnostics.Add(new AiPursuitLaneRouteDiagnostic(
                adjacentPointId, direction, AiRouteSearchFailure.InvalidRequest, null, 0, 0,
                null, null, reason)
            {
                PhysicalRelation = physicalRelation
            });
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
            JunctionId = decision.Value.JunctionId,
            Motivation = AiPursuitLaneMotivation.FutureJunction,
            PhysicalRelation = physicalRelation
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
            reason)
        {
            PhysicalRelation = direction.HasValue ? Relation(direction.Value) : null
        };

    private static AiPursuitLanePhysicalRelation Relation(AiLaneChangeDirection direction) =>
        direction == AiLaneChangeDirection.Left
            ? AiPursuitLanePhysicalRelation.ImmediateLeft
            : AiPursuitLanePhysicalRelation.ImmediateRight;

    private static AiPursuitLanePhysicalRelation GetPhysicalRelation(
        AiSpline spline,
        int sourcePointId,
        int targetPointId,
        AiLaneChangeDirection direction)
    {
        var target = spline.Points[targetPointId];
        var reciprocal = direction == AiLaneChangeDirection.Left
            ? target.RightId == sourcePointId
            : target.LeftId == sourcePointId;
        if (!reciprocal)
            return AiPursuitLanePhysicalRelation.NonAdjacent;

        var sourcePosition = spline.Points[sourcePointId].Position;
        var offset = target.Position - sourcePosition;
        var distance = offset.Length();
        if (!IsFinite(sourcePosition)
            || !IsFinite(target.Position)
            || !float.IsFinite(distance)
            || distance < 1.5f
            || distance > 10.0f
            || MathF.Abs(offset.Y) > 2.5f)
        {
            return AiPursuitLanePhysicalRelation.InvalidGeometry;
        }

        return Relation(direction);
    }

    private static bool IsFinite(Vector3 value) =>
        float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);

    private static AiPursuitLaneEvaluationReason ResolveFailureReason(
        IReadOnlyCollection<AiPursuitLaneCandidateDiagnostic> diagnostics)
    {
        if (diagnostics.Any(candidate =>
                candidate.Rejection == AiPursuitLaneRejectionReason.InvalidGeometry))
        {
            return AiPursuitLaneEvaluationReason.InvalidGeometry;
        }
        if (diagnostics.Any(candidate =>
                candidate.Rejection == AiPursuitLaneRejectionReason.NonAdjacent))
        {
            return AiPursuitLaneEvaluationReason.NonAdjacent;
        }
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
