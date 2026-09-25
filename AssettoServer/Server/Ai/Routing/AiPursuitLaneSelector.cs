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
    TargetLaneAlignment,
    TrafficBypass,
    TrafficReturn
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
    public AiPursuitLaneMotivation? Motivation { get; init; }
}

public sealed record AiPursuitLaneSelectionResult(
    AiPursuitLaneSelectionKind Kind,
    AiPursuitLaneSelection? Selection,
    IReadOnlyList<AiPursuitLaneCandidateDiagnostic> Candidates)
{
    public AiPursuitLaneEvaluationReason Reason { get; init; }
    public AiPursuitLaneRouteDiagnostic CurrentLaneRoute { get; init; } = null!;
    public IReadOnlyList<AiPursuitLaneRouteDiagnostic> CandidateLaneRoutes { get; init; } = [];
    public float? RequiredTransitionDistanceMeters { get; init; }
    public float? SourceAvailableDistanceMeters { get; init; }
    public float? DestinationAvailableDistanceMeters { get; init; }
}

public sealed class AiPursuitLaneSelector
{
    private readonly Func<int, AiAdjacentLanePoints> _getAdjacent;
    private readonly Func<int, int, AiLaneChangeDirection, AiPursuitLanePhysicalRelation> _getPhysicalRelation;
    private readonly Func<int, int, bool> _isSameDirection;
    private readonly Func<int, IReadOnlySet<int>, AiRouteSearchLimits, AiRouteSearchResult> _tryPlan;
    private readonly Func<AiRoutePlan, AiPursuitLaneDecision?> _getDecision;
    private readonly Func<int, int>? _getPhysicalNext;
    private readonly Func<int, float>? _getPhysicalLength;

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
            plan => GetDecision(spline, plan),
            point => spline.Points[point].JunctionStartId >= 0 ? -1 : spline.Points[point].NextId,
            point => spline.Points[point].Length)
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
        Func<AiRoutePlan, AiPursuitLaneDecision?> getDecision,
        Func<int, int>? getPhysicalNext = null,
        Func<int, float>? getPhysicalLength = null)
    {
        _getAdjacent = getAdjacent;
        _isSameDirection = isSameDirection;
        _getPhysicalRelation = getPhysicalRelation;
        _tryPlan = tryPlan;
        _getDecision = getDecision;
        _getPhysicalNext = getPhysicalNext;
        _getPhysicalLength = getPhysicalLength;
    }

    internal bool IsOnTargetPhysicalLane(int current, int target, AiRouteSearchLimits limits)
    {
        float distance = 0;
        // No junction decisions/merges or route costs can prove physical lane membership.
        int budget = Math.Max(1, limits.MaxVisitedNodes / 3);
        while (current >= 0 && budget-- > 0 && distance <= limits.MaxDistanceMeters)
        {
            if (current == target) return true;
            if (_getPhysicalNext == null || _getPhysicalLength == null) return false;
            float length = _getPhysicalLength(current);
            if (!float.IsFinite(length) || length < 0) return false;
            distance += length;
            current = _getPhysicalNext(current);
        }
        return false;
    }

    internal AiPursuitLaneSelection? SelectForTrafficReturn(int current, int target, AiRouteSearchLimits limits)
    {
        var neighbors = _getAdjacent(current);
        int remaining = limits.MaxVisitedNodes;
        foreach (var (point, direction) in new[] { (neighbors.LeftPointId, AiLaneChangeDirection.Left),
                     (neighbors.RightPointId, AiLaneChangeDirection.Right) })
        {
            if (point < 0 || remaining <= 0 || !_isSameDirection(current, point)
                || _getPhysicalRelation(current, point, direction) != Relation(direction)
                || !IsOnTargetPhysicalLane(point, target, limits)) continue;
            var route = _tryPlan(point, new HashSet<int> { target }, new(limits.MaxDistanceMeters, remaining));
            remaining -= route.VisitedNodes;
            if (route.Plan != null)
                return new(current, point, direction, route.Plan, null)
                { Motivation = AiPursuitLaneMotivation.TrafficReturn, PhysicalRelation = Relation(direction) };
        }
        return null;
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

    internal IReadOnlyList<AiPursuitLaneSelection> SelectForTrafficBypass(
        int currentPointId, int physicalTargetPointId, AiRouteSearchLimits limits)
    {
        var adjacent = _getAdjacent(currentPointId);
        // Equivalent immediate target lanes remain forward-only destinations, never graph edges.
        var targetNeighbors = _getAdjacent(physicalTargetPointId);
        var targets = new HashSet<int> { physicalTargetPointId };
        foreach (var (id, direction) in new[] { (targetNeighbors.LeftPointId, AiLaneChangeDirection.Left),
                     (targetNeighbors.RightPointId, AiLaneChangeDirection.Right) })
            if (id >= 0 && _isSameDirection(physicalTargetPointId, id)
                && _getPhysicalRelation(physicalTargetPointId, id, direction) == Relation(direction))
                targets.Add(id);
        var result = new List<AiPursuitLaneSelection>(2);
        int remaining = limits.MaxVisitedNodes;
        foreach (var (id, direction) in new[] { (adjacent.LeftPointId, AiLaneChangeDirection.Left),
                     (adjacent.RightPointId, AiLaneChangeDirection.Right) })
        {
            if (id < 0 || remaining <= 0 || !_isSameDirection(currentPointId, id)
                || _getPhysicalRelation(currentPointId, id, direction) != Relation(direction)) continue;
            var route = _tryPlan(id, targets, new(limits.MaxDistanceMeters, remaining));
            remaining -= route.VisitedNodes;
            if (route.Plan != null)
                result.Add(new(currentPointId, id, direction, route.Plan, null)
                { Motivation = AiPursuitLaneMotivation.TrafficBypass, PhysicalRelation = Relation(direction) });
        }
        return result;
    }

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
                null, null, AiPursuitLaneEvaluationReason.NoAdjacentLane)
            {
                Motivation = AiPursuitLaneMotivation.TargetLaneAlignment
            });
            return;
        }

        if (!_isSameDirection(currentPointId, adjacentPointId))
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId, direction, AiPursuitLaneRejectionReason.OppositeDirection));
            routeDiagnostics.Add(new AiPursuitLaneRouteDiagnostic(
                adjacentPointId, direction, AiRouteSearchFailure.InvalidRequest, null, 0, 0,
                null, null, AiPursuitLaneEvaluationReason.OppositeDirection)
            {
                Motivation = AiPursuitLaneMotivation.TargetLaneAlignment
            });
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
                PhysicalRelation = physicalRelation,
                Motivation = AiPursuitLaneMotivation.TargetLaneAlignment
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
                AiPursuitLaneEvaluationReason.NoForwardRoute,
                AiPursuitLaneMotivation.TargetLaneAlignment));
            return;
        }

        var decision = _getDecision(route.Plan);
        if (decision.HasValue && decision.Value.DistanceMeters < maneuverDistanceMeters)
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId, direction,
                AiPursuitLaneRejectionReason.InsufficientPreparationDistance));
            routeDiagnostics.Add(CreateRouteDiagnostic(
                adjacentPointId, direction, route, decision,
                AiPursuitLaneEvaluationReason.InsufficientPreparationDistance,
                AiPursuitLaneMotivation.TargetLaneAlignment));
            return;
        }

        var distanceToTarget = route.Plan.DistanceMeters;
        if (distanceToTarget > lookaheadMeters)
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId, direction, AiPursuitLaneRejectionReason.BeyondLookahead));
            routeDiagnostics.Add(CreateRouteDiagnostic(
                adjacentPointId, direction, route, null,
                AiPursuitLaneEvaluationReason.BeyondLookahead,
                AiPursuitLaneMotivation.TargetLaneAlignment));
            return;
        }

        diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(adjacentPointId, direction, null));
        routeDiagnostics.Add(CreateRouteDiagnostic(
            adjacentPointId, direction, route, null,
            AiPursuitLaneEvaluationReason.RoutePreparation,
            AiPursuitLaneMotivation.TargetLaneAlignment));
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
                AiPursuitLaneEvaluationReason.NoAdjacentLane)
            {
                Motivation = AiPursuitLaneMotivation.FutureJunction
            });
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
                AiPursuitLaneEvaluationReason.OppositeDirection)
            {
                Motivation = AiPursuitLaneMotivation.FutureJunction
            });
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
                PhysicalRelation = physicalRelation,
                Motivation = AiPursuitLaneMotivation.FutureJunction
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
                AiPursuitLaneEvaluationReason.NoForwardRoute,
                AiPursuitLaneMotivation.FutureJunction));
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
                AiPursuitLaneEvaluationReason.NoRealJunction,
                AiPursuitLaneMotivation.FutureJunction));
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
                AiPursuitLaneEvaluationReason.InsufficientPreparationDistance,
                AiPursuitLaneMotivation.FutureJunction));
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
                AiPursuitLaneEvaluationReason.BeyondLookahead,
                AiPursuitLaneMotivation.FutureJunction));
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
            AiPursuitLaneEvaluationReason.RoutePreparation,
            AiPursuitLaneMotivation.FutureJunction));
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
        AiPursuitLaneEvaluationReason reason,
        AiPursuitLaneMotivation? motivation = null) =>
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
            PhysicalRelation = direction.HasValue ? Relation(direction.Value) : null,
            Motivation = motivation
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
