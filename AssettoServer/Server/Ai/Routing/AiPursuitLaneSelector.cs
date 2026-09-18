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
    InsufficientPreparationDistance
}

public readonly record struct AiAdjacentLanePoints(int LeftPointId, int RightPointId);

public sealed record AiPursuitLaneSelection(
    int FromPointId,
    int ToPointId,
    AiLaneChangeDirection Direction,
    AiRoutePlan DestinationPlan,
    float DistanceToDecisionMeters);

public sealed record AiPursuitLaneCandidateDiagnostic(
    int? PointId,
    AiLaneChangeDirection Direction,
    AiPursuitLaneRejectionReason? Rejection);

public sealed record AiPursuitLaneSelectionResult(
    AiPursuitLaneSelectionKind Kind,
    AiPursuitLaneSelection? Selection,
    IReadOnlyList<AiPursuitLaneCandidateDiagnostic> Candidates);

public sealed class AiPursuitLaneSelector
{
    private readonly Func<int, AiAdjacentLanePoints> _getAdjacent;
    private readonly Func<int, int, bool> _isSameDirection;
    private readonly Func<int, IReadOnlySet<int>, AiRouteSearchLimits, AiRouteSearchResult> _tryPlan;
    private readonly Func<AiRoutePlan, float> _getDistanceToDecision;

    public AiPursuitLaneSelector(AiSpline spline, AiRoutePlanner planner)
        : this(
            pointId => new AiAdjacentLanePoints(
                spline.Points[pointId].LeftId,
                spline.Points[pointId].RightId),
            (sourcePointId, targetPointId) =>
                spline.Operations.IsSameDirection(sourcePointId, targetPointId),
            planner.TryPlan,
            plan => GetDistanceToDecision(spline, plan))
    {
    }

    internal AiPursuitLaneSelector(
        Func<int, AiAdjacentLanePoints> getAdjacent,
        Func<int, int, bool> isSameDirection,
        Func<int, IReadOnlySet<int>, AiRouteSearchLimits, AiRouteSearchResult> tryPlan)
        : this(getAdjacent, isSameDirection, tryPlan, plan => plan.DistanceMeters)
    {
    }

    internal AiPursuitLaneSelector(
        Func<int, AiAdjacentLanePoints> getAdjacent,
        Func<int, int, bool> isSameDirection,
        Func<int, IReadOnlySet<int>, AiRouteSearchLimits, AiRouteSearchResult> tryPlan,
        Func<AiRoutePlan, float> getDistanceToDecision)
    {
        _getAdjacent = getAdjacent;
        _isSameDirection = isSameDirection;
        _tryPlan = tryPlan;
        _getDistanceToDecision = getDistanceToDecision;
    }

    public AiPursuitLaneSelectionResult Select(
        int currentPointId,
        IReadOnlySet<int> targetPointIds,
        AiRouteSearchLimits limits,
        float maneuverDistanceMeters)
    {
        ArgumentNullException.ThrowIfNull(targetPointIds);
        if (!float.IsFinite(maneuverDistanceMeters) || maneuverDistanceMeters <= 0)
            throw new ArgumentOutOfRangeException(nameof(maneuverDistanceMeters));

        if (_tryPlan(currentPointId, targetPointIds, limits).Plan != null)
        {
            return new AiPursuitLaneSelectionResult(
                AiPursuitLaneSelectionKind.Stay,
                null,
                []);
        }

        var adjacent = _getAdjacent(currentPointId);
        var diagnostics = new List<AiPursuitLaneCandidateDiagnostic>(2);
        var selections = new List<AiPursuitLaneSelection>(2);
        Evaluate(
            currentPointId,
            adjacent.LeftPointId,
            AiLaneChangeDirection.Left,
            targetPointIds,
            limits,
            maneuverDistanceMeters,
            diagnostics,
            selections);
        Evaluate(
            currentPointId,
            adjacent.RightPointId,
            AiLaneChangeDirection.Right,
            targetPointIds,
            limits,
            maneuverDistanceMeters,
            diagnostics,
            selections);

        var selected = selections
            .OrderBy(selection => selection.DestinationPlan.DistanceMeters)
            .ThenBy(selection => selection.ToPointId)
            .FirstOrDefault();
        return selected == null
            ? new AiPursuitLaneSelectionResult(
                AiPursuitLaneSelectionKind.Unreachable,
                null,
                diagnostics)
            : new AiPursuitLaneSelectionResult(
                AiPursuitLaneSelectionKind.Change,
                selected,
                diagnostics);
    }

    private void Evaluate(
        int currentPointId,
        int adjacentPointId,
        AiLaneChangeDirection direction,
        IReadOnlySet<int> targetPointIds,
        AiRouteSearchLimits limits,
        float maneuverDistanceMeters,
        ICollection<AiPursuitLaneCandidateDiagnostic> diagnostics,
        ICollection<AiPursuitLaneSelection> selections)
    {
        if (adjacentPointId < 0)
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                null,
                direction,
                AiPursuitLaneRejectionReason.MissingNeighbor));
            return;
        }

        if (!_isSameDirection(currentPointId, adjacentPointId))
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId,
                direction,
                AiPursuitLaneRejectionReason.OppositeDirection));
            return;
        }

        var route = _tryPlan(adjacentPointId, targetPointIds, limits);
        if (route.Plan == null)
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId,
                direction,
                AiPursuitLaneRejectionReason.NoForwardRoute));
            return;
        }

        var distanceToDecision = _getDistanceToDecision(route.Plan);
        if (distanceToDecision < maneuverDistanceMeters)
        {
            diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
                adjacentPointId,
                direction,
                AiPursuitLaneRejectionReason.InsufficientPreparationDistance));
            return;
        }

        diagnostics.Add(new AiPursuitLaneCandidateDiagnostic(
            adjacentPointId,
            direction,
            null));
        selections.Add(new AiPursuitLaneSelection(
            currentPointId,
            adjacentPointId,
            direction,
            route.Plan,
            distanceToDecision));
    }

    private static float GetDistanceToDecision(AiSpline spline, AiRoutePlan plan)
    {
        foreach (var node in plan.Nodes)
        {
            var junctionId = spline.Points[node.PointId].JunctionStartId;
            if (junctionId >= 0 && plan.JunctionDecisions.ContainsKey(junctionId))
                return node.DistanceFromStartMeters;
        }

        return plan.DistanceMeters;
    }
}
