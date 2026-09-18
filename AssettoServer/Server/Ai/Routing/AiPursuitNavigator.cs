using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace AssettoServer.Server.Ai.Routing;

public sealed record AiPursuitNavigationOptions(
    AiRouteSearchLimits SearchLimits,
    int RouteGraceMilliseconds,
    int MaximumTargetCandidates = 16,
    float ExtensionDistanceMeters = 500,
    int ExtensionMaxVisitedNodes = 5000);

public enum AiPursuitNavigationStatus
{
    Active,
    RouteTemporarilyUnavailable,
    NoRoute
}

public enum AiPursuitRouteUpdateKind
{
    Selected,
    Reused,
    Extended,
    Recalculated,
    Recovered
}

public sealed record AiPursuitRouteState(
    int TargetPointId,
    AiRoutePlan Plan,
    long Revision,
    long? FirstFailureMilliseconds);

public sealed record AiPursuitNavigationResult(
    AiPursuitNavigationStatus Status,
    AiPursuitRouteState? State,
    AiPursuitRouteUpdateKind? UpdateKind,
    AiRouteSearchFailure SearchFailure,
    int VisitedNodes);

public sealed class AiPursuitNavigator
{
    private readonly Func<int, IReadOnlySet<int>, AiRouteSearchLimits, AiRouteSearchResult> _tryPlan;
    private readonly Func<Vector3, Vector3, float, int, IReadOnlyList<AiPursuitTargetCandidate>> _findCandidates;

    public AiPursuitNavigator(
        AiRoutePlanner planner,
        AiPursuitTargetLocator targetLocator)
        : this(planner.TryPlan, targetLocator.FindCandidates)
    {
    }

    internal AiPursuitNavigator(
        Func<int, IReadOnlySet<int>, AiRouteSearchLimits, AiRouteSearchResult> tryPlan,
        Func<Vector3, Vector3, float, int, IReadOnlyList<AiPursuitTargetCandidate>> findCandidates)
    {
        _tryPlan = tryPlan ?? throw new ArgumentNullException(nameof(tryPlan));
        _findCandidates = findCandidates ?? throw new ArgumentNullException(nameof(findCandidates));
    }

    public AiPursuitNavigationResult Update(
        int policePointId,
        Vector3 targetPosition,
        Vector3 targetVelocity,
        float maximumTargetDistanceSquared,
        long nowMilliseconds,
        AiPursuitRouteState? previous,
        AiPursuitNavigationOptions options)
    {
        Validate(options);

        var candidates = _findCandidates(
            targetPosition,
            targetVelocity,
            maximumTargetDistanceSquared,
            options.MaximumTargetCandidates);
        var targetPointIds = candidates
            .Select(candidate => candidate.PointId)
            .ToHashSet();

        if (targetPointIds.Count == 0)
        {
            return HandleFailure(
                previous,
                nowMilliseconds,
                options.RouteGraceMilliseconds,
                AiRouteSearchFailure.Unreachable,
                0);
        }

        if (previous != null
            && TryTrimPlan(previous.Plan, policePointId, out var remainingPlan))
        {
            if (targetPointIds.Contains(previous.TargetPointId))
            {
                return ActiveFromCache(previous, remainingPlan, previous.TargetPointId);
            }

            var extensionLimits = new AiRouteSearchLimits(
                Math.Min(options.ExtensionDistanceMeters, options.SearchLimits.MaxDistanceMeters),
                Math.Min(options.ExtensionMaxVisitedNodes, options.SearchLimits.MaxVisitedNodes));
            var extension = _tryPlan(
                previous.TargetPointId,
                targetPointIds,
                extensionLimits);
            if (extension.Plan != null)
            {
                var combined = Combine(remainingPlan, extension.Plan);
                return ActiveWithNewRoute(
                    previous,
                    combined,
                    extension.Plan.Nodes[^1].PointId,
                    previous.FirstFailureMilliseconds.HasValue
                        ? AiPursuitRouteUpdateKind.Recovered
                        : AiPursuitRouteUpdateKind.Extended,
                    extension.VisitedNodes);
            }
        }

        var route = _tryPlan(policePointId, targetPointIds, options.SearchLimits);
        if (route.Plan != null)
        {
            return ActiveWithNewRoute(
                previous,
                route.Plan,
                route.Plan.Nodes[^1].PointId,
                previous == null
                    ? AiPursuitRouteUpdateKind.Selected
                    : previous.FirstFailureMilliseconds.HasValue
                        ? AiPursuitRouteUpdateKind.Recovered
                        : AiPursuitRouteUpdateKind.Recalculated,
                route.VisitedNodes);
        }

        return HandleFailure(
            previous,
            nowMilliseconds,
            options.RouteGraceMilliseconds,
            route.Failure,
            route.VisitedNodes);
    }

    private static AiPursuitNavigationResult ActiveFromCache(
        AiPursuitRouteState previous,
        AiRoutePlan remainingPlan,
        int targetPointId)
    {
        var recovered = previous.FirstFailureMilliseconds.HasValue;
        var state = previous with
        {
            TargetPointId = targetPointId,
            Plan = remainingPlan,
            Revision = recovered ? previous.Revision + 1 : previous.Revision,
            FirstFailureMilliseconds = null
        };
        return new AiPursuitNavigationResult(
            AiPursuitNavigationStatus.Active,
            state,
            recovered ? AiPursuitRouteUpdateKind.Recovered : AiPursuitRouteUpdateKind.Reused,
            AiRouteSearchFailure.None,
            0);
    }

    private static AiPursuitNavigationResult ActiveWithNewRoute(
        AiPursuitRouteState? previous,
        AiRoutePlan plan,
        int targetPointId,
        AiPursuitRouteUpdateKind updateKind,
        int visitedNodes)
    {
        var state = new AiPursuitRouteState(
            targetPointId,
            plan,
            (previous?.Revision ?? 0) + 1,
            null);
        return new AiPursuitNavigationResult(
            AiPursuitNavigationStatus.Active,
            state,
            updateKind,
            AiRouteSearchFailure.None,
            visitedNodes);
    }

    private static AiPursuitNavigationResult HandleFailure(
        AiPursuitRouteState? previous,
        long nowMilliseconds,
        int graceMilliseconds,
        AiRouteSearchFailure failure,
        int visitedNodes)
    {
        if (previous != null)
        {
            var firstFailure = previous.FirstFailureMilliseconds ?? nowMilliseconds;
            if (nowMilliseconds - firstFailure < graceMilliseconds)
            {
                return new AiPursuitNavigationResult(
                    AiPursuitNavigationStatus.RouteTemporarilyUnavailable,
                    previous with { FirstFailureMilliseconds = firstFailure },
                    null,
                    failure,
                    visitedNodes);
            }
        }

        return new AiPursuitNavigationResult(
            AiPursuitNavigationStatus.NoRoute,
            null,
            null,
            failure,
            visitedNodes);
    }

    private static bool TryTrimPlan(
        AiRoutePlan plan,
        int policePointId,
        out AiRoutePlan remainingPlan)
    {
        var startIndex = -1;
        for (var i = 0; i < plan.Nodes.Count; i++)
        {
            if (plan.Nodes[i].PointId == policePointId)
            {
                startIndex = i;
                break;
            }
        }

        if (startIndex < 0)
        {
            remainingPlan = plan;
            return false;
        }

        var offset = plan.Nodes[startIndex].DistanceFromStartMeters;
        var nodes = plan.Nodes
            .Skip(startIndex)
            .Select(node => node with
            {
                DistanceFromStartMeters = node.DistanceFromStartMeters - offset
            })
            .ToArray();
        remainingPlan = new AiRoutePlan(
            plan.DistanceMeters - offset,
            nodes,
            plan.JunctionDecisions);
        return true;
    }

    private static AiRoutePlan Combine(AiRoutePlan current, AiRoutePlan extension)
    {
        var nodes = new List<AiRouteNode>(current.Nodes);
        foreach (var node in extension.Nodes.Skip(1))
        {
            nodes.Add(node with
            {
                DistanceFromStartMeters = current.DistanceMeters + node.DistanceFromStartMeters
            });
        }

        var decisions = new Dictionary<int, bool>(current.JunctionDecisions);
        foreach (var decision in extension.JunctionDecisions)
            decisions[decision.Key] = decision.Value;

        return new AiRoutePlan(
            current.DistanceMeters + extension.DistanceMeters,
            nodes,
            decisions);
    }

    private static void Validate(AiPursuitNavigationOptions options)
    {
        ArgumentNullException.ThrowIfNull(options);
        ArgumentNullException.ThrowIfNull(options.SearchLimits);
        if (options.RouteGraceMilliseconds < 0)
            throw new ArgumentOutOfRangeException(nameof(options.RouteGraceMilliseconds));
        if (options.MaximumTargetCandidates <= 0)
            throw new ArgumentOutOfRangeException(nameof(options.MaximumTargetCandidates));
        if (!float.IsFinite(options.ExtensionDistanceMeters)
            || options.ExtensionDistanceMeters <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(options.ExtensionDistanceMeters));
        }
        if (options.ExtensionMaxVisitedNodes <= 0)
            throw new ArgumentOutOfRangeException(nameof(options.ExtensionMaxVisitedNodes));
    }
}
