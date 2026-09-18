using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Server.Ai;

public enum AiPursuitTrackingStatus
{
    Active,
    WaitingForSpawn,
    RouteTemporarilyUnavailable,
    MaxDistanceExceeded,
    NoRoute
}

public sealed record AiPursuitLaneChangeOptions(
    bool Enabled,
    float DistanceMeters,
    int CooldownMilliseconds);

public sealed record AiPursuitTrackingOptions(
    float MaximumSpatialDistanceMeters,
    float MaximumRouteDistanceMeters,
    int MaximumVisitedNodes,
    int RouteGraceMilliseconds,
    AiPursuitLaneChangeOptions? LaneChange = null);

public sealed record AiPursuitJunctionDecision(
    int JunctionId,
    bool TakeBranch,
    int EndPointId);

public sealed record AiPursuitRouteDiagnostics(
    long Revision,
    AiPursuitRouteUpdateKind UpdateKind,
    int PolicePointId,
    int TargetPointId,
    float RouteDistanceMeters,
    int VisitedNodes,
    IReadOnlyList<AiPursuitJunctionDecision> JunctionDecisions);

public sealed record AiPursuitSearchDiagnostics(
    int PolicePointId,
    int? PreviousTargetPointId,
    int? SelectedTargetPointId,
    IReadOnlyList<int> SpatialPointIds,
    IReadOnlyList<int> LaneEquivalentPointIds,
    IReadOnlyList<AiPursuitTargetRejection> Rejections,
    AiRouteSearchFailure SearchFailure,
    int VisitedNodes,
    float MaximumExploredDistanceMeters,
    int JunctionEdgesExamined);

public sealed record AiPursuitTrackingResult(
    AiPursuitTrackingStatus Status,
    float? RouteDistanceMeters,
    float TargetSpeedMetersPerSecond,
    AiPursuitRouteDiagnostics? RouteDiagnostics = null,
    AiPursuitSearchDiagnostics? SearchDiagnostics = null,
    AiPursuitLaneChangeDiagnostics? LaneChangeDiagnostics = null);

public sealed record AiPursuitLaneChangeDiagnostics(
    long Revision,
    AiPursuitLaneChangeEventKind EventKind,
    int FromPointId,
    int ToPointId,
    AiLaneChangeDirection Direction,
    long RouteRevision,
    float? DistanceToDecisionMeters,
    string? BlockingReason);

internal sealed record AiPursuitSnapshot(
    byte TargetSessionId,
    Vector3 TargetPosition,
    AiPursuitTrackingOptions Options,
    AiPursuitRouteState NavigationState,
    float? DesiredSpeedMetersPerSecond);

public static class AiPursuitControl
{
    public static bool ShouldRetain(float distanceSquared, float maximumDistanceMeters)
    {
        ValidateMaximumDistance(maximumDistanceMeters);
        return distanceSquared <= maximumDistanceMeters * maximumDistanceMeters;
    }

    public static float ResolveRequestedSpeed(float nativeSpeed, float? desiredSpeed)
    {
        if (desiredSpeed.HasValue)
            ValidateDesiredSpeed(desiredSpeed.Value);
        return desiredSpeed ?? nativeSpeed;
    }

    public static float ApplySafetyLimit(float requestedSpeed, float safetyLimit) =>
        Math.Min(requestedSpeed, safetyLimit);

    public static AiPursuitRouteDiagnostics CreateRouteDiagnostics(
        AiPursuitNavigationResult navigation,
        int policePointId,
        Func<int, int> getJunctionEndPoint)
    {
        ArgumentNullException.ThrowIfNull(navigation);
        ArgumentNullException.ThrowIfNull(getJunctionEndPoint);
        if (navigation.Status != AiPursuitNavigationStatus.Active
            || navigation.State == null
            || !navigation.UpdateKind.HasValue)
        {
            throw new ArgumentException("Active navigation result required", nameof(navigation));
        }

        var decisions = navigation.State.Plan.JunctionDecisions
            .OrderBy(decision => decision.Key)
            .Select(decision => new AiPursuitJunctionDecision(
                decision.Key,
                decision.Value,
                getJunctionEndPoint(decision.Key)))
            .ToArray();
        return new AiPursuitRouteDiagnostics(
            navigation.State.Revision,
            navigation.UpdateKind.Value,
            policePointId,
            navigation.State.TargetPointId,
            navigation.State.Plan.DistanceMeters,
            navigation.VisitedNodes,
            decisions);
    }

    public static AiPursuitSearchDiagnostics CreateSearchDiagnostics(
        AiPursuitNavigationResult navigation,
        int policePointId,
        int? previousTargetPointId)
    {
        ArgumentNullException.ThrowIfNull(navigation);
        var location = navigation.TargetLocationDiagnostics;
        return new AiPursuitSearchDiagnostics(
            policePointId,
            previousTargetPointId,
            navigation.Status == AiPursuitNavigationStatus.Active
                ? navigation.State?.TargetPointId
                : null,
            location.SpatialPointIds,
            location.LaneEquivalentPointIds,
            location.Rejections,
            navigation.SearchFailure,
            navigation.VisitedNodes,
            navigation.MaximumExploredDistanceMeters,
            navigation.JunctionEdgesExamined);
    }

    public static void ValidateDesiredSpeed(float speed)
    {
        if (!float.IsFinite(speed) || speed < 0)
            throw new ArgumentOutOfRangeException(nameof(speed));
    }

    public static void ValidateMaximumDistance(float maximumDistanceMeters)
    {
        if (!float.IsFinite(maximumDistanceMeters) || maximumDistanceMeters <= 0)
            throw new ArgumentOutOfRangeException(nameof(maximumDistanceMeters));
    }

    public static void ValidateTrackingOptions(AiPursuitTrackingOptions options)
    {
        ArgumentNullException.ThrowIfNull(options);
        ValidateMaximumDistance(options.MaximumSpatialDistanceMeters);
        if (!float.IsFinite(options.MaximumRouteDistanceMeters)
            || options.MaximumRouteDistanceMeters <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(options.MaximumRouteDistanceMeters));
        }
        if (options.MaximumVisitedNodes <= 0)
            throw new ArgumentOutOfRangeException(nameof(options.MaximumVisitedNodes));
        if (options.RouteGraceMilliseconds < 0)
            throw new ArgumentOutOfRangeException(nameof(options.RouteGraceMilliseconds));
        if (options.LaneChange is { Enabled: true } laneChange)
        {
            if (!float.IsFinite(laneChange.DistanceMeters)
                || laneChange.DistanceMeters <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(laneChange.DistanceMeters));
            }
            if (laneChange.CooldownMilliseconds < 0)
                throw new ArgumentOutOfRangeException(nameof(laneChange.CooldownMilliseconds));
        }
    }
}
