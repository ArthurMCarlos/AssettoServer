using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using AssettoServer.Server.Ai.Routing;
using AssettoServer.Utils;

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
    int CooldownMilliseconds,
    float LookaheadMeters = 1000);

public sealed record AiPursuitTrackingOptions(
    float MaximumSpatialDistanceMeters,
    float MaximumRouteDistanceMeters,
    int MaximumVisitedNodes,
    int RouteGraceMilliseconds,
    AiPursuitLaneChangeOptions? LaneChange = null,
    AiPursuitDrivingOptions? Driving = null);

public enum AiPursuitDrivingState
{
    CatchUp,
    Approach,
    ClosePressure,
    Contact,
    Recovery
}

public enum AiPursuitDrivingReason
{
    DistanceCatchUp,
    DistanceApproach,
    ClosePressure,
    ContactPressure,
    ContactDisabled,
    ExcessClosingSpeed,
    LaneChangeLimited,
    CollisionRecovery,
    InvalidMeasurement
}

internal enum AiCollisionDisposition
{
    PursuitRecovery,
    NativeStop
}

public sealed record AiPursuitDrivingOptions(
    bool Enabled,
    bool ContactEnabled,
    float CatchUpDistanceMeters,
    float CloseDistanceMeters,
    float ContactDistanceMeters,
    float MaximumSpeedMetersPerSecond,
    float MaximumClosingSpeedMetersPerSecond,
    float ContactClosingSpeedMetersPerSecond,
    AiPursuitPitOptions? Pit = null);

public sealed record AiPursuitPitOptions(
    bool Enabled,
    float MaxDistanceMeters,
    float MaxClosingSpeedMetersPerSecond,
    float LateralOffsetMeters,
    int CommitMilliseconds,
    int CooldownMilliseconds);

public enum AiPursuitPitPhase { Idle, Armed, Attempting, Cooldown }
public enum AiPursuitPitSide { Left, Right }
public enum AiPursuitPitEventKind { Armed, Started, Aborted, Contact }
public enum AiPursuitPitAbortReason
{
    None, Disabled, RouteLost, TargetChanged, OutOfRange, ClosingSpeedUnsafe,
    BlockedSide, LaneChange, Junction, Recovery, GeometryInvalid, CommitElapsed
}

public sealed record AiPursuitPitDiagnostics(
    long Revision,
    AiPursuitPitEventKind EventKind,
    AiPursuitPitSide? Side,
    AiPursuitPitAbortReason Reason,
    float PhysicalClearanceMeters,
    float ClosingSpeedMetersPerSecond,
    float OffsetMeters);

// Temporary, observational PIT gate diagnostics. Never feeds the PIT controller.
public sealed record AiPursuitPitEligibilityDiagnostics(
    AiPursuitPitPhase Phase,
    AiPursuitPitAbortReason? RejectionReason,
    bool NavigationActive,
    bool LaneFitsOffset,
    bool OffsetReady,
    bool TargetAligned,
    float LaneWidthMeters,
    float CurrentOffsetMeters,
    float TargetLongitudinalMeters,
    float TargetLateralMeters,
    float HeadingDot,
    bool JunctionNear,
    bool LeftSafe,
    bool RightSafe,
    float PhysicalClearanceMeters,
    float ClosingSpeedMetersPerSecond,
    AiPursuitDrivingState DrivingState,
    AiPursuitDrivingReason DrivingReason)
{
    public long RouteRevision { get; init; }
    public AiLaneChangePhase LaneChangePhase { get; init; }
    public AiPursuitPitSideSafetyResult? SideSafety { get; init; }
}

public sealed record AiPursuitDrivingDiagnostics(
    long Revision,
    AiPursuitDrivingState State,
    AiPursuitDrivingReason Reason,
    float RouteDistanceMeters,
    float PhysicalClearanceMeters,
    float TargetSpeedMetersPerSecond,
    float PoliceSpeedMetersPerSecond,
    float ClosingSpeedMetersPerSecond,
    float DesiredClosingSpeedMetersPerSecond,
    float RequestedSpeedMetersPerSecond,
    bool CollisionReported);

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
    AiPursuitLaneChangeDiagnostics? LaneChangeDiagnostics = null,
    AiPursuitDrivingDiagnostics? DrivingDiagnostics = null,
    AiPursuitPitDiagnostics? PitDiagnostics = null,
    AiPursuitPitEligibilityDiagnostics? PitEligibilityDiagnostics = null);

public sealed record AiPursuitLaneChangeDiagnostics(
    long Revision,
    AiPursuitLaneChangeEventKind EventKind,
    int? FromPointId,
    int? ToPointId,
    AiLaneChangeDirection? Direction,
    long RouteRevision,
    float? DistanceToDecisionMeters)
{
    public AiPursuitLaneChangeDiagnosticReason Reason { get; init; }
    public int PolicePointId { get; init; }
    public int? PreferredPhysicalTargetPointId { get; init; }
    public int? JunctionId { get; init; }
    public AiPursuitLaneMotivation? Motivation { get; init; }
    public AiPursuitLanePhysicalRelation? PhysicalRelation { get; init; }
    public AiPursuitLaneRouteDiagnostic? CurrentLaneRoute { get; init; }
    public IReadOnlyList<AiPursuitLaneRouteDiagnostic> CandidateLaneRoutes { get; init; } = [];
    public AiLaneChangeSafetyStatus? SafetyStatus { get; init; }
    public float? RequiredTransitionDistanceMeters { get; init; }
    public float? SourceAvailableDistanceMeters { get; init; }
    public float? DestinationAvailableDistanceMeters { get; init; }
}

public enum AiPursuitLaneChangeDiagnosticReason
{
    Disabled,
    NoPhysicalTarget,
    CurrentLaneValid,
    NoAdjacentLane,
    NonAdjacent,
    OppositeDirection,
    InvalidGeometry,
    NoForwardRoute,
    NoRealJunction,
    BeyondLookahead,
    InsufficientPreparationDistance,
    Cooldown,
    ObstacleAhead,
    ObstacleAlongside,
    ObstacleBehind,
    RouteRevisionChanged,
    RoutePreparation,
    Requested,
    Started,
    Completed,
    Cancelled
}

internal sealed record AiPursuitSnapshot(
    byte TargetSessionId,
    Vector3 TargetPosition,
    AiPursuitTrackingOptions Options,
    AiPursuitRouteState NavigationState,
    float? DesiredSpeedMetersPerSecond,
    AiPursuitDrivingControllerState? DrivingState = null,
    AiPursuitDrivingDiagnostics? DrivingDiagnostics = null,
    AiPursuitPitControllerState? PitState = null,
    AiPursuitPitDiagnostics? PitDiagnostics = null);

internal sealed class AiPursuitUpdateGate
{
    private readonly object _sync = new();

    public TResult Run<TResult>(Func<TResult> update)
    {
        lock (_sync)
            return update();
    }

    public void Run(Action update)
    {
        lock (_sync)
            update();
    }
}

public static class AiPursuitControl
{
    private const float WalkingSpeedMetersPerSecond = 10 / 3.6f;

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

    public static float CalculatePhysicalClearance(
        float centerDistance,
        float policeFrontLength,
        float targetRearLength)
    {
        ValidateNonNegativeFinite(centerDistance, nameof(centerDistance));
        ValidateNonNegativeFinite(policeFrontLength, nameof(policeFrontLength));
        ValidateNonNegativeFinite(targetRearLength, nameof(targetRearLength));
        return Math.Max(0, centerDistance - policeFrontLength - targetRearLength);
    }

    internal static bool HasFiniteTrackingMeasurements(
        Vector3 policePosition,
        Vector3 targetPosition,
        Vector3 targetVelocity,
        float policeSpeed) =>
        float.IsFinite(Vector3.DistanceSquared(policePosition, targetPosition))
        && float.IsFinite(targetVelocity.Length())
        && float.IsFinite(policeSpeed)
        && policeSpeed >= 0;

    public static bool IsControlledTargetObstacle(
        byte? pursuitTargetSessionId,
        byte obstacleSessionId,
        bool aggressiveDrivingEnabled) =>
        aggressiveDrivingEnabled
        && pursuitTargetSessionId.HasValue
        && pursuitTargetSessionId.Value == obstacleSessionId;

    internal static bool ShouldReplaceClosestPlayerObstacle(
        byte? exemptTargetSessionId,
        byte candidateSessionId,
        float candidateDistanceSquared,
        bool isAhead,
        float closestDistanceSquared) =>
        candidateSessionId != exemptTargetSessionId
        && isAhead
        && float.IsFinite(candidateDistanceSquared)
        && candidateDistanceSquared >= 0
        && candidateDistanceSquared < closestDistanceSquared;

    internal static AiPursuitDrivingDiagnostics? SelectDrivingDiagnosticsForDelivery(
        AiPursuitDrivingDiagnostics? previous,
        AiPursuitDrivingDiagnostics? current) =>
        previous?.CollisionReported == true ? previous : current;

    internal static AiPursuitPitDiagnostics? SelectPitDiagnosticsForDelivery(
        AiPursuitPitDiagnostics? previous,
        AiPursuitPitDiagnostics? current) =>
        previous?.EventKind == AiPursuitPitEventKind.Contact ? previous : current;

    internal static AiCollisionDisposition ResolveCollisionDisposition(
        byte? pursuitTargetSessionId,
        bool aggressiveDrivingEnabled,
        bool hasDrivingState,
        byte senderSessionId) =>
        aggressiveDrivingEnabled
        && hasDrivingState
        && pursuitTargetSessionId == senderSessionId
            ? AiCollisionDisposition.PursuitRecovery
            : AiCollisionDisposition.NativeStop;

    public static float? ResolvePlayerObstacleSpeed(
        float currentSpeed,
        float playerSpeed,
        float playerDistance,
        float minimumObstacleDistance,
        float deceleration,
        bool controlledTarget)
    {
        ValidateNonNegativeFinite(currentSpeed, nameof(currentSpeed));
        ValidateNonNegativeFinite(playerSpeed, nameof(playerSpeed));
        ValidateNonNegativeFinite(playerDistance, nameof(playerDistance));
        ValidateNonNegativeFinite(minimumObstacleDistance, nameof(minimumObstacleDistance));
        if (!float.IsFinite(deceleration) || deceleration <= 0)
            throw new ArgumentOutOfRangeException(nameof(deceleration));
        if (controlledTarget)
            return null;
        if (playerDistance < minimumObstacleDistance)
            return 0;

        if (playerSpeed < 0.1f)
            playerSpeed = 0;
        if ((playerSpeed < currentSpeed || playerSpeed == 0)
            && playerDistance < PhysicsUtils.CalculateBrakingDistance(
                currentSpeed - playerSpeed,
                deceleration) * 2 + 20)
        {
            return Math.Max(WalkingSpeedMetersPerSecond, playerSpeed);
        }
        return null;
    }

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

    private static void ValidateNonNegativeFinite(float value, string parameterName)
    {
        if (!float.IsFinite(value) || value < 0)
            throw new ArgumentOutOfRangeException(parameterName);
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
            if (!float.IsFinite(laneChange.LookaheadMeters)
                || laneChange.LookaheadMeters < 100
                || laneChange.LookaheadMeters > 5000
                || laneChange.LookaheadMeters < laneChange.DistanceMeters)
            {
                throw new ArgumentOutOfRangeException(nameof(laneChange.LookaheadMeters));
            }
        }
        if (options.Driving is { Enabled: true } driving)
            ValidateDrivingOptions(driving);
    }

    public static void ValidateDrivingOptions(AiPursuitDrivingOptions options)
    {
        ArgumentNullException.ThrowIfNull(options);
        if (!float.IsFinite(options.CatchUpDistanceMeters)
            || !float.IsFinite(options.CloseDistanceMeters)
            || !float.IsFinite(options.ContactDistanceMeters)
            || options.CatchUpDistanceMeters <= options.CloseDistanceMeters
            || options.CloseDistanceMeters <= options.ContactDistanceMeters
            || options.ContactDistanceMeters <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(options));
        }
        if (!float.IsFinite(options.MaximumSpeedMetersPerSecond)
            || options.MaximumSpeedMetersPerSecond <= 0
            || !float.IsFinite(options.MaximumClosingSpeedMetersPerSecond)
            || options.MaximumClosingSpeedMetersPerSecond < 0
            || !float.IsFinite(options.ContactClosingSpeedMetersPerSecond)
            || options.ContactClosingSpeedMetersPerSecond < 0
            || options.ContactClosingSpeedMetersPerSecond
                > options.MaximumClosingSpeedMetersPerSecond)
        {
            throw new ArgumentOutOfRangeException(nameof(options));
        }
        if (options.Pit is { Enabled: true } pit
            && (!options.Enabled
                || !options.ContactEnabled
                || !float.IsFinite(pit.MaxDistanceMeters)
                || pit.MaxDistanceMeters <= options.ContactDistanceMeters
                || pit.MaxDistanceMeters > options.CloseDistanceMeters
                || !float.IsFinite(pit.MaxClosingSpeedMetersPerSecond)
                || pit.MaxClosingSpeedMetersPerSecond <= 0
                || pit.MaxClosingSpeedMetersPerSecond > options.MaximumClosingSpeedMetersPerSecond
                || !float.IsFinite(pit.LateralOffsetMeters)
                || pit.LateralOffsetMeters <= 0
                || pit.LateralOffsetMeters > 1.1f
                || pit.CommitMilliseconds < 200
                || pit.CommitMilliseconds > 5000
                || pit.CooldownMilliseconds < 0
                || pit.CooldownMilliseconds > 30000))
        {
            throw new ArgumentOutOfRangeException(nameof(options.Pit));
        }
    }
}
