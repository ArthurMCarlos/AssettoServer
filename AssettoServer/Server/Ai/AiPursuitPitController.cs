using System;

namespace AssettoServer.Server.Ai;

public sealed record AiPursuitPitControllerState(
    AiPursuitPitPhase Phase,
    byte TargetSessionId,
    long RouteRevision,
    AiPursuitPitSide? Side,
    long AttemptStartedAtMilliseconds,
    long CooldownUntilMilliseconds,
    long DiagnosticRevision,
    int CooldownMilliseconds,
    float LastClearanceMeters,
    float LastClosingSpeedMetersPerSecond,
    float LastOffsetMeters)
{
    public static AiPursuitPitControllerState Idle(byte target, long routeRevision) =>
        new(AiPursuitPitPhase.Idle, target, routeRevision, null, 0, 0, 0, 0, 0, 0, 0);
}

public sealed record AiPursuitPitRequest(
    AiPursuitPitOptions Options,
    byte TargetSessionId,
    bool RouteValid,
    bool SameLane,
    bool PoliceBehindTarget,
    float PhysicalClearanceMeters,
    float ClosingSpeedMetersPerSecond,
    AiPursuitDrivingState DrivingState,
    AiPursuitDrivingReason DrivingReason,
    AiLaneChangePhase LaneChangePhase,
    bool JunctionNear,
    bool LeftSafe,
    bool RightSafe,
    long RouteRevision,
    long NowMilliseconds,
    AiPursuitPitControllerState? Previous)
{
    public AiPursuitPitContinuityDiagnostics? Continuity { get; init; }
}

public sealed record AiPursuitPitDecision(
    AiPursuitPitControllerState State,
    float DesiredOffsetMeters,
    AiPursuitPitDiagnostics? Diagnostics)
{
    public AiPursuitPitAbortReason? EligibilityRejection { get; init; }
}

public sealed class AiPursuitPitController
{
    public AiPursuitPitDecision Update(AiPursuitPitRequest request)
    {
        ArgumentNullException.ThrowIfNull(request);
        var previous = request.Previous ?? AiPursuitPitControllerState.Idle(
            request.TargetSessionId, request.RouteRevision);
        if (previous.Phase == AiPursuitPitPhase.Cooldown)
        {
            if (request.NowMilliseconds < previous.CooldownUntilMilliseconds)
                return new AiPursuitPitDecision(previous, 0, null);
            return new AiPursuitPitDecision(previous with
            {
                Phase = AiPursuitPitPhase.Idle,
                Side = null,
                TargetSessionId = request.TargetSessionId,
                RouteRevision = request.RouteRevision
            }, 0, null);
        }

        var invalid = GetAbortReason(request, previous);
        if (invalid != AiPursuitPitAbortReason.None)
        {
            if (previous.Phase is AiPursuitPitPhase.Armed or AiPursuitPitPhase.Attempting)
                return Finish(request, previous, AiPursuitPitEventKind.Aborted, invalid);
            return new AiPursuitPitDecision(previous with
            {
                Phase = AiPursuitPitPhase.Idle,
                Side = null,
                TargetSessionId = request.TargetSessionId,
                RouteRevision = request.RouteRevision
            }, 0, null) { EligibilityRejection = invalid };
        }

        if (previous.Phase == AiPursuitPitPhase.Attempting)
        {
            if (request.NowMilliseconds - previous.AttemptStartedAtMilliseconds
                >= request.Options.CommitMilliseconds)
                return Finish(request, previous, AiPursuitPitEventKind.Aborted,
                    AiPursuitPitAbortReason.CommitElapsed);
            var continuing = previous with
            {
                LastClearanceMeters = request.PhysicalClearanceMeters,
                LastClosingSpeedMetersPerSecond = request.ClosingSpeedMetersPerSecond
            };
            return new AiPursuitPitDecision(continuing,
                SignedOffset(continuing.Side!.Value, request.Options.LateralOffsetMeters), null);
        }

        if (previous.Phase == AiPursuitPitPhase.Armed)
        {
            var started = previous with
            {
                Phase = AiPursuitPitPhase.Attempting,
                AttemptStartedAtMilliseconds = request.NowMilliseconds,
                LastClearanceMeters = request.PhysicalClearanceMeters,
                LastClosingSpeedMetersPerSecond = request.ClosingSpeedMetersPerSecond,
                LastOffsetMeters = SignedOffset(previous.Side!.Value,
                    request.Options.LateralOffsetMeters),
                DiagnosticRevision = previous.DiagnosticRevision + 1
            };
            return new AiPursuitPitDecision(started,
                SignedOffset(started.Side!.Value, request.Options.LateralOffsetMeters),
                Diagnostics(request, started, AiPursuitPitEventKind.Started,
                    AiPursuitPitAbortReason.None));
        }

        var side = request.LeftSafe ? AiPursuitPitSide.Left : AiPursuitPitSide.Right;
        var armed = previous with
        {
            Phase = AiPursuitPitPhase.Armed,
            TargetSessionId = request.TargetSessionId,
            RouteRevision = request.RouteRevision,
            Side = side,
            CooldownMilliseconds = request.Options.CooldownMilliseconds,
            LastClearanceMeters = request.PhysicalClearanceMeters,
            LastClosingSpeedMetersPerSecond = request.ClosingSpeedMetersPerSecond,
            LastOffsetMeters = 0,
            DiagnosticRevision = previous.DiagnosticRevision + 1
        };
        return new AiPursuitPitDecision(armed, 0,
            Diagnostics(request, armed, AiPursuitPitEventKind.Armed,
                AiPursuitPitAbortReason.None));
    }

    public AiPursuitPitDecision ReportCollision(
        AiPursuitPitControllerState state,
        long nowMilliseconds)
    {
        ArgumentNullException.ThrowIfNull(state);
        if (state.Phase != AiPursuitPitPhase.Attempting)
            return new AiPursuitPitDecision(state, 0, null);
        var finished = state with
        {
            Phase = AiPursuitPitPhase.Cooldown,
            Side = null,
            CooldownUntilMilliseconds = nowMilliseconds + state.CooldownMilliseconds,
            DiagnosticRevision = state.DiagnosticRevision + 1
        };
        return new AiPursuitPitDecision(finished, 0,
            new AiPursuitPitDiagnostics(finished.DiagnosticRevision,
                AiPursuitPitEventKind.Contact, state.Side,
                AiPursuitPitAbortReason.None, state.LastClearanceMeters,
                state.LastClosingSpeedMetersPerSecond, state.LastOffsetMeters));
    }

    public static float ApproachOffset(float current, float desired, float maximumStep)
    {
        if (!float.IsFinite(current) || !float.IsFinite(desired)
            || !float.IsFinite(maximumStep) || maximumStep < 0)
            throw new ArgumentOutOfRangeException(nameof(maximumStep));
        return current + Math.Clamp(desired - current, -maximumStep, maximumStep);
    }

    private static AiPursuitPitAbortReason GetAbortReason(
        AiPursuitPitRequest request, AiPursuitPitControllerState previous)
    {
        if (!request.Options.Enabled) return AiPursuitPitAbortReason.Disabled;
        if (previous.Phase != AiPursuitPitPhase.Idle
            && previous.TargetSessionId != request.TargetSessionId)
            return AiPursuitPitAbortReason.TargetChanged;
        // Replanning changes revision during normal pursuit. Revalidate the current
        // navigation and physical safety gates, not the identity of the armed route.
        if (!request.RouteValid)
            return AiPursuitPitAbortReason.RouteLost;
        if (request.LaneChangePhase is AiLaneChangePhase.WaitingForGap
            or AiLaneChangePhase.Changing)
            return AiPursuitPitAbortReason.LaneChange;
        if (request.JunctionNear) return AiPursuitPitAbortReason.Junction;
        if (request.DrivingState == AiPursuitDrivingState.Recovery)
            return AiPursuitPitAbortReason.Recovery;
        if (request.DrivingReason == AiPursuitDrivingReason.ExcessClosingSpeed)
            return AiPursuitPitAbortReason.ClosingSpeedUnsafe;
        if (request.DrivingState is not (AiPursuitDrivingState.ClosePressure
            or AiPursuitDrivingState.Contact))
            return AiPursuitPitAbortReason.OutOfRange;
        if (!request.SameLane || !request.PoliceBehindTarget)
            return AiPursuitPitAbortReason.GeometryInvalid;
        if (!float.IsFinite(request.PhysicalClearanceMeters)
            || request.PhysicalClearanceMeters < 0
            || request.PhysicalClearanceMeters > request.Options.MaxDistanceMeters)
            return AiPursuitPitAbortReason.OutOfRange;
        if (!float.IsFinite(request.ClosingSpeedMetersPerSecond)
            || request.ClosingSpeedMetersPerSecond < 0
            || request.ClosingSpeedMetersPerSecond > request.Options.MaxClosingSpeedMetersPerSecond)
            return AiPursuitPitAbortReason.ClosingSpeedUnsafe;
        if (previous.Side == AiPursuitPitSide.Left && !request.LeftSafe
            || previous.Side == AiPursuitPitSide.Right && !request.RightSafe
            || previous.Side == null && !request.LeftSafe && !request.RightSafe)
            return AiPursuitPitAbortReason.BlockedSide;
        return AiPursuitPitAbortReason.None;
    }

    private static AiPursuitPitDecision Finish(AiPursuitPitRequest request,
        AiPursuitPitControllerState previous, AiPursuitPitEventKind kind,
        AiPursuitPitAbortReason reason)
    {
        var finished = previous with
        {
            Phase = AiPursuitPitPhase.Cooldown,
            Side = null,
            CooldownUntilMilliseconds = request.NowMilliseconds + request.Options.CooldownMilliseconds,
            DiagnosticRevision = previous.DiagnosticRevision + 1
        };
        return new AiPursuitPitDecision(finished, 0,
            Diagnostics(request, finished, kind, reason, previous.Side));
    }

    private static AiPursuitPitDiagnostics Diagnostics(
        AiPursuitPitRequest request, AiPursuitPitControllerState state,
        AiPursuitPitEventKind kind, AiPursuitPitAbortReason reason,
        AiPursuitPitSide? side = null) =>
        new(state.DiagnosticRevision, kind, side ?? state.Side, reason,
            request.PhysicalClearanceMeters, request.ClosingSpeedMetersPerSecond,
            kind == AiPursuitPitEventKind.Started
                ? SignedOffset(state.Side!.Value, request.Options.LateralOffsetMeters)
                : 0)
        {
            Continuity = request.Continuity
        };

    private static float SignedOffset(AiPursuitPitSide side, float offset) =>
        side == AiPursuitPitSide.Left ? offset : -offset;
}
