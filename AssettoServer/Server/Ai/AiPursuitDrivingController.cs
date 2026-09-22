using System;

namespace AssettoServer.Server.Ai;

public sealed record AiPursuitDrivingControllerState(
    AiPursuitDrivingState State,
    AiPursuitDrivingReason Reason,
    float LastRequestedSpeedMetersPerSecond,
    long DiagnosticRevision,
    bool RecoveryActive,
    long LastCollisionMilliseconds,
    long RouteRevision);

public sealed record AiPursuitDrivingRequest(
    AiPursuitDrivingOptions Options,
    float RouteDistanceMeters,
    float PhysicalClearanceMeters,
    float TargetSpeedMetersPerSecond,
    float PoliceSpeedMetersPerSecond,
    AiLaneChangePhase LaneChangePhase,
    long RouteRevision,
    long NowMilliseconds,
    AiPursuitDrivingControllerState? PreviousState);

public sealed record AiPursuitDrivingDecision(
    float RequestedSpeedMetersPerSecond,
    AiPursuitDrivingControllerState State,
    AiPursuitDrivingDiagnostics Diagnostics);

public sealed class AiPursuitDrivingController
{
    private const float HysteresisFactor = 0.1f;
    private const float ClosingSpeedDeadbandMetersPerSecond = 0.5f;
    private const float LaneChangeClosingSpeedFactor = 0.5f;

    public AiPursuitDrivingDecision Update(AiPursuitDrivingRequest request)
    {
        ArgumentNullException.ThrowIfNull(request);
        AiPursuitControl.ValidateDrivingOptions(request.Options);
        if (!IsValidMeasurement(request.RouteDistanceMeters)
            || !IsValidMeasurement(request.PhysicalClearanceMeters)
            || !IsValidMeasurement(request.TargetSpeedMetersPerSecond)
            || !IsValidMeasurement(request.PoliceSpeedMetersPerSecond))
        {
            return InvalidMeasurementDecision(request);
        }

        var options = request.Options;
        var previous = request.PreviousState;
        var closingSpeed = request.PoliceSpeedMetersPerSecond
                           - request.TargetSpeedMetersPerSecond;
        var state = SelectState(request, closingSpeed);
        var desiredClosingSpeed = GetDesiredClosingSpeed(
            state,
            request.RouteDistanceMeters,
            request.PhysicalClearanceMeters,
            options);

        if (request.LaneChangePhase == AiLaneChangePhase.Changing)
            desiredClosingSpeed *= LaneChangeClosingSpeedFactor;

        var reason = GetReason(state, options.ContactEnabled);
        if (state != AiPursuitDrivingState.Recovery
            && closingSpeed > desiredClosingSpeed + ClosingSpeedDeadbandMetersPerSecond)
        {
            reason = AiPursuitDrivingReason.ExcessClosingSpeed;
        }
        else if (state != AiPursuitDrivingState.Recovery
                 && request.LaneChangePhase == AiLaneChangePhase.Changing)
        {
            reason = AiPursuitDrivingReason.LaneChangeLimited;
        }

        var requestedSpeed = request.TargetSpeedMetersPerSecond + desiredClosingSpeed;
        if (previous != null
            && MathF.Abs(closingSpeed - desiredClosingSpeed)
                <= ClosingSpeedDeadbandMetersPerSecond)
        {
            requestedSpeed = previous.LastRequestedSpeedMetersPerSecond;
        }
        requestedSpeed = Math.Clamp(requestedSpeed, 0, options.MaximumSpeedMetersPerSecond);

        var recoveryActive = state == AiPursuitDrivingState.Recovery;
        var revision = previous?.DiagnosticRevision ?? 0;
        if (previous == null
            || previous.State != state
            || previous.Reason != reason
            || previous.RecoveryActive != recoveryActive)
        {
            revision++;
        }

        var controllerState = new AiPursuitDrivingControllerState(
            state,
            reason,
            requestedSpeed,
            revision,
            recoveryActive,
            previous?.LastCollisionMilliseconds ?? 0,
            request.RouteRevision);
        var diagnostics = new AiPursuitDrivingDiagnostics(
            revision,
            state,
            reason,
            request.RouteDistanceMeters,
            request.PhysicalClearanceMeters,
            request.TargetSpeedMetersPerSecond,
            request.PoliceSpeedMetersPerSecond,
            closingSpeed,
            desiredClosingSpeed,
            requestedSpeed,
            false);
        return new AiPursuitDrivingDecision(requestedSpeed, controllerState, diagnostics);
    }

    public AiPursuitDrivingControllerState ReportCollision(
        AiPursuitDrivingControllerState state,
        long nowMilliseconds)
    {
        ArgumentNullException.ThrowIfNull(state);
        if (nowMilliseconds < 0)
            throw new ArgumentOutOfRangeException(nameof(nowMilliseconds));

        return state with
        {
            State = AiPursuitDrivingState.Recovery,
            Reason = AiPursuitDrivingReason.CollisionRecovery,
            DiagnosticRevision = state.DiagnosticRevision + 1,
            RecoveryActive = true,
            LastCollisionMilliseconds = nowMilliseconds
        };
    }

    private static AiPursuitDrivingState SelectState(
        AiPursuitDrivingRequest request,
        float closingSpeed)
    {
        var options = request.Options;
        var previous = request.PreviousState;
        if (previous?.RecoveryActive == true
            && request.RouteDistanceMeters < options.CatchUpDistanceMeters
            && request.PhysicalClearanceMeters <= options.ContactDistanceMeters + 2
            && closingSpeed >= -ClosingSpeedDeadbandMetersPerSecond)
        {
            return AiPursuitDrivingState.Recovery;
        }

        if (previous?.State == AiPursuitDrivingState.Contact
            && request.PhysicalClearanceMeters
                <= options.ContactDistanceMeters * (1 + HysteresisFactor))
        {
            return AiPursuitDrivingState.Contact;
        }
        if (request.PhysicalClearanceMeters <= options.ContactDistanceMeters)
            return AiPursuitDrivingState.Contact;

        if (previous?.State == AiPursuitDrivingState.ClosePressure
            && request.PhysicalClearanceMeters
                <= options.CloseDistanceMeters * (1 + HysteresisFactor))
        {
            return AiPursuitDrivingState.ClosePressure;
        }
        if (request.PhysicalClearanceMeters <= options.CloseDistanceMeters)
            return AiPursuitDrivingState.ClosePressure;

        if (previous?.State == AiPursuitDrivingState.CatchUp
            && request.RouteDistanceMeters
                >= options.CatchUpDistanceMeters * (1 - HysteresisFactor))
        {
            return AiPursuitDrivingState.CatchUp;
        }
        return request.RouteDistanceMeters >= options.CatchUpDistanceMeters
            ? AiPursuitDrivingState.CatchUp
            : AiPursuitDrivingState.Approach;
    }

    private static float GetDesiredClosingSpeed(
        AiPursuitDrivingState state,
        float routeDistance,
        float clearance,
        AiPursuitDrivingOptions options)
    {
        var closePressureSpeed = Math.Min(
            options.MaximumClosingSpeedMetersPerSecond,
            Math.Max(
                options.ContactClosingSpeedMetersPerSecond,
                options.MaximumClosingSpeedMetersPerSecond * 0.25f));
        return state switch
        {
            AiPursuitDrivingState.CatchUp => options.MaximumClosingSpeedMetersPerSecond,
            AiPursuitDrivingState.Approach => Lerp(
                closePressureSpeed,
                options.MaximumClosingSpeedMetersPerSecond,
                Normalize(
                    routeDistance,
                    options.CloseDistanceMeters,
                    options.CatchUpDistanceMeters)),
            AiPursuitDrivingState.ClosePressure => Lerp(
                options.ContactEnabled
                    ? options.ContactClosingSpeedMetersPerSecond
                    : 0,
                closePressureSpeed,
                Normalize(
                    clearance,
                    options.ContactDistanceMeters,
                    options.CloseDistanceMeters)),
            AiPursuitDrivingState.Contact => options.ContactEnabled
                ? options.ContactClosingSpeedMetersPerSecond
                : 0,
            AiPursuitDrivingState.Recovery => 0,
            _ => throw new ArgumentOutOfRangeException(nameof(state), state, null)
        };
    }

    private static AiPursuitDrivingReason GetReason(
        AiPursuitDrivingState state,
        bool contactEnabled) =>
        state switch
        {
            AiPursuitDrivingState.CatchUp => AiPursuitDrivingReason.DistanceCatchUp,
            AiPursuitDrivingState.Approach => AiPursuitDrivingReason.DistanceApproach,
            AiPursuitDrivingState.ClosePressure => AiPursuitDrivingReason.ClosePressure,
            AiPursuitDrivingState.Contact => contactEnabled
                ? AiPursuitDrivingReason.ContactPressure
                : AiPursuitDrivingReason.ContactDisabled,
            AiPursuitDrivingState.Recovery => AiPursuitDrivingReason.CollisionRecovery,
            _ => throw new ArgumentOutOfRangeException(nameof(state), state, null)
        };

    private static float Normalize(float value, float minimum, float maximum) =>
        Math.Clamp((value - minimum) / (maximum - minimum), 0, 1);

    private static float Lerp(float start, float end, float amount) =>
        start + (end - start) * amount;

    private static AiPursuitDrivingDecision InvalidMeasurementDecision(
        AiPursuitDrivingRequest request)
    {
        var previous = request.PreviousState;
        var state = previous?.State ?? AiPursuitDrivingState.Approach;
        var revision = (previous?.DiagnosticRevision ?? 0)
                       + (previous?.Reason == AiPursuitDrivingReason.InvalidMeasurement ? 0 : 1);
        var controllerState = new AiPursuitDrivingControllerState(
            state,
            AiPursuitDrivingReason.InvalidMeasurement,
            0,
            revision,
            previous?.RecoveryActive ?? false,
            previous?.LastCollisionMilliseconds ?? 0,
            request.RouteRevision);
        var diagnostics = new AiPursuitDrivingDiagnostics(
            revision,
            state,
            AiPursuitDrivingReason.InvalidMeasurement,
            FiniteOrZero(request.RouteDistanceMeters),
            FiniteOrZero(request.PhysicalClearanceMeters),
            FiniteOrZero(request.TargetSpeedMetersPerSecond),
            FiniteOrZero(request.PoliceSpeedMetersPerSecond),
            0,
            0,
            0,
            false);
        return new AiPursuitDrivingDecision(0, controllerState, diagnostics);
    }

    private static bool IsValidMeasurement(float value) => float.IsFinite(value) && value >= 0;

    private static float FiniteOrZero(float value) => IsValidMeasurement(value) ? value : 0;
}
