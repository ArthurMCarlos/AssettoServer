using System;

namespace AssettoServer.Server.Ai;

public sealed record AiRecoveryAssistInput(
    float PhysicalClearanceMeters, float RouteDistanceMeters,
    float TargetSpeedMetersPerSecond, float PoliceSpeedMetersPerSecond,
    float BaseRequestedSpeedMetersPerSecond, float NativeAcceleration,
    float Deceleration, bool NavigationActive, bool RecoveryActive)
{
    public float CorneringBrakeForceFactor { get; init; } = 1;
}

public sealed record AiRecoveryAssistDecision(
    float RequestedSpeedMetersPerSecond, float AccelerationLimit, bool Active);

public static class AiPursuitRecoveryAssist
{
    public static float UnavailableSpeed(float? previous, float target) =>
        float.IsFinite(target) && target >= 0 && previous.HasValue && float.IsFinite(previous.Value)
            ? Math.Max(0, Math.Min(previous.Value, target)) : 0;

    public static AiRecoveryAssistDecision Evaluate(AiPursuitCloseOptions options, AiRecoveryAssistInput input)
    {
        float baseSpeed = float.IsFinite(input.BaseRequestedSpeedMetersPerSecond)
            ? Math.Max(0, input.BaseRequestedSpeedMetersPerSecond) : 0;
        float nativeAcceleration = float.IsFinite(input.NativeAcceleration)
            ? Math.Max(0, input.NativeAcceleration) : 0;
        var inactive = new AiRecoveryAssistDecision(baseSpeed, nativeAcceleration, false);
        if (!input.NavigationActive || input.RecoveryActive
            || !float.IsFinite(input.PhysicalClearanceMeters) || input.PhysicalClearanceMeters < 0
            || !float.IsFinite(input.RouteDistanceMeters) || input.RouteDistanceMeters < 0
            || !float.IsFinite(input.TargetSpeedMetersPerSecond) || input.TargetSpeedMetersPerSecond < 0
            || !float.IsFinite(input.PoliceSpeedMetersPerSecond) || input.PoliceSpeedMetersPerSecond < 0
            || !float.IsFinite(input.Deceleration) || input.Deceleration <= 0
            || !float.IsFinite(input.CorneringBrakeForceFactor) || input.CorneringBrakeForceFactor <= 0)
            return inactive;

        float distance = Math.Min(input.PhysicalClearanceMeters, input.RouteDistanceMeters);
        float x = Math.Clamp((distance - options.AssistStartMeters)
                            / (options.AssistFullMeters - options.AssistStartMeters), 0, 1);
        float weight = x * x * (3 - 2 * x);
        if (weight <= 0) return inactive;
        float speed = baseSpeed + weight * Math.Max(0,
            input.TargetSpeedMetersPerSecond + options.MaxAdvantageMetersPerSecond - baseSpeed);
        // Double intermediate avoids overflow from finite float measurements.
        double availableDeceleration = input.Deceleration * (double)Math.Min(1, input.CorneringBrakeForceFactor);
        double brakingEnvelope = Math.Sqrt((double)input.TargetSpeedMetersPerSecond * input.TargetSpeedMetersPerSecond
            + 2d * availableDeceleration * Math.Max(0, input.PhysicalClearanceMeters - options.AssistStartMeters));
        speed = Math.Min(options.MaxSpeedMetersPerSecond, Math.Min(speed, Math.Max(baseSpeed, (float)brakingEnvelope)));
        float acceleration = Math.Min(options.MaxAccelerationMetersPerSecondSquared,
            nativeAcceleration + weight * Math.Max(0, options.MaxAccelerationMetersPerSecondSquared - nativeAcceleration));
        return float.IsFinite(speed) && float.IsFinite(acceleration)
            ? new(speed, acceleration, true) : inactive;
    }

    public static float StepAcceleration(float current, float desired, float jerk, float dtSeconds)
    {
        if (!float.IsFinite(current)) current = 0;
        if (!float.IsFinite(desired)) return 0;
        if (desired <= current) return desired;
        if (!float.IsFinite(dtSeconds) || dtSeconds <= 0 || !float.IsFinite(jerk) || jerk <= 0)
            return current;
        return (float)Math.Min(desired, (double)current + (double)jerk * dtSeconds);
    }
}
