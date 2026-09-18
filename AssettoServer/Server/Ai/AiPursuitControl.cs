using System;
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

public sealed record AiPursuitTrackingResult(
    AiPursuitTrackingStatus Status,
    float? RouteDistanceMeters,
    float TargetSpeedMetersPerSecond);

internal sealed record AiPursuitSnapshot(
    byte TargetSessionId,
    Vector3 TargetPosition,
    float MaxDistanceMeters,
    AiRoutePlan Route,
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
}
