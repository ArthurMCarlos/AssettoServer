using System;
using System.Collections.Generic;
using System.Numerics;

namespace AssettoServer.Server.Ai;

public enum AiLaneChangeSafetyStatus
{
    Safe,
    BlockedFront,
    BlockedSide,
    BlockedRear,
    BlockedRearClosing
}

public readonly record struct AiLaneChangeObstacle(
    Vector3 Position,
    Vector3 Velocity,
    float LengthMeters);

public sealed record AiLaneChangeSafetyRequest(
    Vector3 PolicePosition,
    Vector3 DestinationForward,
    float PoliceSpeed,
    float PoliceLengthMeters,
    float LaneWidthMeters,
    IReadOnlyList<AiLaneChangeObstacle> Obstacles);

public readonly record struct AiLaneChangeSafetyResult(
    AiLaneChangeSafetyStatus Status);

public static class AiLaneChangeSafety
{
    private const float BaseGapMeters = 8;
    private const float RearClosingTimeSeconds = 2;

    public static AiLaneChangeSafetyResult Evaluate(AiLaneChangeSafetyRequest request)
    {
        ArgumentNullException.ThrowIfNull(request);
        var forward = Vector3.Normalize(request.DestinationForward);
        var right = Vector3.Normalize(Vector3.Cross(forward, Vector3.UnitY));

        foreach (var obstacle in request.Obstacles)
        {
            var relative = obstacle.Position - request.PolicePosition;
            var longitudinal = Vector3.Dot(relative, forward);
            var lateral = MathF.Abs(Vector3.Dot(relative, right));
            if (lateral > request.LaneWidthMeters / 2)
                continue;

            var combinedHalfLength = (request.PoliceLengthMeters + obstacle.LengthMeters) / 2;
            if (MathF.Abs(longitudinal) <= combinedHalfLength)
                return new AiLaneChangeSafetyResult(AiLaneChangeSafetyStatus.BlockedSide);

            if (longitudinal > 0 && longitudinal < combinedHalfLength + BaseGapMeters)
                return new AiLaneChangeSafetyResult(AiLaneChangeSafetyStatus.BlockedFront);

            if (longitudinal < 0)
            {
                var closingSpeed = Vector3.Dot(obstacle.Velocity, forward) - request.PoliceSpeed;
                var rearGap = combinedHalfLength + BaseGapMeters;
                if (-longitudinal < rearGap)
                    return new AiLaneChangeSafetyResult(AiLaneChangeSafetyStatus.BlockedRear);
                if (closingSpeed > 0
                    && -longitudinal < rearGap + closingSpeed * RearClosingTimeSeconds)
                {
                    return new AiLaneChangeSafetyResult(
                        AiLaneChangeSafetyStatus.BlockedRearClosing);
                }
            }
        }

        return new AiLaneChangeSafetyResult(AiLaneChangeSafetyStatus.Safe);
    }
}
