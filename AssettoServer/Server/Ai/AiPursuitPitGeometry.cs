using System;
using System.Collections.Generic;
using System.Numerics;

namespace AssettoServer.Server.Ai;

public readonly record struct AiPursuitPitPose(Vector3 Position, Vector3 Forward);
public readonly record struct AiPursuitPitObstacle(
    byte SessionId, Vector3 Position, Vector3 Velocity, float LengthMeters);

public static class AiPursuitPitGeometry
{
    public static bool CanStartOrContinue(
        AiPursuitPitPhase? previousPhase,
        float currentOffsetMeters) =>
        float.IsFinite(currentOffsetMeters)
        && (previousPhase is AiPursuitPitPhase.Armed or AiPursuitPitPhase.Attempting
            || MathF.Abs(currentOffsetMeters) <= .02f);

    public static bool IsJunctionWithinDistance(
        float distanceFromAnchorMeters,
        float progressInSegmentMeters,
        float windowMeters)
    {
        if (!float.IsFinite(distanceFromAnchorMeters)
            || !float.IsFinite(progressInSegmentMeters)
            || !float.IsFinite(windowMeters) || windowMeters < 0)
            return false;
        var ahead = distanceFromAnchorMeters - progressInSegmentMeters;
        return ahead >= 0 && ahead <= windowMeters;
    }

    public static (bool Left, bool Right) EvaluateSideSafety(
        Vector3 policePosition,
        Vector3 splineForward,
        float policeSpeed,
        float policeLength,
        float laneWidth,
        float offset,
        byte targetSessionId,
        IReadOnlyList<AiPursuitPitObstacle> vehicles)
    {
        ArgumentNullException.ThrowIfNull(vehicles);
        if (!IsFinite(policePosition) || !IsFinite(splineForward)
            || !float.IsFinite(policeSpeed) || policeSpeed < 0
            || !float.IsFinite(policeLength) || policeLength <= 0
            || !float.IsFinite(laneWidth) || laneWidth <= 0
            || !float.IsFinite(offset) || offset <= 0)
            return (false, false);
        var forward = Vector3.Normalize(new Vector3(splineForward.X, 0, splineForward.Z));
        if (!IsFinite(forward))
            return (false, false);
        var left = Vector3.Cross(Vector3.UnitY, forward);
        var obstacles = new List<AiLaneChangeObstacle>();
        foreach (var vehicle in vehicles)
        {
            if (vehicle.SessionId == targetSessionId)
                continue;
            if (!IsFinite(vehicle.Position) || !IsFinite(vehicle.Velocity)
                || !float.IsFinite(vehicle.LengthMeters) || vehicle.LengthMeters <= 0)
                return (false, false);
            obstacles.Add(new AiLaneChangeObstacle(
                vehicle.Position, vehicle.Velocity, vehicle.LengthMeters));
        }
        bool IsSafe(float signedOffset) => AiLaneChangeSafety.Evaluate(
            new AiLaneChangeSafetyRequest(
                policePosition + left * signedOffset,
                splineForward, policeSpeed, policeLength,
                laneWidth + 2 * offset, obstacles)).Status == AiLaneChangeSafetyStatus.Safe;
        return (IsSafe(offset), IsSafe(-offset));
    }

    public static bool IsTargetAligned(
        Vector3 policePosition,
        Vector3 splineForward,
        Vector3 targetPosition,
        Vector3 targetVelocity,
        float laneWidthMeters)
    {
        if (!IsFinite(policePosition) || !IsFinite(splineForward)
            || !IsFinite(targetPosition) || !IsFinite(targetVelocity)
            || !float.IsFinite(laneWidthMeters) || laneWidthMeters <= 0)
            return false;
        var flatForward = Vector3.Normalize(new Vector3(splineForward.X, 0, splineForward.Z));
        var flatVelocity = Vector3.Normalize(new Vector3(targetVelocity.X, 0, targetVelocity.Z));
        if (!IsFinite(flatForward) || !IsFinite(flatVelocity)
            || Vector3.Dot(flatForward, flatVelocity) < .8f)
            return false;
        var relative = targetPosition - policePosition;
        var ahead = Vector3.Dot(relative, flatForward);
        var right = Vector3.Cross(flatForward, Vector3.UnitY);
        var lateral = MathF.Abs(Vector3.Dot(relative, right));
        return ahead > 0 && lateral <= laneWidthMeters * .35f;
    }

    public static AiPursuitPitPose ApplyOffset(
        Vector3 centerPosition,
        Vector3 splineForward,
        float previousOffset,
        float currentOffset,
        float deltaSeconds,
        float speedMetersPerSecond)
    {
        if (!IsFinite(centerPosition) || !IsFinite(splineForward)
            || !float.IsFinite(previousOffset) || !float.IsFinite(currentOffset)
            || !float.IsFinite(deltaSeconds) || deltaSeconds < 0
            || !float.IsFinite(speedMetersPerSecond) || speedMetersPerSecond < 0)
            throw new ArgumentOutOfRangeException(nameof(currentOffset));
        var flatForward = Vector3.Normalize(new Vector3(splineForward.X, 0, splineForward.Z));
        if (!IsFinite(flatForward))
            return new AiPursuitPitPose(centerPosition, splineForward);
        var left = Vector3.Cross(Vector3.UnitY, flatForward);
        var position = centerPosition + left * currentOffset;
        var lateralVelocity = deltaSeconds > 0
            ? (currentOffset - previousOffset) / deltaSeconds
            : 0;
        var forward = speedMetersPerSecond > .1f
            ? Vector3.Normalize(splineForward + left * (lateralVelocity / speedMetersPerSecond))
            : splineForward;
        return new AiPursuitPitPose(position, forward);
    }

    private static bool IsFinite(Vector3 vector) =>
        float.IsFinite(vector.X) && float.IsFinite(vector.Y) && float.IsFinite(vector.Z);
}
