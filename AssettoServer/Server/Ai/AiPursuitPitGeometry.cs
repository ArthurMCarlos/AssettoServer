using System;
using System.Collections.Generic;
using System.Numerics;

namespace AssettoServer.Server.Ai;

public readonly record struct AiPursuitPitPose(Vector3 Position, Vector3 Forward);
public readonly record struct AiPursuitPitObstacle(
    byte SessionId, Vector3 Position, Vector3 Velocity, float LengthMeters,
    string Kind = "Unknown", string? Model = null, byte? SpawnCounter = null);
public sealed record AiPursuitPitSideSafetyResult(
    bool Left, bool Right, string LeftReason, string RightReason,
    AiPursuitPitObstacle? LeftBlocker, AiPursuitPitObstacle? RightBlocker,
    float PoliceLengthMeters, Vector3 PolicePosition);
public readonly record struct AiPursuitPitAlignmentMeasurements(
    float LongitudinalMeters, float LateralMeters, float HeadingDot);

public static class AiPursuitPitGeometry
{
    public static AiPursuitPitAlignmentMeasurements MeasureAlignment(
        Vector3 policePosition, Vector3 splineForward,
        Vector3 targetPosition, Vector3 targetVelocity)
    {
        if (!IsFinite(policePosition) || !IsFinite(splineForward)
            || !IsFinite(targetPosition) || !IsFinite(targetVelocity))
            return new(float.NaN, float.NaN, float.NaN);
        var forward = Vector3.Normalize(new Vector3(splineForward.X, 0, splineForward.Z));
        var velocity = Vector3.Normalize(new Vector3(targetVelocity.X, 0, targetVelocity.Z));
        if (!IsFinite(forward) || !IsFinite(velocity))
            return new(float.NaN, float.NaN, float.NaN);
        var relative = targetPosition - policePosition;
        return new(Vector3.Dot(relative, forward),
            MathF.Abs(Vector3.Dot(relative, Vector3.Cross(forward, Vector3.UnitY))),
            Vector3.Dot(forward, velocity));
    }

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
        var result = EvaluateSideSafetyDetailed(policePosition, splineForward,
            policeSpeed, policeLength, laneWidth, offset, targetSessionId, vehicles);
        return (result.Left, result.Right);
    }

    public static AiPursuitPitSideSafetyResult EvaluateSideSafetyDetailed(
        Vector3 policePosition, Vector3 splineForward, float policeSpeed,
        float policeLength, float laneWidth, float offset, byte targetSessionId,
        IReadOnlyList<AiPursuitPitObstacle> vehicles)
    {
        ArgumentNullException.ThrowIfNull(vehicles);
        AiPursuitPitSideSafetyResult Invalid(string reason, AiPursuitPitObstacle? blocker = null) =>
            new(false, false, reason, reason, blocker, blocker, policeLength, policePosition);
        if (!IsFinite(policePosition) || !IsFinite(splineForward)
            || !float.IsFinite(policeSpeed) || policeSpeed < 0
            || !float.IsFinite(laneWidth) || laneWidth <= 0
            || !float.IsFinite(offset) || offset <= 0)
            return Invalid("InvalidPoliceGeometry");
        // EntryCar defaults to zero when no model dimension override is supplied.
        // Preserve the native safety evaluator's nonnegative-length contract and
        // its existing gap/closing margins; do not invent vehicle dimensions.
        if (!float.IsFinite(policeLength) || policeLength < 0)
            return Invalid("InvalidPoliceLength");
        var forward = Vector3.Normalize(new Vector3(splineForward.X, 0, splineForward.Z));
        if (!IsFinite(forward))
            return Invalid("InvalidPoliceDirection");
        var left = Vector3.Cross(Vector3.UnitY, forward);
        foreach (var vehicle in vehicles)
        {
            if (vehicle.SessionId == targetSessionId)
                continue;
            if (!IsFinite(vehicle.Position) || !IsFinite(vehicle.Velocity))
                return Invalid("InvalidObstacleGeometry", vehicle);
            if (!float.IsFinite(vehicle.LengthMeters) || vehicle.LengthMeters < 0)
                return Invalid("InvalidObstacleLength", vehicle);
        }
        (AiLaneChangeSafetyStatus Status, AiPursuitPitObstacle? Blocker) Evaluate(float signedOffset)
        {
            foreach (var vehicle in vehicles)
            {
                if (vehicle.SessionId == targetSessionId)
                    continue;
                var status = AiLaneChangeSafety.Evaluate(new AiLaneChangeSafetyRequest(
                    policePosition + left * signedOffset, splineForward, policeSpeed,
                    policeLength, laneWidth + 2 * offset,
                    [new AiLaneChangeObstacle(vehicle.Position, vehicle.Velocity, vehicle.LengthMeters)])).Status;
                if (status != AiLaneChangeSafetyStatus.Safe)
                    return (status, vehicle);
            }
            return (AiLaneChangeSafetyStatus.Safe, null);
        }
        var leftResult = Evaluate(offset);
        var rightResult = Evaluate(-offset);
        return new(leftResult.Status == AiLaneChangeSafetyStatus.Safe,
            rightResult.Status == AiLaneChangeSafetyStatus.Safe,
            leftResult.Status.ToString(), rightResult.Status.ToString(),
            leftResult.Blocker, rightResult.Blocker, policeLength, policePosition);
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
