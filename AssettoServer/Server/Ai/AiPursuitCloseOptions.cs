using System;

namespace AssettoServer.Server.Ai;

public sealed record AiPursuitCloseOptions(
    float AssistStartMeters, float AssistFullMeters,
    float MaxAdvantageMetersPerSecond, float MaxAccelerationMetersPerSecondSquared,
    float MaxJerkMetersPerSecondCubed, float MaxSpeedMetersPerSecond,
    int ObstacleHoldMilliseconds, float ObstacleDeficitMetersPerSecond,
    float BypassLookaheadMeters, float ReturnClearanceMeters,
    float EscapeDistanceMeters, int EscapeHoldMilliseconds, int RearmDelayMilliseconds)
{
    internal void Validate(AiPursuitTrackingOptions tracking)
    {
        static bool Positive(float value) => float.IsFinite(value) && value > 0;
        if (tracking.Driving is not { Enabled: true } driving
            || tracking.LaneChange is not { Enabled: true } lane
            || !Positive(AssistStartMeters) || !Positive(AssistFullMeters)
            || !Positive(MaxAdvantageMetersPerSecond)
            || !Positive(MaxAccelerationMetersPerSecondSquared)
            || !Positive(MaxJerkMetersPerSecondCubed) || !Positive(MaxSpeedMetersPerSecond)
            || !Positive(ObstacleDeficitMetersPerSecond) || !Positive(BypassLookaheadMeters)
            || !Positive(ReturnClearanceMeters) || !Positive(EscapeDistanceMeters)
            || AssistStartMeters <= driving.CloseDistanceMeters
            || AssistFullMeters <= AssistStartMeters || EscapeDistanceMeters <= AssistFullMeters
            || EscapeDistanceMeters >= tracking.MaximumSpatialDistanceMeters
            || BypassLookaheadMeters < lane.DistanceMeters
            || MaxSpeedMetersPerSecond < driving.MaximumSpeedMetersPerSecond
            || ObstacleHoldMilliseconds <= 0 || EscapeHoldMilliseconds <= 0 || RearmDelayMilliseconds <= 0)
            throw new ArgumentOutOfRangeException(nameof(tracking.ClosePursuit));
    }
}
