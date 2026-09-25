namespace AssettoServer.Server.Ai;

public enum AiPursuitSpeedLimiter { Requested, TechnicalCap, Curve, AiObstacle, PlayerObstacle, LaneSafety, Recovery }

public sealed record AiPursuitCloseDiagnostics(
    float? PhysicalClearanceMeters, float? RouteDistanceMeters,
    float? TargetSpeedMetersPerSecond, float? PoliceSpeedMetersPerSecond,
    float? RequestedSpeedMetersPerSecond, float? EffectiveSpeedMetersPerSecond,
    float? AppliedAcceleration, AiPursuitSpeedLimiter? Limiter,
    bool AssistActive, string TacticPhase, bool EscapePending)
{
    public long Revision { get; init; }
}
