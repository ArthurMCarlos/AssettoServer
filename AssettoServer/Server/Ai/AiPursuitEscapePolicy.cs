using System;

namespace AssettoServer.Server.Ai;

public sealed record AiPursuitEscapeState(long? FarSinceMilliseconds);
public sealed record AiPursuitEscapeDecision(AiPursuitEscapeState State, bool Escaped);

public static class AiPursuitEscapePolicy
{
    public static AiPursuitEscapeDecision Update(float spatialDistance, bool navigationActive,
        long nowMilliseconds, AiPursuitCloseOptions options, AiPursuitEscapeState previous)
    {
        if (!navigationActive || !float.IsFinite(spatialDistance) || spatialDistance <= options.EscapeDistanceMeters)
            return new(new(null), false);
        long since = previous.FarSinceMilliseconds is { } old && old <= nowMilliseconds ? old : nowMilliseconds;
        return new(new(since), nowMilliseconds - since >= options.EscapeHoldMilliseconds);
    }
}
