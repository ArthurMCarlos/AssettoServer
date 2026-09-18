using System;
using System.Numerics;

namespace AssettoServer.Server.Ai;

public static class AiLaneChangeTrajectory
{
    public static AiSplinePose Blend(
        AiSplinePose source,
        AiSplinePose destination,
        float progress,
        float distanceMeters)
    {
        if (!float.IsFinite(distanceMeters) || distanceMeters <= 0)
            throw new ArgumentOutOfRangeException(nameof(distanceMeters));

        var t = Math.Clamp(progress, 0, 1);
        var weight = t * t * t * (t * (t * 6 - 15) + 10);
        var derivative = 30 * t * t * (t * (t - 2) + 1) / distanceMeters;
        var position = Vector3.Lerp(source.Position, destination.Position, weight);
        var tangent = Vector3.Lerp(source.Tangent, destination.Tangent, weight)
                      + (destination.Position - source.Position) * derivative;
        if (tangent.LengthSquared() > 0)
            tangent = Vector3.Normalize(tangent);
        return new AiSplinePose(position, tangent);
    }
}
