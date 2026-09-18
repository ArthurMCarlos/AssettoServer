using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using AssettoServer.Server.Ai.Splines;

namespace AssettoServer.Server.Ai.Routing;

public readonly record struct AiPursuitTargetCandidateSource(
    int PointId,
    float DistanceSquared,
    Vector3 Forward);

public readonly record struct AiPursuitTargetCandidate(
    int PointId,
    float DistanceSquared,
    float DirectionAlignment);

public sealed class AiPursuitTargetLocator
{
    private const float DirectionSpeedThreshold = 2.0f;
    private const float MinimumDirectionAlignment = 0.25f;

    private readonly Func<Vector3, int, IReadOnlyList<AiPursuitTargetCandidateSource>> _findNearest;

    public AiPursuitTargetLocator(AiSpline spline)
        : this((position, count) => FindNearest(spline, position, count))
    {
    }

    internal AiPursuitTargetLocator(
        Func<Vector3, int, IReadOnlyList<AiPursuitTargetCandidateSource>> findNearest)
    {
        _findNearest = findNearest ?? throw new ArgumentNullException(nameof(findNearest));
    }

    public IReadOnlyList<AiPursuitTargetCandidate> FindCandidates(
        Vector3 position,
        Vector3 velocity,
        float maximumDistanceSquared,
        int maximumCandidates)
    {
        if (!float.IsFinite(maximumDistanceSquared) || maximumDistanceSquared < 0)
            throw new ArgumentOutOfRangeException(nameof(maximumDistanceSquared));
        if (maximumCandidates <= 0)
            throw new ArgumentOutOfRangeException(nameof(maximumCandidates));

        var speed = velocity.Length();
        var hasDirection = speed >= DirectionSpeedThreshold;
        var velocityDirection = hasDirection ? velocity / speed : Vector3.Zero;
        var candidates = new List<AiPursuitTargetCandidate>();

        foreach (var source in _findNearest(position, maximumCandidates))
        {
            if (!float.IsFinite(source.DistanceSquared)
                || source.DistanceSquared < 0
                || source.DistanceSquared > maximumDistanceSquared)
            {
                continue;
            }

            var alignment = 0.0f;
            if (hasDirection)
            {
                var forwardLength = source.Forward.Length();
                if (forwardLength <= 0)
                    continue;

                alignment = Vector3.Dot(velocityDirection, source.Forward / forwardLength);
                if (alignment < MinimumDirectionAlignment)
                    continue;
            }

            candidates.Add(new AiPursuitTargetCandidate(
                source.PointId,
                source.DistanceSquared,
                alignment));
        }

        return candidates
            .OrderBy(candidate => candidate.DistanceSquared)
            .ThenBy(candidate => candidate.PointId)
            .Take(maximumCandidates)
            .ToArray();
    }

    private static IReadOnlyList<AiPursuitTargetCandidateSource> FindNearest(
        AiSpline spline,
        Vector3 position,
        int count)
    {
        var nearest = spline.KdTree.NearestNeighbors(position, count);
        var points = spline.Points;
        var operations = spline.Operations;
        var result = new AiPursuitTargetCandidateSource[nearest.Length];
        for (var i = 0; i < nearest.Length; i++)
        {
            var pointId = nearest[i].Item2;
            result[i] = new AiPursuitTargetCandidateSource(
                pointId,
                Vector3.DistanceSquared(position, points[pointId].Position),
                operations.GetForwardVector(pointId));
        }

        return result;
    }
}
