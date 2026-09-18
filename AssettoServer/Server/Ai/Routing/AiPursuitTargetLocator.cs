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
    private readonly Func<int, Vector3, IReadOnlyList<AiPursuitTargetCandidateSource>> _findLaneEquivalents;

    public AiPursuitTargetLocator(AiSpline spline)
        : this(
            (position, count) => FindNearest(spline, position, count),
            (pointId, position) => FindLaneEquivalents(spline, pointId, position))
    {
    }

    internal AiPursuitTargetLocator(
        Func<Vector3, int, IReadOnlyList<AiPursuitTargetCandidateSource>> findNearest)
        : this(findNearest, (_, _) => [])
    {
    }

    internal AiPursuitTargetLocator(
        Func<Vector3, int, IReadOnlyList<AiPursuitTargetCandidateSource>> findNearest,
        Func<int, Vector3, IReadOnlyList<AiPursuitTargetCandidateSource>> findLaneEquivalents)
    {
        _findNearest = findNearest ?? throw new ArgumentNullException(nameof(findNearest));
        _findLaneEquivalents = findLaneEquivalents
            ?? throw new ArgumentNullException(nameof(findLaneEquivalents));
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
        var spatialCandidates = new List<AiPursuitTargetCandidate>();

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

            spatialCandidates.Add(new AiPursuitTargetCandidate(
                source.PointId,
                source.DistanceSquared,
                alignment));
        }

        var orderedSpatialCandidates = spatialCandidates
            .OrderBy(candidate => candidate.DistanceSquared)
            .ThenBy(candidate => candidate.PointId)
            .Take(maximumCandidates)
            .ToArray();
        var candidates = orderedSpatialCandidates.ToDictionary(
            candidate => candidate.PointId);

        foreach (var spatialCandidate in orderedSpatialCandidates)
        {
            foreach (var source in _findLaneEquivalents(spatialCandidate.PointId, position))
            {
                if (candidates.ContainsKey(source.PointId)
                    || !float.IsFinite(source.DistanceSquared)
                    || source.DistanceSquared < 0)
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

                candidates.Add(
                    source.PointId,
                    new AiPursuitTargetCandidate(
                        source.PointId,
                        source.DistanceSquared,
                        alignment));
            }
        }

        return candidates.Values
            .OrderBy(candidate => candidate.DistanceSquared)
            .ThenBy(candidate => candidate.PointId)
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

    private static IReadOnlyList<AiPursuitTargetCandidateSource> FindLaneEquivalents(
        AiSpline spline,
        int pointId,
        Vector3 position)
    {
        var lanes = spline.GetLanes(pointId);
        var points = spline.Points;
        var operations = spline.Operations;
        var result = new List<AiPursuitTargetCandidateSource>(lanes.Length);
        foreach (var lanePointId in lanes)
        {
            if (lanePointId == pointId)
                continue;

            result.Add(new AiPursuitTargetCandidateSource(
                lanePointId,
                Vector3.DistanceSquared(position, points[lanePointId].Position),
                operations.GetForwardVector(lanePointId)));
        }

        return result;
    }
}
