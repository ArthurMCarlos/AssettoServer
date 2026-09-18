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

public enum AiPursuitTargetRejectionReason
{
    InvalidDistance,
    OutsideMaximumDistance,
    MissingForwardDirection,
    OppositeDirection
}

public readonly record struct AiPursuitTargetRejection(
    int PointId,
    AiPursuitTargetRejectionReason Reason);

public sealed record AiPursuitTargetLocationDiagnostics(
    IReadOnlyList<int> SpatialPointIds,
    IReadOnlyList<int> LaneEquivalentPointIds,
    IReadOnlyList<AiPursuitTargetRejection> Rejections);

public sealed record AiPursuitTargetLocationResult(
    IReadOnlyList<AiPursuitTargetCandidate> Candidates,
    AiPursuitTargetLocationDiagnostics Diagnostics);

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
        int maximumCandidates) =>
        LocateCandidates(
            position,
            velocity,
            maximumDistanceSquared,
            maximumCandidates).Candidates;

    public AiPursuitTargetLocationResult LocateCandidates(
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
        var spatialSources = new List<AiPursuitTargetCandidateSource>();
        var spatialPointIds = new List<int>();
        var laneEquivalentPointIds = new List<int>();
        var rejections = new List<AiPursuitTargetRejection>();

        foreach (var source in _findNearest(position, maximumCandidates))
        {
            spatialPointIds.Add(source.PointId);
            if (!float.IsFinite(source.DistanceSquared) || source.DistanceSquared < 0)
            {
                rejections.Add(new AiPursuitTargetRejection(
                    source.PointId,
                    AiPursuitTargetRejectionReason.InvalidDistance));
                continue;
            }

            if (source.DistanceSquared > maximumDistanceSquared)
            {
                rejections.Add(new AiPursuitTargetRejection(
                    source.PointId,
                    AiPursuitTargetRejectionReason.OutsideMaximumDistance));
                continue;
            }

            spatialSources.Add(source);
        }

        var orderedSpatialSources = spatialSources
            .OrderBy(source => source.DistanceSquared)
            .ThenBy(source => source.PointId)
            .Take(maximumCandidates)
            .ToArray();
        var candidates = new Dictionary<int, AiPursuitTargetCandidate>();

        foreach (var spatialSource in orderedSpatialSources)
        {
            if (TryCreateCandidate(
                    spatialSource,
                    velocityDirection,
                    hasDirection,
                    maximumDistanceSquared,
                    enforceMaximumDistance: false,
                    out var spatialCandidate,
                    out var spatialRejection))
            {
                candidates.TryAdd(spatialCandidate.PointId, spatialCandidate);
            }
            else
            {
                rejections.Add(spatialRejection);
            }

            foreach (var source in _findLaneEquivalents(spatialSource.PointId, position))
            {
                if (candidates.ContainsKey(source.PointId))
                    continue;

                laneEquivalentPointIds.Add(source.PointId);
                if (TryCreateCandidate(
                        source,
                        velocityDirection,
                        hasDirection,
                        maximumDistanceSquared,
                        enforceMaximumDistance: false,
                        out var candidate,
                        out var rejection))
                {
                    candidates.Add(source.PointId, candidate);
                }
                else
                {
                    rejections.Add(rejection);
                }
            }
        }

        var orderedCandidates = candidates.Values
            .OrderBy(candidate => candidate.DistanceSquared)
            .ThenBy(candidate => candidate.PointId)
            .ToArray();
        return new AiPursuitTargetLocationResult(
            orderedCandidates,
            new AiPursuitTargetLocationDiagnostics(
                spatialPointIds,
                laneEquivalentPointIds.Distinct().ToArray(),
                rejections));
    }

    private static bool TryCreateCandidate(
        AiPursuitTargetCandidateSource source,
        Vector3 velocityDirection,
        bool hasDirection,
        float maximumDistanceSquared,
        bool enforceMaximumDistance,
        out AiPursuitTargetCandidate candidate,
        out AiPursuitTargetRejection rejection)
    {
        candidate = default;
        if (!float.IsFinite(source.DistanceSquared) || source.DistanceSquared < 0)
        {
            rejection = new AiPursuitTargetRejection(
                source.PointId,
                AiPursuitTargetRejectionReason.InvalidDistance);
            return false;
        }

        if (enforceMaximumDistance && source.DistanceSquared > maximumDistanceSquared)
        {
            rejection = new AiPursuitTargetRejection(
                source.PointId,
                AiPursuitTargetRejectionReason.OutsideMaximumDistance);
            return false;
        }

        var alignment = 0.0f;
        if (hasDirection)
        {
            var forwardLength = source.Forward.Length();
            if (forwardLength <= 0)
            {
                rejection = new AiPursuitTargetRejection(
                    source.PointId,
                    AiPursuitTargetRejectionReason.MissingForwardDirection);
                return false;
            }

            alignment = Vector3.Dot(velocityDirection, source.Forward / forwardLength);
            if (alignment < MinimumDirectionAlignment)
            {
                rejection = new AiPursuitTargetRejection(
                    source.PointId,
                    AiPursuitTargetRejectionReason.OppositeDirection);
                return false;
            }
        }

        candidate = new AiPursuitTargetCandidate(
            source.PointId,
            source.DistanceSquared,
            alignment);
        rejection = default;
        return true;
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
