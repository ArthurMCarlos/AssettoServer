using System;

namespace AssettoServer.Server.Ai;

public readonly record struct AiSplinePose(
    System.Numerics.Vector3 Position,
    System.Numerics.Vector3 Tangent);

public sealed class AiSplineCursor
{
    private readonly Func<int, int?> _getNext;
    private readonly Func<int, float> _getSegmentLength;
    private readonly Func<int, float, AiSplinePose> _evaluate;

    public int PointId { get; private set; }
    public float SegmentProgressMeters { get; private set; }

    public AiSplineCursor(
        int pointId,
        float segmentProgressMeters,
        Func<int, int?> getNext,
        Func<int, float> getSegmentLength,
        Func<int, float, AiSplinePose> evaluate)
    {
        PointId = pointId;
        SegmentProgressMeters = segmentProgressMeters;
        _getNext = getNext;
        _getSegmentLength = getSegmentLength;
        _evaluate = evaluate;
    }

    public bool TryAdvance(float distanceMeters)
    {
        if (!float.IsFinite(distanceMeters) || distanceMeters < 0)
            throw new ArgumentOutOfRangeException(nameof(distanceMeters));

        var progress = SegmentProgressMeters + distanceMeters;
        while (progress > _getSegmentLength(PointId))
        {
            progress -= _getSegmentLength(PointId);
            var next = _getNext(PointId);
            if (!next.HasValue)
                return false;
            PointId = next.Value;
        }

        SegmentProgressMeters = progress;
        return true;
    }

    public bool CanAdvance(float distanceMeters)
    {
        if (!float.IsFinite(distanceMeters) || distanceMeters < 0)
            throw new ArgumentOutOfRangeException(nameof(distanceMeters));

        var pointId = PointId;
        var progress = SegmentProgressMeters + distanceMeters;
        while (progress > _getSegmentLength(pointId))
        {
            progress -= _getSegmentLength(pointId);
            var next = _getNext(pointId);
            if (!next.HasValue)
                return false;
            pointId = next.Value;
        }

        return true;
    }

    public float GetAvailableDistance(float maximumDistanceMeters)
    {
        if (!float.IsFinite(maximumDistanceMeters) || maximumDistanceMeters < 0)
            throw new ArgumentOutOfRangeException(nameof(maximumDistanceMeters));

        var pointId = PointId;
        var progress = SegmentProgressMeters;
        var available = 0f;
        while (available < maximumDistanceMeters)
        {
            var segmentRemaining = Math.Max(0, _getSegmentLength(pointId) - progress);
            var requestedRemaining = maximumDistanceMeters - available;
            if (segmentRemaining >= requestedRemaining)
                return maximumDistanceMeters;

            available += segmentRemaining;
            var next = _getNext(pointId);
            if (!next.HasValue)
                return available;

            pointId = next.Value;
            progress = 0;
        }

        return maximumDistanceMeters;
    }

    public AiSplinePose Evaluate() => _evaluate(PointId, SegmentProgressMeters);
}
