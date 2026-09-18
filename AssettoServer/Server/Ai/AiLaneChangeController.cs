using System;
using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Server.Ai;

public enum AiLaneChangePhase
{
    None,
    WaitingForGap,
    Changing,
    Cooldown
}

public enum AiPursuitLaneChangeEventKind
{
    Required,
    Waiting,
    Started,
    Completed,
    Cancelled,
    RouteRevised
}

public sealed record AiLaneChangeEvent(
    long Revision,
    AiPursuitLaneChangeEventKind Kind,
    int FromPointId,
    int ToPointId,
    AiLaneChangeDirection Direction,
    long RouteRevision,
    float? DistanceToDecisionMeters,
    AiLaneChangeSafetyStatus? SafetyStatus);

public readonly record struct AiLaneChangeMovement(
    AiSplinePose Pose,
    bool Completed,
    int DestinationPointId,
    float DestinationProgressMeters);

public sealed class AiLaneChangeController
{
    private readonly float _distanceMeters;
    private readonly int _cooldownMilliseconds;
    private AiPursuitLaneSelection? _selection;
    private AiSplineCursor? _source;
    private AiSplineCursor? _destination;
    private float _distanceTravelled;
    private long _eventRevision;
    private long _cooldownEnds;

    public AiLaneChangePhase Phase { get; private set; }
    public AiLaneChangeEvent? Event { get; private set; }

    public AiLaneChangeController(float distanceMeters, int cooldownMilliseconds)
    {
        if (!float.IsFinite(distanceMeters) || distanceMeters <= 0)
            throw new ArgumentOutOfRangeException(nameof(distanceMeters));
        if (cooldownMilliseconds < 0)
            throw new ArgumentOutOfRangeException(nameof(cooldownMilliseconds));
        _distanceMeters = distanceMeters;
        _cooldownMilliseconds = cooldownMilliseconds;
    }

    public bool Request(
        AiPursuitLaneSelection selection,
        AiSplineCursor source,
        AiSplineCursor destination,
        long nowMilliseconds,
        long routeRevision)
    {
        if (Phase == AiLaneChangePhase.Changing
            || Phase == AiLaneChangePhase.WaitingForGap
            || nowMilliseconds < _cooldownEnds)
        {
            return false;
        }

        _selection = selection;
        _source = source;
        _destination = destination;
        _distanceTravelled = 0;
        Phase = AiLaneChangePhase.WaitingForGap;
        Publish(AiPursuitLaneChangeEventKind.Required, routeRevision, null);
        return true;
    }

    public void UpdateWaiting(AiLaneChangeSafetyStatus status, long nowMilliseconds)
    {
        if (Phase != AiLaneChangePhase.WaitingForGap)
            return;
        if (status == AiLaneChangeSafetyStatus.Safe)
        {
            Phase = AiLaneChangePhase.Changing;
            Publish(AiPursuitLaneChangeEventKind.Started, Event!.RouteRevision, status);
        }
        else if (Event?.Kind != AiPursuitLaneChangeEventKind.Waiting
                 || Event.SafetyStatus != status)
        {
            Publish(AiPursuitLaneChangeEventKind.Waiting, Event!.RouteRevision, status);
        }
    }

    public bool TryMove(
        float distanceMeters,
        long nowMilliseconds,
        out AiLaneChangeMovement movement)
    {
        movement = default;
        if (Phase != AiLaneChangePhase.Changing
            || _source == null
            || _destination == null)
        {
            return false;
        }

        if (!_source.TryAdvance(distanceMeters)
            || !_destination.TryAdvance(distanceMeters))
        {
            Reset();
            return false;
        }

        _distanceTravelled = Math.Min(_distanceMeters, _distanceTravelled + distanceMeters);
        var progress = _distanceTravelled / _distanceMeters;
        var pose = AiLaneChangeTrajectory.Blend(
            _source.Evaluate(),
            _destination.Evaluate(),
            progress,
            _distanceMeters);
        var completed = progress >= 1;
        movement = new AiLaneChangeMovement(
            pose,
            completed,
            _destination.PointId,
            _destination.SegmentProgressMeters);
        if (completed)
        {
            Phase = AiLaneChangePhase.Cooldown;
            _cooldownEnds = nowMilliseconds + _cooldownMilliseconds;
            Publish(AiPursuitLaneChangeEventKind.Completed, Event!.RouteRevision, null);
        }
        return true;
    }

    public bool CancelWaiting(long routeRevision)
    {
        if (Phase != AiLaneChangePhase.WaitingForGap)
            return false;
        Publish(AiPursuitLaneChangeEventKind.Cancelled, routeRevision, null);
        Phase = AiLaneChangePhase.None;
        _selection = null;
        _source = null;
        _destination = null;
        return true;
    }

    public void Reset()
    {
        Phase = AiLaneChangePhase.None;
        Event = null;
        _selection = null;
        _source = null;
        _destination = null;
        _distanceTravelled = 0;
        _cooldownEnds = 0;
    }

    private void Publish(
        AiPursuitLaneChangeEventKind kind,
        long routeRevision,
        AiLaneChangeSafetyStatus? safetyStatus)
    {
        var selection = _selection
            ?? throw new InvalidOperationException("Lane change selection is required");
        Event = new AiLaneChangeEvent(
            ++_eventRevision,
            kind,
            selection.FromPointId,
            selection.ToPointId,
            selection.Direction,
            routeRevision,
            selection.DistanceToDecisionMeters,
            safetyStatus);
    }
}
