using System;
using System.Collections.Generic;
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
    private readonly object _sync = new();
    private readonly Queue<AiLaneChangeEvent> _events = new();
    private AiPursuitLaneSelection? _selection;
    private AiSplineCursor? _source;
    private AiSplineCursor? _destination;
    private float _distanceTravelled;
    private long _eventRevision;
    private long _cooldownEnds;

    private AiLaneChangePhase _phase;
    private AiLaneChangeEvent? _event;

    public AiLaneChangePhase Phase
    {
        get { lock (_sync) return _phase; }
    }

    public AiLaneChangeEvent? Event
    {
        get { lock (_sync) return _event; }
    }

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
        lock (_sync)
        {
            if (_phase == AiLaneChangePhase.Changing
                || _phase == AiLaneChangePhase.WaitingForGap
                || nowMilliseconds < _cooldownEnds)
            {
                return false;
            }

            _selection = selection;
            _source = source;
            _destination = destination;
            _distanceTravelled = 0;
            _phase = AiLaneChangePhase.WaitingForGap;
            Publish(AiPursuitLaneChangeEventKind.Required, routeRevision, null);
            return true;
        }
    }

    public void UpdateWaiting(AiLaneChangeSafetyStatus status, long nowMilliseconds)
    {
        lock (_sync)
        {
            if (_phase != AiLaneChangePhase.WaitingForGap)
                return;
            if (status == AiLaneChangeSafetyStatus.Safe)
            {
                _phase = AiLaneChangePhase.Changing;
                Publish(AiPursuitLaneChangeEventKind.Started, _event!.RouteRevision, status);
            }
            else if (_event?.Kind != AiPursuitLaneChangeEventKind.Waiting
                     || _event.SafetyStatus != status)
            {
                Publish(AiPursuitLaneChangeEventKind.Waiting, _event!.RouteRevision, status);
            }
        }
    }

    public void RefreshWaiting(
        AiPursuitLaneSelection selection,
        AiSplineCursor source,
        AiSplineCursor destination,
        long routeRevision)
    {
        lock (_sync)
        {
            if (_phase != AiLaneChangePhase.WaitingForGap)
                return;
            var routeChanged = _selection?.ToPointId != selection.ToPointId
                               || _event?.RouteRevision != routeRevision;
            _selection = selection;
            _source = source;
            _destination = destination;
            if (routeChanged)
                Publish(AiPursuitLaneChangeEventKind.RouteRevised, routeRevision, null);
        }
    }

    public bool TryGetDestinationPose(out AiSplinePose pose)
    {
        lock (_sync)
        {
            if (_destination == null
                || _phase is AiLaneChangePhase.None or AiLaneChangePhase.Cooldown)
            {
                pose = default;
                return false;
            }

            pose = _destination.Evaluate();
            return true;
        }
    }

    public AiLaneChangeEvent? ConsumeEvent()
    {
        lock (_sync)
            return _events.Count == 0 ? null : _events.Dequeue();
    }


    public bool TryMove(
        float distanceMeters,
        long nowMilliseconds,
        out AiLaneChangeMovement movement)
    {
        lock (_sync)
        {
            movement = default;
            if (_phase != AiLaneChangePhase.Changing
                || _source == null
                || _destination == null)
            {
                return false;
            }

            if (!_source.TryAdvance(distanceMeters)
                || !_destination.TryAdvance(distanceMeters))
            {
                ResetCore();
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
                _phase = AiLaneChangePhase.Cooldown;
                _cooldownEnds = nowMilliseconds + _cooldownMilliseconds;
                Publish(AiPursuitLaneChangeEventKind.Completed, _event!.RouteRevision, null);
            }
            return true;
        }
    }

    public bool CancelWaiting(long routeRevision)
    {
        lock (_sync)
        {
            if (_phase != AiLaneChangePhase.WaitingForGap)
                return false;
            Publish(AiPursuitLaneChangeEventKind.Cancelled, routeRevision, null);
            _phase = AiLaneChangePhase.None;
            _selection = null;
            _source = null;
            _destination = null;
            return true;
        }
    }

    public void Reset()
    {
        lock (_sync)
        {
            ResetCore();
            _events.Clear();
        }
    }

    private void ResetCore()
    {
        _phase = AiLaneChangePhase.None;
        _event = null;
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
        _event = new AiLaneChangeEvent(
            ++_eventRevision,
            kind,
            selection.FromPointId,
            selection.ToPointId,
            selection.Direction,
            routeRevision,
            selection.DistanceToDecisionMeters,
            safetyStatus);
        _events.Enqueue(_event);
    }
}
