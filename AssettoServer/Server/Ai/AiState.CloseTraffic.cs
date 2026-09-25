using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
using AssettoServer.Server.Ai.Routing;
using AssettoServer.Server.Ai.Splines;

namespace AssettoServer.Server.Ai;

public partial class AiState
{
    private static long _trafficIdentitySequence;
    private sealed record TrafficIdentity(long Value);
    private static readonly ConditionalWeakTable<object, TrafficIdentity> PlayerTrafficIdentities = new();
    private static long NextTrafficIdentity() => Interlocked.Increment(ref _trafficIdentitySequence);
    internal long SpawnGeneration { get; private set; }
    private sealed record TrafficObstacle(long Identity, Func<bool> Alive, Func<Vector3> Position, float Length);
    private TrafficObstacle? _blockingObstacle;
    private TrafficObstacle? _bypassObstacle;
    private float _blockingDeficit;
    private float _blockingDistance;
    private AiTrafficTacticState _trafficState = AiPursuitTrafficTactics.Initial;
    private readonly AiTrafficTacticCommit _trafficCommit = new();
    private TrafficObstacle? _pendingBypassObstacle;

    private void ReconcileTrafficCommit() => _pursuitUpdateGate.Run(() =>
    {
        var committed = _trafficCommit.Observe(_laneChangeController?.Event);
        if (committed == null) return;
        if (committed.State.Phase == AiTrafficTacticPhase.Bypass)
            _bypassObstacle = _pendingBypassObstacle;
        _trafficState = committed.State;
        _pendingBypassObstacle = null;
    });

    private void ResetTrafficTactics()
    {
        _trafficState = AiPursuitTrafficTactics.Initial;
        _trafficCommit.Clear();
        _pendingBypassObstacle = null;
        _blockingObstacle = _bypassObstacle = null;
        _blockingDeficit = 0;
    }

    private void ObserveTrafficBlocker(bool blocked, float deficit, EntryCar? player, AiState? ai, float distance)
    {
        _blockingObstacle = null;
        _blockingDeficit = blocked ? deficit : 0;
        _blockingDistance = distance;
        if (!blocked) return;
        if (ai != null && !ReferenceEquals(ai, this))
        {
            long generation = ai.SpawnGeneration;
            _blockingObstacle = new(generation, () => ai.Initialized && ai.SpawnGeneration == generation,
                () => ai.Status.Position, ai.EntryCar.VehicleLengthPreMeters + ai.EntryCar.VehicleLengthPostMeters);
        }
        else if (player?.Client is { } client)
        {
            long identity = PlayerTrafficIdentities.GetValue(client, _ => new(NextTrafficIdentity())).Value;
            _blockingObstacle = new(identity, () => ReferenceEquals(player.Client, client),
                () => player.Status.Position, player.VehicleLengthPreMeters + player.VehicleLengthPostMeters);
        }
    }

    private AiTrafficTacticDecision EvaluateTrafficTactic(AiPursuitTrackingOptions options,
        int physicalTarget, AiPursuitLaneSelectionResult alignment, bool pitBusy)
    {
        var close = options.ClosePursuit!;
        var obstacles = CaptureTrafficObstacles();
        var candidates = new List<AiTrafficLaneOption>(2);
        bool blocked = _blockingObstacle?.Alive() == true && _blockingDeficit >= close.ObstacleDeficitMetersPerSecond;
        if (!pitBusy && blocked && _trafficState.Phase == AiTrafficTacticPhase.Follow
            && _trafficState.BlockedSinceMilliseconds is { } since
            && _sessionManager.ServerTimeMilliseconds - since >= close.ObstacleHoldMilliseconds)
            foreach (var selection in _laneSelector.SelectForTrafficBypass(CurrentSplinePointId, physicalTarget,
                         new(options.MaximumRouteDistanceMeters, options.MaximumVisitedNodes)))
                candidates.Add(ObserveCandidateLane(selection, close.BypassLookaheadMeters, obstacles));
        bool passed = _bypassObstacle == null || !_bypassObstacle.Alive();
        if (!passed)
        {
            var forward = _spline.Operations.GetForwardVector(CurrentSplinePointId);
            float behind = Vector3.Dot(Status.Position - _bypassObstacle!.Position(), forward);
            passed = behind >= close.ReturnClearanceMeters + EntryCar.VehicleLengthPostMeters + _bypassObstacle.Length / 2;
        }
        var physicalLimits = new AiRouteSearchLimits(options.MaximumRouteDistanceMeters, options.MaximumVisitedNodes);
        var returnSelection = _trafficState.Phase == AiTrafficTacticPhase.Follow ? alignment.Selection
            : _laneSelector.SelectForTrafficReturn(CurrentSplinePointId, physicalTarget, physicalLimits) ?? alignment.Selection;
        if (returnSelection != null && !ObserveCandidateLane(returnSelection, close.BypassLookaheadMeters, obstacles).SafeToEnter)
            returnSelection = null;
        bool targetLaneReached = _laneSelector.IsOnTargetPhysicalLane(CurrentSplinePointId, physicalTarget, physicalLimits);
        return AiPursuitTrafficTactics.Update(close, new(
            _sessionManager.ServerTimeMilliseconds, blocked, _blockingObstacle?.Identity,
            _blockingDeficit, _blockingDistance, passed, targetLaneReached, pitBusy, false, false,
            candidates, returnSelection, _trafficState));
    }

    private List<AiLaneChangeObstacle> CaptureTrafficObstacles()
    {
        var result = new List<AiLaneChangeObstacle>();
        foreach (var car in _entryCarManager.EntryCars)
        {
            if (car.AiControlled)
            {
                var states = new List<AiState>();
                car.GetInitializedStates(states);
                foreach (var state in states)
                    if (!ReferenceEquals(state, this))
                        result.Add(new(state.Status.Position, state.Status.Velocity,
                            car.VehicleLengthPreMeters + car.VehicleLengthPostMeters));
            }
            else if (car.Client?.HasSentFirstUpdate == true)
                result.Add(new(car.Status.Position, car.Status.Velocity,
                    car.VehicleLengthPreMeters + car.VehicleLengthPostMeters));
        }
        return result;
    }

    private AiTrafficLaneOption ObserveCandidateLane(AiPursuitLaneSelection selection, float horizon,
        IReadOnlyList<AiLaneChangeObstacle> obstacles)
    {
        float progress = _currentVecLength > 0 ? GetSegmentLength(selection.ToPointId) * _currentVecProgress / _currentVecLength : 0;
        var pose = EvaluateSplinePose(selection.ToPointId, progress);
        bool safe = pose.Tangent.LengthSquared() > .01f && AiLaneChangeSafety.Evaluate(new(
            pose.Position, pose.Tangent, CurrentSpeed, EntryCar.VehicleLengthPreMeters + EntryCar.VehicleLengthPostMeters,
            _configuration.Extra.AiParams.LaneWidthMeters, obstacles)).Status == AiLaneChangeSafetyStatus.Safe;
        // Walk actual lane geometry with a finite node budget, not the plan length as a free-space proxy.
        var evaluator = new JunctionEvaluator(_spline);
        evaluator.SetExplicitDecisions(selection.DestinationPlan.JunctionDecisions);
        float traveled = -progress;
        float free = horizon;
        int point = selection.ToPointId;
        int remaining = 2048;
        while (traveled < horizon && remaining-- > 0 && evaluator.TryNext(point, out var next))
        {
            Vector3 start = _spline.Points[point].Position;
            Vector3 delta = _spline.Points[next].Position - start;
            float length = delta.Length();
            if (length > .001f)
            {
                var forward = delta / length;
                foreach (var obstacle in obstacles)
                {
                    var relative = obstacle.Position - start;
                    float along = Vector3.Dot(relative, forward);
                    float lateral = (relative - along * forward).Length();
                    if (along >= 0 && along <= length && traveled + along >= 0
                        && lateral < _configuration.Extra.AiParams.LaneWidthMeters / 2)
                        free = Math.Min(free, Math.Max(0, traveled + along - obstacle.LengthMeters / 2 - EntryCar.VehicleLengthPreMeters));
                }
            }
            traveled += length;
            point = next;
        }
        // Unknown road beyond the observed geometry is not claimed free.
        free = Math.Min(free, Math.Max(0, traveled));
        return new(selection, free, safe);
    }
}
