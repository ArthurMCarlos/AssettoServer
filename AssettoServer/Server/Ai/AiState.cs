using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Threading;
using AssettoServer.Server.Ai.Routing;
using AssettoServer.Server.Ai.Splines;
using AssettoServer.Server.Configuration;
using AssettoServer.Server.Weather;
using AssettoServer.Shared.Model;
using AssettoServer.Shared.Network.Packets.Outgoing;
using AssettoServer.Utils;
using JPBotelho;
using Serilog;

namespace AssettoServer.Server.Ai;

public class AiState
{
    public CarStatus Status { get; } = new();
    public bool Initialized { get; private set; }

    public int CurrentSplinePointId
    {
        get => _currentSplinePointId;
        private set
        {
            _spline.SlowestAiStates.Enter(value, this);
            _spline.SlowestAiStates.Leave(_currentSplinePointId, this);
            _currentSplinePointId = value;
        }
    }

    private int _currentSplinePointId;
    
    public long SpawnProtectionEnds { get; set; }
    public float SafetyDistanceSquared { get; set; } = 20 * 20;
    public float Acceleration { get; set; }
    public float CurrentSpeed { get; private set; }
    public float TargetSpeed { get; private set; }
    public float InitialMaxSpeed { get; private set; }
    public float MaxSpeed { get; private set; }
    public Color Color { get; private set; }
    public byte SpawnCounter { get; private set; }
    public float ClosestAiObstacleDistance { get; private set; }
    public EntryCar EntryCar { get; }

    private const float WalkingSpeed = 10 / 3.6f;

    private Vector3 _startTangent;
    private Vector3 _endTangent;

    private float _currentVecLength;
    private float _currentVecProgress;
    private long _lastTick;
    private bool _stoppedForObstacle;
    private long _stoppedForObstacleSince;
    private long _ignoreObstaclesUntil;
    private long _stoppedForCollisionUntil;
    private long _obstacleHonkStart;
    private long _obstacleHonkEnd;
    private CarStatusFlags _indicator = 0;
    private int _nextJunctionId;
    private bool _junctionPassed;
    private float _endIndicatorDistance;
    private float _minObstacleDistance;

    private readonly ACServerConfiguration _configuration;
    private readonly SessionManager _sessionManager;
    private readonly EntryCarManager _entryCarManager;
    private readonly WeatherManager _weatherManager;
    private readonly AiSpline _spline;
    private readonly JunctionEvaluator _junctionEvaluator;
    private readonly AiPursuitNavigator _pursuitNavigator;
    private readonly AiPursuitDrivingController _pursuitDrivingController = new();
    private readonly AiPursuitLaneSelector _laneSelector;
    private AiLaneChangeController? _laneChangeController;
    private AiPursuitLaneChangePipeline? _laneChangePipeline;
    private readonly AiPursuitLaneDiagnosticTracker _laneDiagnosticTracker = new();
    private readonly AiPursuitUpdateGate _pursuitUpdateGate = new();
    private AiPursuitSnapshot? _pursuit;

    private static readonly List<Color> CarColors =
    [
        Color.FromArgb(13, 17, 22),
        Color.FromArgb(19, 24, 31),
        Color.FromArgb(28, 29, 33),
        Color.FromArgb(12, 13, 24),
        Color.FromArgb(11, 20, 33),
        Color.FromArgb(151, 154, 151),
        Color.FromArgb(153, 157, 160),
        Color.FromArgb(194, 196, 198),
        Color.FromArgb(234, 234, 234),
        Color.FromArgb(255, 255, 255),
        Color.FromArgb(182, 17, 27),
        Color.FromArgb(218, 25, 24),
        Color.FromArgb(73, 17, 29),
        Color.FromArgb(35, 49, 85),
        Color.FromArgb(28, 53, 81),
        Color.FromArgb(37, 58, 167),
        Color.FromArgb(21, 92, 45),
        Color.FromArgb(18, 46, 43)
    ];

    public AiState(EntryCar entryCar, SessionManager sessionManager, WeatherManager weatherManager, ACServerConfiguration configuration, EntryCarManager entryCarManager, AiSpline spline)
    {
        EntryCar = entryCar;
        _sessionManager = sessionManager;
        _weatherManager = weatherManager;
        _configuration = configuration;
        _entryCarManager = entryCarManager;
        _spline = spline;
        _junctionEvaluator = new JunctionEvaluator(spline);
        var routePlanner = new AiRoutePlanner(spline);
        _pursuitNavigator = new AiPursuitNavigator(
            routePlanner,
            new AiPursuitTargetLocator(spline));
        _laneSelector = new AiPursuitLaneSelector(spline, routePlanner);

        _lastTick = _sessionManager.ServerTimeMilliseconds;
    }

    public void Despawn()
    {
        ReleasePursuit();
        Initialized = false;
        _spline.SlowestAiStates.Leave(CurrentSplinePointId, this);
    }

    public bool ShouldRetainPursuit
    {
        get
        {
            var pursuit = Volatile.Read(ref _pursuit);
            return pursuit != null
                   && AiPursuitControl.ShouldRetain(
                       Vector3.DistanceSquared(Status.Position, pursuit.TargetPosition),
                       pursuit.Options.MaximumSpatialDistanceMeters);
        }
    }

    public AiPursuitTrackingResult TrackPursuit(
        EntryCar target,
        AiPursuitTrackingOptions options)
    {
        return _pursuitUpdateGate.Run(() => TrackPursuitLocked(target, options));
    }

    private AiPursuitTrackingResult TrackPursuitLocked(
        EntryCar target,
        AiPursuitTrackingOptions options)
    {
        ArgumentNullException.ThrowIfNull(target);
        AiPursuitControl.ValidateTrackingOptions(options);

        if (!Initialized)
        {
            return new AiPursuitTrackingResult(
                AiPursuitTrackingStatus.WaitingForSpawn,
                null,
                target.Status.Velocity.Length());
        }

        var targetPosition = target.Status.Position;
        var targetVelocity = target.Status.Velocity;
        var previous = Volatile.Read(ref _pursuit);
        if (!AiPursuitControl.HasFiniteTrackingMeasurements(
                Status.Position, targetPosition, targetVelocity, CurrentSpeed))
        {
            if (previous?.TargetSessionId == target.SessionId
                && options.Driving is { Enabled: true } fallbackDriving)
            {
                var fallback = _pursuitDrivingController.Update(
                    new AiPursuitDrivingRequest(
                        fallbackDriving,
                        previous.NavigationState.Plan.DistanceMeters,
                        float.NaN,
                        targetVelocity.Length(),
                        CurrentSpeed,
                        _laneChangeController?.Phase ?? AiLaneChangePhase.None,
                        previous.NavigationState.Revision,
                        _sessionManager.ServerTimeMilliseconds,
                        previous.DrivingState));
                Interlocked.Exchange(ref _pursuit, previous with
                {
                    DesiredSpeedMetersPerSecond = fallback.RequestedSpeedMetersPerSecond,
                    DrivingState = fallback.State,
                    DrivingDiagnostics = fallback.Diagnostics
                });
                return new AiPursuitTrackingResult(
                    AiPursuitTrackingStatus.RouteTemporarilyUnavailable,
                    null,
                    0,
                    DrivingDiagnostics: AiPursuitControl.SelectDrivingDiagnosticsForDelivery(
                        previous.DrivingDiagnostics,
                        fallback.Diagnostics));
            }

            return new AiPursuitTrackingResult(
                AiPursuitTrackingStatus.RouteTemporarilyUnavailable,
                null,
                0);
        }
        var targetSpeed = targetVelocity.Length();
        var spatialDistanceSquared = Vector3.DistanceSquared(Status.Position, targetPosition);
        if (!AiPursuitControl.ShouldRetain(
                spatialDistanceSquared,
                options.MaximumSpatialDistanceMeters))
        {
            ReleasePursuit();
            return new AiPursuitTrackingResult(
                AiPursuitTrackingStatus.MaxDistanceExceeded,
                null,
                targetSpeed);
        }

        var previousNavigation = previous?.TargetSessionId == target.SessionId
            ? previous.NavigationState
            : null;
        var navigation = _pursuitNavigator.Update(
            CurrentSplinePointId,
            targetPosition,
            targetVelocity,
            _configuration.Extra.AiParams.MaxPlayerDistanceToAiSplineSquared,
            _sessionManager.ServerTimeMilliseconds,
            previousNavigation,
            new AiPursuitNavigationOptions(
                new AiRouteSearchLimits(
                    options.MaximumRouteDistanceMeters,
                    options.MaximumVisitedNodes),
                options.RouteGraceMilliseconds));
        AiPursuitLaneChangeDiagnostics? laneEvaluationDiagnostics = null;
        if (options.LaneChange is { Enabled: true } laneChange
            && navigation.PreferredPhysicalTargetPointId.HasValue)
        {
            var preferredPhysicalTargetPointId =
                navigation.PreferredPhysicalTargetPointId.Value;
            _laneChangeController ??= new AiLaneChangeController(
                laneChange.DistanceMeters,
                laneChange.CooldownMilliseconds);
            _laneChangePipeline ??= new AiPursuitLaneChangePipeline(
                _laneSelector,
                _laneChangeController,
                CreateSplineCursor,
                GetSegmentLength);
            var preparation = _laneChangePipeline.Evaluate(
                CurrentSplinePointId,
                _currentVecProgress,
                _currentVecLength,
                navigation,
                previousNavigation,
                laneChange,
                new AiRouteSearchLimits(
                    options.MaximumRouteDistanceMeters,
                    options.MaximumVisitedNodes),
                _sessionManager.ServerTimeMilliseconds);
            navigation = preparation.EffectiveNavigation;
            laneEvaluationDiagnostics = _laneDiagnosticTracker.PublishEvaluation(
                CurrentSplinePointId,
                preferredPhysicalTargetPointId,
                navigation.State?.Revision ?? previousNavigation?.Revision ?? 0,
                preparation.Evaluation);
            if (preparation.RequestPrepared)
                UpdateLaneChangeSafety();
        }
        else if (TryRetainCommittedLaneChange(navigation, out var committedNavigation))
        {
            navigation = committedNavigation;
        }
        else
        {
            var routeRevision = navigation.State?.Revision
                                ?? previousNavigation?.Revision
                                ?? 0;
            var cancelled = _laneChangeController?.CancelWaiting(routeRevision) == true;
            if (!cancelled)
            {
                laneEvaluationDiagnostics = _laneDiagnosticTracker.PublishGate(
                    CurrentSplinePointId,
                    navigation.PreferredPhysicalTargetPointId,
                    routeRevision,
                    options.LaneChange is { Enabled: true }
                        ? AiPursuitLaneChangeDiagnosticReason.NoPhysicalTarget
                        : AiPursuitLaneChangeDiagnosticReason.Disabled);
            }
        }
        var searchDiagnostics = AiPursuitControl.CreateSearchDiagnostics(
            navigation,
            CurrentSplinePointId,
            previousNavigation?.TargetPointId);
        var laneChangeDiagnostics = CreateLaneChangeDiagnostics(
            navigation.PreferredPhysicalTargetPointId) ?? laneEvaluationDiagnostics;

        if (navigation.Status == AiPursuitNavigationStatus.NoRoute)
        {
            ReleasePursuit();
            return new AiPursuitTrackingResult(
                AiPursuitTrackingStatus.NoRoute,
                null,
                targetSpeed,
                SearchDiagnostics: searchDiagnostics,
                LaneChangeDiagnostics: laneChangeDiagnostics);
        }

        var sameTarget = previous?.TargetSessionId == target.SessionId;
        var desiredSpeed = sameTarget
            ? previous?.DesiredSpeedMetersPerSecond
            : null;
        var navigationState = navigation.State
            ?? throw new InvalidOperationException("Navigation state is required while pursuit is retained");
        AiPursuitDrivingControllerState? drivingState = sameTarget
            ? previous?.DrivingState
            : null;
        AiPursuitDrivingDiagnostics? drivingDiagnostics = sameTarget
            ? previous?.DrivingDiagnostics
            : null;
        if (navigation.Status != AiPursuitNavigationStatus.RouteTemporarilyUnavailable
            && options.Driving is { Enabled: true } driving)
        {
            var physicalClearance = AiPursuitControl.CalculatePhysicalClearance(
                MathF.Sqrt(spatialDistanceSquared),
                EntryCar.VehicleLengthPreMeters,
                target.VehicleLengthPostMeters);
            var drivingDecision = _pursuitDrivingController.Update(
                new AiPursuitDrivingRequest(
                    driving,
                    navigationState.Plan.DistanceMeters,
                    physicalClearance,
                    targetSpeed,
                    CurrentSpeed,
                    _laneChangeController?.Phase ?? AiLaneChangePhase.None,
                    navigationState.Revision,
                    _sessionManager.ServerTimeMilliseconds,
                    drivingState));
            desiredSpeed = drivingDecision.RequestedSpeedMetersPerSecond;
            drivingState = drivingDecision.State;
            drivingDiagnostics = drivingDecision.Diagnostics;
        }
        else if (options.Driving is not { Enabled: true })
        {
            drivingState = null;
            drivingDiagnostics = null;
        }
        var snapshot = new AiPursuitSnapshot(
            target.SessionId,
            targetPosition,
            options,
            navigationState,
            desiredSpeed,
            drivingState,
            drivingDiagnostics);
        Interlocked.Exchange(ref _pursuit, snapshot);
        _junctionEvaluator.SetExplicitDecisions(navigationState.Plan.JunctionDecisions);
        var deliveredDrivingDiagnostics = AiPursuitControl.SelectDrivingDiagnosticsForDelivery(
            sameTarget ? previous?.DrivingDiagnostics : null,
            drivingDiagnostics);

        if (navigation.Status == AiPursuitNavigationStatus.RouteTemporarilyUnavailable)
        {
            return new AiPursuitTrackingResult(
                AiPursuitTrackingStatus.RouteTemporarilyUnavailable,
                null,
                targetSpeed,
                SearchDiagnostics: searchDiagnostics,
                LaneChangeDiagnostics: laneChangeDiagnostics,
                DrivingDiagnostics: deliveredDrivingDiagnostics);
        }

        var diagnostics = AiPursuitControl.CreateRouteDiagnostics(
            navigation,
            CurrentSplinePointId,
            junctionId => _spline.Junctions[junctionId].EndPointId);

        return new AiPursuitTrackingResult(
            AiPursuitTrackingStatus.Active,
            navigationState.Plan.DistanceMeters,
            targetSpeed,
            diagnostics,
            searchDiagnostics,
            laneChangeDiagnostics,
            deliveredDrivingDiagnostics);
    }

    private bool TryRetainCommittedLaneChange(
        AiPursuitNavigationResult navigation,
        out AiPursuitNavigationResult committedNavigation)
    {
        if (_laneChangeController?.TryGetCommittedRoute(
                out var selection,
                out var revision) != true)
        {
            committedNavigation = null!;
            return false;
        }

        var state = new AiPursuitRouteState(
            selection.DestinationPlan.Nodes[^1].PointId,
            selection.DestinationPlan,
            revision,
            null)
        {
            PreferredPhysicalTargetPointId = navigation.PreferredPhysicalTargetPointId
        };
        committedNavigation = new AiPursuitNavigationResult(
            AiPursuitNavigationStatus.Active,
            state,
            AiPursuitRouteUpdateKind.Reused,
            AiRouteSearchFailure.None,
            navigation.VisitedNodes,
            navigation.TargetLocationDiagnostics,
            navigation.MaximumExploredDistanceMeters,
            navigation.JunctionEdgesExamined)
        {
            TargetPointIds = navigation.TargetPointIds,
            PreferredPhysicalTargetPointId = navigation.PreferredPhysicalTargetPointId
        };
        return true;
    }

    private AiPursuitLaneChangeDiagnostics? CreateLaneChangeDiagnostics(
        int? preferredPhysicalTargetPointId)
    {
        var laneEvent = _laneChangeController?.ConsumeEvent();
        return laneEvent == null
            ? null
            : new AiPursuitLaneChangeDiagnostics(
                laneEvent.Revision,
                laneEvent.Kind,
                laneEvent.FromPointId,
                laneEvent.ToPointId,
                laneEvent.Direction,
                laneEvent.RouteRevision,
                laneEvent.DistanceToDecisionMeters)
            {
                Reason = GetDiagnosticReason(laneEvent),
                PolicePointId = CurrentSplinePointId,
                PreferredPhysicalTargetPointId = preferredPhysicalTargetPointId,
                JunctionId = laneEvent.JunctionId,
                Motivation = laneEvent.Motivation,
                PhysicalRelation = laneEvent.PhysicalRelation,
                SafetyStatus = laneEvent.SafetyStatus
            };
    }

    private static AiPursuitLaneChangeDiagnosticReason GetDiagnosticReason(
        AiLaneChangeEvent laneEvent) =>
        laneEvent.Kind switch
        {
            AiPursuitLaneChangeEventKind.Required =>
                AiPursuitLaneChangeDiagnosticReason.Requested,
            AiPursuitLaneChangeEventKind.Waiting => laneEvent.SafetyStatus switch
            {
                AiLaneChangeSafetyStatus.BlockedFront =>
                    AiPursuitLaneChangeDiagnosticReason.ObstacleAhead,
                AiLaneChangeSafetyStatus.BlockedSide =>
                    AiPursuitLaneChangeDiagnosticReason.ObstacleAlongside,
                AiLaneChangeSafetyStatus.BlockedRear or
                    AiLaneChangeSafetyStatus.BlockedRearClosing =>
                    AiPursuitLaneChangeDiagnosticReason.ObstacleBehind,
                _ => AiPursuitLaneChangeDiagnosticReason.RoutePreparation
            },
            AiPursuitLaneChangeEventKind.Started =>
                AiPursuitLaneChangeDiagnosticReason.Started,
            AiPursuitLaneChangeEventKind.Completed =>
                AiPursuitLaneChangeDiagnosticReason.Completed,
            AiPursuitLaneChangeEventKind.Cancelled =>
                AiPursuitLaneChangeDiagnosticReason.Cancelled,
            AiPursuitLaneChangeEventKind.RouteRevised =>
                AiPursuitLaneChangeDiagnosticReason.RouteRevisionChanged,
            _ => throw new ArgumentOutOfRangeException(
                nameof(laneEvent.Kind), laneEvent.Kind, null)
        };

    public void SetPursuitDesiredSpeed(float metersPerSecond)
    {
        AiPursuitControl.ValidateDesiredSpeed(metersPerSecond);
        _pursuitUpdateGate.Run(() => SetPursuitDesiredSpeedLocked(metersPerSecond));
    }

    private void SetPursuitDesiredSpeedLocked(float metersPerSecond)
    {
        while (true)
        {
            var current = Volatile.Read(ref _pursuit);
            if (current == null)
                return;

            var updated = current with { DesiredSpeedMetersPerSecond = metersPerSecond };
            if (ReferenceEquals(
                    Interlocked.CompareExchange(ref _pursuit, updated, current),
                    current))
            {
                return;
            }
        }
    }

    public bool IsPursuing(byte targetSessionId)
    {
        var pursuit = Volatile.Read(ref _pursuit);
        return pursuit?.TargetSessionId == targetSessionId
               && pursuit.Options.Driving is { Enabled: true }
               && pursuit.DrivingState != null;
    }

    public bool ReportPursuitCollision(byte targetSessionId)
    {
        return _pursuitUpdateGate.Run(() => ReportPursuitCollisionLocked(targetSessionId));
    }

    private bool ReportPursuitCollisionLocked(byte targetSessionId)
    {
        while (true)
        {
            var current = Volatile.Read(ref _pursuit);
            var disposition = AiPursuitControl.ResolveCollisionDisposition(
                current?.TargetSessionId,
                current?.Options.Driving is { Enabled: true },
                current?.DrivingState != null && current.DrivingDiagnostics != null,
                targetSessionId);
            if (disposition != AiCollisionDisposition.PursuitRecovery)
            {
                return false;
            }

            var collisionState = _pursuitDrivingController.ReportCollision(
                current!.DrivingState!,
                _sessionManager.ServerTimeMilliseconds);
            var collisionDiagnostics = current.DrivingDiagnostics! with
            {
                Revision = collisionState.DiagnosticRevision,
                State = collisionState.State,
                Reason = collisionState.Reason,
                CollisionReported = true
            };
            var updated = current with
            {
                DrivingState = collisionState,
                DrivingDiagnostics = collisionDiagnostics
            };
            if (ReferenceEquals(
                    Interlocked.CompareExchange(ref _pursuit, updated, current),
                    current))
            {
                return true;
            }
        }
    }

    public void ReleasePursuit()
    {
        _pursuitUpdateGate.Run(ReleasePursuitLocked);
    }

    private void ReleasePursuitLocked()
    {
        Interlocked.Exchange(ref _pursuit, null);
        _junctionEvaluator.SetExplicitDecisions(null);
        _laneChangeController?.Reset();
        _laneDiagnosticTracker.Reset();
    }

    private AiSplineCursor CreateSplineCursor(int pointId, float progress) =>
        new(
            pointId,
            progress,
            point => _junctionEvaluator.TryNext(point, out var next) ? next : null,
            GetSegmentLength,
            EvaluateSplinePose);

    private float GetSegmentLength(int pointId)
    {
        return _junctionEvaluator.TryNext(pointId, out var next)
            ? Vector3.Distance(_spline.Points[pointId].Position, _spline.Points[next].Position)
            : 0;
    }

    private AiSplinePose EvaluateSplinePose(int pointId, float progress)
    {
        if (!_junctionEvaluator.TryNext(pointId, out var nextPointId))
            return new AiSplinePose(_spline.Points[pointId].Position, Vector3.Zero);
        var points = _spline.Points;
        var length = Vector3.Distance(points[pointId].Position, points[nextPointId].Position);
        var startTangent = _junctionEvaluator.TryPrevious(pointId, out var previousPointId)
            ? (points[nextPointId].Position - points[previousPointId].Position) * 0.5f
            : (points[nextPointId].Position - points[pointId].Position) * 0.5f;
        var endTangent = _junctionEvaluator.TryNext(pointId, out var nextNextPointId, 2)
            ? (points[nextNextPointId].Position - points[pointId].Position) * 0.5f
            : (points[nextPointId].Position - points[pointId].Position) * 0.5f;
        var pose = CatmullRom.Evaluate(
            points[pointId].Position,
            points[nextPointId].Position,
            startTangent,
            endTangent,
            length > 0 ? progress / length : 0);
        return new AiSplinePose(pose.Position, pose.Tangent);
    }

    private AiLaneChangeSafetyStatus UpdateLaneChangeSafety()
    {
        if (_laneChangeController?.Event == null
            || _laneChangeController.Phase is AiLaneChangePhase.None
                or AiLaneChangePhase.Cooldown
            || !_laneChangeController.TryGetDestinationPose(out var destinationPose))
        {
            return AiLaneChangeSafetyStatus.Safe;
        }

        var obstacles = new List<AiLaneChangeObstacle>();
        foreach (var car in _entryCarManager.EntryCars)
        {
            if (car.AiControlled)
            {
                var states = new List<AiState>();
                car.GetInitializedStates(states);
                foreach (var state in states)
                {
                    if (!ReferenceEquals(state, this))
                    {
                        obstacles.Add(new AiLaneChangeObstacle(
                            state.Status.Position,
                            state.Status.Velocity,
                            state.EntryCar.VehicleLengthPreMeters
                            + state.EntryCar.VehicleLengthPostMeters));
                    }
                }
            }
            else if (car.Client?.HasSentFirstUpdate == true)
            {
                obstacles.Add(new AiLaneChangeObstacle(
                    car.Status.Position,
                    car.Status.Velocity,
                    car.VehicleLengthPreMeters + car.VehicleLengthPostMeters));
            }
        }

        var result = AiLaneChangeSafety.Evaluate(new AiLaneChangeSafetyRequest(
            destinationPose.Position,
            destinationPose.Tangent,
            CurrentSpeed,
            EntryCar.VehicleLengthPreMeters + EntryCar.VehicleLengthPostMeters,
            _configuration.Extra.AiParams.LaneWidthMeters,
            obstacles));
        _laneChangeController.UpdateWaiting(
            result.Status,
            _sessionManager.ServerTimeMilliseconds);
        return result.Status;
    }

    private void SetRandomSpeed()
    {
        float variation = _configuration.Extra.AiParams.MaxSpeedMs * _configuration.Extra.AiParams.MaxSpeedVariationPercent;

        float fastLaneOffset = 0;
        if (_spline.Points[CurrentSplinePointId].LeftId >= 0)
        {
            fastLaneOffset = _configuration.Extra.AiParams.RightLaneOffsetMs;
        }
        InitialMaxSpeed = _configuration.Extra.AiParams.MaxSpeedMs + fastLaneOffset - (variation / 2) + (float)Random.Shared.NextDouble() * variation;
        CurrentSpeed = InitialMaxSpeed;
        TargetSpeed = InitialMaxSpeed;
        MaxSpeed = InitialMaxSpeed;
    }

    private void SetRandomColor()
    {
        Color = CarColors[Random.Shared.Next(CarColors.Count)];
    }

    public void Teleport(int pointId)
    {
        _junctionEvaluator.Clear();
        CurrentSplinePointId = pointId;
        if (!_junctionEvaluator.TryNext(CurrentSplinePointId, out var nextPointId))
            throw new InvalidOperationException($"Cannot get next spline point for {CurrentSplinePointId}");
        _currentVecLength = (_spline.Points[nextPointId].Position - _spline.Points[CurrentSplinePointId].Position).Length();
        _currentVecProgress = 0;
            
        CalculateTangents();
        
        SetRandomSpeed();
        SetRandomColor();

        var minDist = _configuration.Extra.AiParams.MinAiSafetyDistanceSquared;
        var maxDist = _configuration.Extra.AiParams.MaxAiSafetyDistanceSquared;
        if (_configuration.Extra.AiParams.LaneCountSpecificOverrides.TryGetValue(_spline.GetLanes(CurrentSplinePointId).Length, out var overrides))
        {
            minDist = overrides.MinAiSafetyDistanceSquared;
            maxDist = overrides.MaxAiSafetyDistanceSquared;
        }
        
        if (EntryCar.MinAiSafetyDistanceMetersSquared.HasValue)
            minDist = EntryCar.MinAiSafetyDistanceMetersSquared.Value;
        if (EntryCar.MaxAiSafetyDistanceMetersSquared.HasValue)
            maxDist = EntryCar.MaxAiSafetyDistanceMetersSquared.Value;

        SpawnProtectionEnds = _sessionManager.ServerTimeMilliseconds + Random.Shared.Next(EntryCar.AiMinSpawnProtectionTimeMilliseconds, EntryCar.AiMaxSpawnProtectionTimeMilliseconds);
        SafetyDistanceSquared = Random.Shared.Next((int)Math.Round(minDist * (1.0f / _configuration.Extra.AiParams.TrafficDensity)),
            (int)Math.Round(maxDist * (1.0f / _configuration.Extra.AiParams.TrafficDensity)));
        _stoppedForCollisionUntil = 0;
        _ignoreObstaclesUntil = 0;
        _obstacleHonkEnd = 0;
        _obstacleHonkStart = 0;
        _indicator = 0;
        _nextJunctionId = -1;
        _junctionPassed = false;
        _endIndicatorDistance = 0;
        _lastTick = _sessionManager.ServerTimeMilliseconds;
        _minObstacleDistance = Random.Shared.Next(8, 13);
        SpawnCounter++;
        Initialized = true;
        Update();
    }

    private void CalculateTangents()
    {
        if (!_junctionEvaluator.TryNext(CurrentSplinePointId, out var nextPointId))
            throw new InvalidOperationException("Cannot get next spline point");

        var points = _spline.Points;
        
        if (_junctionEvaluator.TryPrevious(CurrentSplinePointId, out var previousPointId))
        {
            _startTangent = (points[nextPointId].Position - points[previousPointId].Position) * 0.5f;
        }
        else
        {
            _startTangent = (points[nextPointId].Position - points[CurrentSplinePointId].Position) * 0.5f;
        }

        if (_junctionEvaluator.TryNext(CurrentSplinePointId, out var nextNextPointId, 2))
        {
            _endTangent = (points[nextNextPointId].Position - points[CurrentSplinePointId].Position) * 0.5f;
        }
        else
        {
            _endTangent = (points[nextPointId].Position - points[CurrentSplinePointId].Position) * 0.5f;
        }
    }

    private bool Move(float progress)
    {
        var points = _spline.Points;
        var junctions = _spline.Junctions;
        
        bool recalculateTangents = false;
        while (progress > _currentVecLength)
        {
            progress -= _currentVecLength;
                
            if (!_junctionEvaluator.TryNext(CurrentSplinePointId, out var nextPointId)
                || !_junctionEvaluator.TryNext(nextPointId, out var nextNextPointId))
            {
                return false;
            }

            CurrentSplinePointId = nextPointId;
            _currentVecLength = (points[nextNextPointId].Position - points[CurrentSplinePointId].Position).Length();
            recalculateTangents = true;

            if (_junctionPassed)
            {
                _endIndicatorDistance -= _currentVecLength;

                if (_endIndicatorDistance < 0)
                {
                    _indicator = 0;
                    _junctionPassed = false;
                    _endIndicatorDistance = 0;
                }
            }
                
            if (_nextJunctionId >= 0 && points[CurrentSplinePointId].JunctionEndId == _nextJunctionId)
            {
                _junctionPassed = true;
                _endIndicatorDistance = junctions[_nextJunctionId].IndicateDistancePost;
                _nextJunctionId = -1;
            }
        }

        if (recalculateTangents)
        {
            CalculateTangents();
        }

        _currentVecProgress = progress;

        return true;
    }

    public bool CanSpawn(int spawnPointId, AiState? previousAi, AiState? nextAi)
    {
        var ops = _spline.Operations;
        ref readonly var spawnPoint = ref ops.Points[spawnPointId];

        if (!IsAllowedLaneCount(spawnPointId))
            return false;
        if (!IsAllowedLane(in spawnPoint))
            return false;
        if (!IsKeepingSafetyDistances(in spawnPoint, previousAi, nextAi))
            return false;

        return EntryCar.CanSpawnAiState(spawnPoint.Position, this);
    }

    private bool IsKeepingSafetyDistances(in SplinePoint spawnPoint, AiState? previousAi, AiState? nextAi)
    {
        if (previousAi != null)
        {
            var distance = MathF.Max(0, Vector3.Distance(spawnPoint.Position, previousAi.Status.Position)
                           - previousAi.EntryCar.VehicleLengthPreMeters
                           - EntryCar.VehicleLengthPostMeters);

            var distanceSquared = distance * distance;
            if (distanceSquared < previousAi.SafetyDistanceSquared || distanceSquared < SafetyDistanceSquared)
                return false;
        }
        
        if (nextAi != null)
        {
            var distance = MathF.Max(0, Vector3.Distance(spawnPoint.Position, nextAi.Status.Position)
                                        - nextAi.EntryCar.VehicleLengthPostMeters
                                        - EntryCar.VehicleLengthPreMeters);

            var distanceSquared = distance * distance;
            if (distanceSquared < nextAi.SafetyDistanceSquared || distanceSquared < SafetyDistanceSquared)
                return false;
        }

        return true;
    }

    private bool IsAllowedLaneCount(int spawnPointId)
    {
        var laneCount = _spline.GetLanes(spawnPointId).Length;
        if (EntryCar.MinLaneCount.HasValue && laneCount < EntryCar.MinLaneCount.Value)
            return false;
        if (EntryCar.MaxLaneCount.HasValue && laneCount > EntryCar.MaxLaneCount.Value)
            return false;
        
        return true;
    }

    private bool IsAllowedLane(in SplinePoint spawnPoint)
    {
        var isAllowedLane = true;
        if (EntryCar.AiAllowedLanes != null)
        {
            isAllowedLane = (EntryCar.AiAllowedLanes.Contains(LaneSpawnBehavior.Middle) && spawnPoint.LeftId >= 0 && spawnPoint.RightId >= 0)
                            || (EntryCar.AiAllowedLanes.Contains(LaneSpawnBehavior.Left) && spawnPoint.LeftId < 0)
                            || (EntryCar.AiAllowedLanes.Contains(LaneSpawnBehavior.Right) && spawnPoint.RightId < 0);
        }

        return isAllowedLane;
    }

    private (AiState? ClosestAiState, float ClosestAiStateDistance, float MaxSpeed) SplineLookahead()
    {
        var points = _spline.Points;
        var junctions = _spline.Junctions;
        
        float maxBrakingDistance = PhysicsUtils.CalculateBrakingDistance(CurrentSpeed, EntryCar.AiDeceleration) * 2 + 20;
        AiState? closestAiState = null;
        float closestAiStateDistance = float.MaxValue;
        bool junctionFound = false;
        float distanceTravelled = 0;
        var pointId = CurrentSplinePointId;
        ref readonly var point = ref points[pointId]; 
        float maxSpeed = float.MaxValue;
        float currentSpeedSquared = CurrentSpeed * CurrentSpeed;
        while (distanceTravelled < maxBrakingDistance)
        {
            distanceTravelled += point.Length;
            pointId = _junctionEvaluator.Next(pointId);
            if (pointId < 0)
                break;

            point = ref points[pointId];

            if (!junctionFound && point.JunctionStartId >= 0 && distanceTravelled < junctions[point.JunctionStartId].IndicateDistancePre)
            {
                ref readonly var jct = ref junctions[point.JunctionStartId];
                
                var indicator = _junctionEvaluator.WillTakeJunction(point.JunctionStartId) ? jct.IndicateWhenTaken : jct.IndicateWhenNotTaken;
                if (indicator != 0)
                {
                    _indicator = indicator;
                    _nextJunctionId = point.JunctionStartId;
                    junctionFound = true;
                }
            }

            if (closestAiState == null)
            {
                var slowest = _spline.SlowestAiStates[pointId];

                if (slowest != null)
                {
                    closestAiState = slowest;
                    closestAiStateDistance = MathF.Max(0, Vector3.Distance(Status.Position, closestAiState.Status.Position)
                                                          - EntryCar.VehicleLengthPreMeters
                                                          - closestAiState.EntryCar.VehicleLengthPostMeters);
                }
            }

            float maxCorneringSpeedSquared = PhysicsUtils.CalculateMaxCorneringSpeedSquared(point.Radius, EntryCar.AiCorneringSpeedFactor);
            if (maxCorneringSpeedSquared < currentSpeedSquared)
            {
                float maxCorneringSpeed = MathF.Sqrt(maxCorneringSpeedSquared);
                float brakingDistance = PhysicsUtils.CalculateBrakingDistance(CurrentSpeed - maxCorneringSpeed,
                                            EntryCar.AiDeceleration * EntryCar.AiCorneringBrakeForceFactor)
                                        * EntryCar.AiCorneringBrakeDistanceFactor;

                if (brakingDistance > distanceTravelled)
                {
                    maxSpeed = Math.Min(maxCorneringSpeed, maxSpeed);
                }
            }
        }

        return (closestAiState, closestAiStateDistance, maxSpeed);
    }

    private bool ShouldIgnorePlayerObstacles()
    {
        if (_configuration.Extra.AiParams.IgnorePlayerObstacleSpheres != null)
        {
            foreach (var sphere in _configuration.Extra.AiParams.IgnorePlayerObstacleSpheres)
            {
                if (Vector3.DistanceSquared(Status.Position, sphere.Center) < sphere.RadiusMeters * sphere.RadiusMeters)
                {
                    return true;
                }
            }
        }

        return false;
    }

    private (EntryCar? entryCar, float distance) FindClosestPlayerObstacle(
        byte? exemptTargetSessionId = null)
    {
        if (!ShouldIgnorePlayerObstacles())
        {
            EntryCar? closestCar = null;
            float minDistance = float.MaxValue;
            for (var i = 0; i < _entryCarManager.EntryCars.Length; i++)
            {
                var playerCar = _entryCarManager.EntryCars[i];
                if (playerCar.Client?.HasSentFirstUpdate == true)
                {
                    float distance = Vector3.DistanceSquared(playerCar.Status.Position, Status.Position);

                    if (AiPursuitControl.ShouldReplaceClosestPlayerObstacle(
                            exemptTargetSessionId,
                            playerCar.SessionId,
                            distance,
                            GetAngleToCar(playerCar.Status) is > 166 and < 194,
                            minDistance))
                    {
                        minDistance = distance;
                        closestCar = playerCar;
                    }
                }
            }

            if (closestCar != null)
            {
                return (closestCar, MathF.Sqrt(minDistance));
            }
        }

        return (null, float.MaxValue);
    }

    private bool IsObstacle(EntryCar playerCar)
    {
        float aiRectWidth = 4; // Lane width
        float halfAiRectWidth = aiRectWidth / 2;
        float aiRectLength = 10; // length of rectangle infront of ai traffic
        float aiRectOffset = 1; // offset of the rectangle from ai position

        float obstacleRectWidth = 1; // width of obstacle car 
        float obstacleRectLength = 1; // length of obstacle car
        float halfObstacleRectWidth = obstacleRectWidth / 2;
        float halfObstanceRectLength = obstacleRectLength / 2;

        Vector3 forward = Vector3.Transform(-Vector3.UnitX, Matrix4x4.CreateRotationY(Status.Rotation.X));
        Matrix4x4 aiViewMatrix = Matrix4x4.CreateLookAt(Status.Position, Status.Position + forward, Vector3.UnitY);

        Matrix4x4 targetWorldViewMatrix = Matrix4x4.CreateRotationY(playerCar.Status.Rotation.X) * Matrix4x4.CreateTranslation(playerCar.Status.Position) * aiViewMatrix;

        Vector3 targetFrontLeft = Vector3.Transform(new Vector3(-halfObstanceRectLength, 0, halfObstacleRectWidth), targetWorldViewMatrix);
        Vector3 targetFrontRight = Vector3.Transform(new Vector3(-halfObstanceRectLength, 0, -halfObstacleRectWidth), targetWorldViewMatrix);
        Vector3 targetRearLeft = Vector3.Transform(new Vector3(halfObstanceRectLength, 0, halfObstacleRectWidth), targetWorldViewMatrix);
        Vector3 targetRearRight = Vector3.Transform(new Vector3(halfObstanceRectLength, 0, -halfObstacleRectWidth), targetWorldViewMatrix);

        static bool IsPointInside(Vector3 point, float width, float length, float offset)
            => MathF.Abs(point.X) >= width || (-point.Z >= offset && -point.Z <= offset + length);

        bool isObstacle = IsPointInside(targetFrontLeft, halfAiRectWidth, aiRectLength, aiRectOffset)
                          || IsPointInside(targetFrontRight, halfAiRectWidth, aiRectLength, aiRectOffset)
                          || IsPointInside(targetRearLeft, halfAiRectWidth, aiRectLength, aiRectOffset)
                          || IsPointInside(targetRearRight, halfAiRectWidth, aiRectLength, aiRectOffset);

        return isObstacle;
    }

    public void DetectObstacles()
    {
        if (!Initialized) return;
            
        if (_sessionManager.ServerTimeMilliseconds < _ignoreObstaclesUntil)
        {
            SetTargetSpeed(MaxSpeed);
            return;
        }

        if (_sessionManager.ServerTimeMilliseconds < _stoppedForCollisionUntil)
        {
            SetTargetSpeed(0);
            return;
        }
            
        var pursuit = Volatile.Read(ref _pursuit);
        float requestedSpeed = AiPursuitControl.ResolveRequestedSpeed(
            InitialMaxSpeed,
            pursuit?.DesiredSpeedMetersPerSecond);
        float targetSpeed = requestedSpeed;
        float maxSpeed = requestedSpeed;
        bool hasObstacle = false;

        var splineLookahead = SplineLookahead();
        var exemptTargetSessionId = pursuit?.Options.Driving is { Enabled: true }
            ? pursuit.TargetSessionId
            : (byte?)null;
        var playerObstacle = FindClosestPlayerObstacle(exemptTargetSessionId);
        var laneChangeSafety = UpdateLaneChangeSafety();
        var playerObstacleSpeed = playerObstacle.entryCar == null
            ? null
            : AiPursuitControl.ResolvePlayerObstacleSpeed(
                CurrentSpeed,
                playerObstacle.entryCar.Status.Velocity.Length(),
                playerObstacle.distance,
                _minObstacleDistance,
                EntryCar.AiDeceleration,
                controlledTarget: false);

        ClosestAiObstacleDistance = splineLookahead.ClosestAiState != null ? splineLookahead.ClosestAiStateDistance : -1;

        if (playerObstacleSpeed == 0
            || splineLookahead.ClosestAiStateDistance < _minObstacleDistance)
        {
            targetSpeed = 0;
            hasObstacle = true;
        }

        else if (_laneChangeController?.Phase == AiLaneChangePhase.Changing
            && laneChangeSafety != AiLaneChangeSafetyStatus.Safe)
        {
            targetSpeed = 0;
            hasObstacle = true;
        }
        else if (playerObstacle.distance < splineLookahead.ClosestAiStateDistance
                 && playerObstacleSpeed.HasValue)
        {
            targetSpeed = playerObstacleSpeed.Value;
            hasObstacle = true;
        }
        else if (splineLookahead.ClosestAiState != null)
        {
            float closestTargetSpeed = Math.Min(splineLookahead.ClosestAiState.CurrentSpeed, splineLookahead.ClosestAiState.TargetSpeed);
            if ((closestTargetSpeed < CurrentSpeed || splineLookahead.ClosestAiState.CurrentSpeed == 0)
                && splineLookahead.ClosestAiStateDistance < PhysicsUtils.CalculateBrakingDistance(CurrentSpeed - closestTargetSpeed, EntryCar.AiDeceleration) * 2 + 20)
            {
                targetSpeed = Math.Max(WalkingSpeed, closestTargetSpeed);
                hasObstacle = true;
            }
        }

        targetSpeed = AiPursuitControl.ApplySafetyLimit(targetSpeed, splineLookahead.MaxSpeed);

        if (CurrentSpeed == 0 && !_stoppedForObstacle)
        {
            _stoppedForObstacle = true;
            _stoppedForObstacleSince = _sessionManager.ServerTimeMilliseconds;
            _obstacleHonkStart = _stoppedForObstacleSince + Random.Shared.Next(3000, 7000);
            _obstacleHonkEnd = _obstacleHonkStart + Random.Shared.Next(500, 1500);
            Log.Verbose("AI {SessionId} stopped for obstacle", EntryCar.SessionId);
        }
        else if (CurrentSpeed > 0 && _stoppedForObstacle)
        {
            _stoppedForObstacle = false;
            Log.Verbose("AI {SessionId} no longer stopped for obstacle", EntryCar.SessionId);
        }
        else if (_stoppedForObstacle && _sessionManager.ServerTimeMilliseconds - _stoppedForObstacleSince > _configuration.Extra.AiParams.IgnoreObstaclesAfterMilliseconds)
        {
            _ignoreObstaclesUntil = _sessionManager.ServerTimeMilliseconds + 10_000;
            Log.Verbose("AI {SessionId} ignoring obstacles until {IgnoreObstaclesUntil}", EntryCar.SessionId, _ignoreObstaclesUntil);
        }

        float deceleration = EntryCar.AiDeceleration;
        if (!hasObstacle)
        {
            deceleration *= EntryCar.AiCorneringBrakeForceFactor;
        }
        
        MaxSpeed = maxSpeed;
        SetTargetSpeed(targetSpeed, deceleration, EntryCar.AiAcceleration);
    }

    public void StopForCollision()
    {
        if (!ShouldIgnorePlayerObstacles())
        {
            _stoppedForCollisionUntil = _sessionManager.ServerTimeMilliseconds + Random.Shared.Next(EntryCar.AiMinCollisionStopTimeMilliseconds, EntryCar.AiMaxCollisionStopTimeMilliseconds);
        }
    }

    public float GetAngleToCar(CarStatus car)
    {
        float challengedAngle = (float) (Math.Atan2(Status.Position.X - car.Position.X, Status.Position.Z - car.Position.Z) * 180 / Math.PI);
        if (challengedAngle < 0)
            challengedAngle += 360;
        float challengedRot = Status.GetRotationAngle();

        challengedAngle += challengedRot;
        challengedAngle %= 360;

        return challengedAngle;
    }

    private void SetTargetSpeed(float speed, float deceleration, float acceleration)
    {
        TargetSpeed = speed;
        if (speed < CurrentSpeed)
        {
            Acceleration = -deceleration;
        }
        else if (speed > CurrentSpeed)
        {
            Acceleration = acceleration;
        }
        else
        {
            Acceleration = 0;
        }
    }

    private void SetTargetSpeed(float speed)
    {
        SetTargetSpeed(speed, EntryCar.AiDeceleration, EntryCar.AiAcceleration);
    }

    public void Update()
    {
        if (!Initialized)
            return;

        var ops = _spline.Operations;

        long currentTime = _sessionManager.ServerTimeMilliseconds;
        long dt = currentTime - _lastTick;
        _lastTick = currentTime;

        if (Acceleration != 0)
        {
            CurrentSpeed += Acceleration * (dt / 1000.0f);
                
            if ((Acceleration < 0 && CurrentSpeed < TargetSpeed) || (Acceleration > 0 && CurrentSpeed > TargetSpeed))
            {
                CurrentSpeed = TargetSpeed;
                Acceleration = 0;
            }
        }

        float moveMeters = (dt / 1000.0f) * CurrentSpeed;
        Vector3 position;
        Vector3 tangent;
        AiLaneChangeMovement transition = default;
        var laneMovement = _laneChangeController != null
                           && _laneChangeController.TryMove(
                               moveMeters,
                               currentTime,
                               out transition);
        if (laneMovement)
        {
            position = transition.Pose.Position;
            tangent = transition.Pose.Tangent;
            if (transition.Completed)
            {
                CurrentSplinePointId = transition.DestinationPointId;
                _currentVecProgress = transition.DestinationProgressMeters;
                _currentVecLength = GetSegmentLength(CurrentSplinePointId);
                CalculateTangents();
            }
        }
        else if (!Move(_currentVecProgress + moveMeters)
                 || !_junctionEvaluator.TryNext(CurrentSplinePointId, out var nextPoint))
        {
            Log.Debug("Car {SessionId} reached spline end, despawning", EntryCar.SessionId);
            Despawn();
            return;
        }
        else
        {
            var smoothPos = CatmullRom.Evaluate(
                ops.Points[CurrentSplinePointId].Position,
                ops.Points[nextPoint].Position,
                _startTangent,
                _endTangent,
                _currentVecProgress / _currentVecLength);
            position = smoothPos.Position;
            tangent = smoothPos.Tangent;
        }
            
        Vector3 rotation = new Vector3
        {
            X = MathF.Atan2(tangent.Z, tangent.X) - MathF.PI / 2,
            Y = (MathF.Atan2(new Vector2(tangent.Z, tangent.X).Length(), tangent.Y) - MathF.PI / 2) * -1f,
            Z = ops.GetCamber(CurrentSplinePointId, _currentVecProgress / _currentVecLength)
        };

        float tyreAngularSpeed = GetTyreAngularSpeed(CurrentSpeed, EntryCar.TyreDiameterMeters);
        byte encodedTyreAngularSpeed =  (byte) (Math.Clamp(MathF.Round(MathF.Log10(tyreAngularSpeed + 1.0f) * 20.0f) * Math.Sign(tyreAngularSpeed), -100.0f, 154.0f) + 100.0f);

        Status.Timestamp = _sessionManager.ServerTimeMilliseconds;
        Status.Position = position with { Y = position.Y + EntryCar.AiSplineHeightOffsetMeters };
        Status.Rotation = rotation;
        Status.Velocity = tangent * CurrentSpeed;
        Status.SteerAngle = 127;
        Status.WheelAngle = 127;
        Status.TyreAngularSpeed[0] = encodedTyreAngularSpeed;
        Status.TyreAngularSpeed[1] = encodedTyreAngularSpeed;
        Status.TyreAngularSpeed[2] = encodedTyreAngularSpeed;
        Status.TyreAngularSpeed[3] = encodedTyreAngularSpeed;
        var pursuitSpeed = Volatile.Read(ref _pursuit)?.DesiredSpeedMetersPerSecond;
        var engineSpeedReference = pursuitSpeed.HasValue
            ? Math.Max(_configuration.Extra.AiParams.MaxSpeedMs, pursuitSpeed.Value)
            : _configuration.Extra.AiParams.MaxSpeedMs;
        Status.EngineRpm = (ushort)MathUtils.Lerp(
            EntryCar.AiIdleEngineRpm,
            EntryCar.AiMaxEngineRpm,
            Math.Clamp(CurrentSpeed / engineSpeedReference, 0, 1));
        Status.StatusFlag = CarStatusFlags.LightsOn
                            | CarStatusFlags.HighBeamsOff
                            | (_sessionManager.ServerTimeMilliseconds < _stoppedForCollisionUntil || CurrentSpeed < 20 / 3.6f ? CarStatusFlags.HazardsOn : 0)
                            | (CurrentSpeed == 0 || Acceleration < 0 ? CarStatusFlags.BrakeLightsOn : 0)
                            | (_stoppedForObstacle && _sessionManager.ServerTimeMilliseconds > _obstacleHonkStart && _sessionManager.ServerTimeMilliseconds < _obstacleHonkEnd ? CarStatusFlags.Horn : 0)
                            | GetWiperSpeed(_weatherManager.CurrentWeather.RainIntensity)
                            | _indicator;
        Status.Gear = 2;
    }
        
    private static float GetTyreAngularSpeed(float speed, float wheelDiameter)
    {
        return speed / (MathF.PI * wheelDiameter) * 6;
    }

    private static CarStatusFlags GetWiperSpeed(float rainIntensity)
    {
        return rainIntensity switch
        {
            < 0.05f => 0,
            < 0.25f => CarStatusFlags.WiperLevel1,
            < 0.5f => CarStatusFlags.WiperLevel2,
            _ => CarStatusFlags.WiperLevel3
        };
    }
}
