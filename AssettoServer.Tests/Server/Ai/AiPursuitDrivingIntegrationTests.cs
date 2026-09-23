using AssettoServer.Server.Ai;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitDrivingIntegrationTests
{
    private static readonly AiPursuitDrivingOptions DrivingOptions = new(
        Enabled: true,
        ContactEnabled: true,
        CatchUpDistanceMeters: 100,
        CloseDistanceMeters: 15,
        ContactDistanceMeters: 3,
        MaximumSpeedMetersPerSecond: 50,
        MaximumClosingSpeedMetersPerSecond: 35 / 3.6f,
        ContactClosingSpeedMetersPerSecond: 5 / 3.6f);

    [Test]
    public void PhysicalClearanceSubtractsPoliceFrontAndTargetRearLengths()
    {
        var clearance = AiPursuitControl.CalculatePhysicalClearance(
            centerDistance: 8,
            policeFrontLength: 2,
            targetRearLength: 2);

        Assert.That(clearance, Is.EqualTo(4));
    }

    [Test]
    public void PhysicalClearanceNeverBecomesNegative()
    {
        var clearance = AiPursuitControl.CalculatePhysicalClearance(3, 2, 2);

        Assert.That(clearance, Is.Zero);
    }

    [Test]
    public void InvalidLivePositionOrSpeedIsRejectedBeforeRouteLookup()
    {
        var position = new Vector3(1, 2, 3);
        Assert.Multiple(() =>
        {
            Assert.That(AiPursuitControl.HasFiniteTrackingMeasurements(
                position, position, position, 20), Is.True);
            Assert.That(AiPursuitControl.HasFiniteTrackingMeasurements(
                position, new Vector3(float.NaN, 2, 3), position, 20), Is.False);
            Assert.That(AiPursuitControl.HasFiniteTrackingMeasurements(
                position, position, new Vector3(1, float.PositiveInfinity, 3), 20), Is.False);
            Assert.That(AiPursuitControl.HasFiniteTrackingMeasurements(
                position, position, position, float.NaN), Is.False);
        });
    }

    [TestCase((byte)10, (byte)10, true, true)]
    [TestCase((byte)10, (byte)11, true, false)]
    [TestCase((byte)10, (byte)10, false, false)]
    [TestCase(null, (byte)10, true, false)]
    public void ControlledObstacleRequiresEnabledPursuitAndExactTarget(
        byte? pursuitTargetSessionId,
        byte obstacleSessionId,
        bool aggressiveDrivingEnabled,
        bool expected)
    {
        var controlled = AiPursuitControl.IsControlledTargetObstacle(
            pursuitTargetSessionId,
            obstacleSessionId,
            aggressiveDrivingEnabled);

        Assert.That(controlled, Is.EqualTo(expected));
    }

    [Test]
    public void ControlledTargetBypassesGenericPlayerHardStop()
    {
        var limit = AiPursuitControl.ResolvePlayerObstacleSpeed(
            currentSpeed: 20,
            playerSpeed: 10,
            playerDistance: 1,
            minimumObstacleDistance: 10,
            deceleration: 8.5f,
            controlledTarget: true);

        Assert.That(limit, Is.Null);
    }

    [Test]
    public void PursuedTargetDoesNotMaskAnotherPlayerInBrakingRange()
    {
        byte? selected = null;
        var closestDistanceSquared = float.MaxValue;
        foreach (var candidate in new[]
                 {
                     (SessionId: (byte)10, DistanceSquared: 4f),
                     (SessionId: (byte)11, DistanceSquared: 9f)
                 })
        {
            if (!AiPursuitControl.ShouldReplaceClosestPlayerObstacle(
                    exemptTargetSessionId: 10,
                    candidate.SessionId,
                    candidate.DistanceSquared,
                    isAhead: true,
                    closestDistanceSquared))
                continue;

            selected = candidate.SessionId;
            closestDistanceSquared = candidate.DistanceSquared;
        }

        Assert.That(selected, Is.EqualTo(11));
        Assert.That(AiPursuitControl.ResolvePlayerObstacleSpeed(
            currentSpeed: 20,
            playerSpeed: 10,
            playerDistance: MathF.Sqrt(closestDistanceSquared),
            minimumObstacleDistance: 10,
            deceleration: 8.5f,
            controlledTarget: false), Is.Zero);
    }

    [TestCase(false)]
    [TestCase(true)]
    public void OrdinaryPlayerAndDisabledPursuitKeepGenericHardStop(bool hasPursuit)
    {
        var controlledTarget = hasPursuit
            && AiPursuitControl.IsControlledTargetObstacle(10, 11, true);
        var limit = AiPursuitControl.ResolvePlayerObstacleSpeed(
            currentSpeed: 20,
            playerSpeed: 10,
            playerDistance: 1,
            minimumObstacleDistance: 10,
            deceleration: 8.5f,
            controlledTarget);

        Assert.That(limit, Is.Zero);
    }

    [Test]
    public void SlowerOrdinaryPlayerKeepsNativeBrakingEnvelope()
    {
        var limit = AiPursuitControl.ResolvePlayerObstacleSpeed(
            currentSpeed: 30,
            playerSpeed: 10,
            playerDistance: 30,
            minimumObstacleDistance: 10,
            deceleration: 8.5f,
            controlledTarget: false);

        Assert.That(limit, Is.EqualTo(10));
    }

    [Test]
    public void DistantOrdinaryPlayerDoesNotLimitSpeed()
    {
        var limit = AiPursuitControl.ResolvePlayerObstacleSpeed(
            currentSpeed: 30,
            playerSpeed: 10,
            playerDistance: 100,
            minimumObstacleDistance: 10,
            deceleration: 8.5f,
            controlledTarget: false);

        Assert.That(limit, Is.Null);
    }

    [Test]
    public void ControlledTargetDoesNotBypassTrafficOrCurveSafetyLimits()
    {
        const float pursuitRequest = 180 / 3.6f;
        var afterTraffic = AiPursuitControl.ApplySafetyLimit(pursuitRequest, 80 / 3.6f);
        var afterCurve = AiPursuitControl.ApplySafetyLimit(afterTraffic, 60 / 3.6f);

        Assert.That(afterCurve * 3.6f, Is.EqualTo(60).Within(0.01));
    }

    [Test]
    public void TrackingValidationRejectsInvalidDrivingThresholds()
    {
        var options = new AiPursuitTrackingOptions(
            MaximumSpatialDistanceMeters: 1500,
            MaximumRouteDistanceMeters: 20_000,
            MaximumVisitedNodes: 50_000,
            RouteGraceMilliseconds: 2000,
            Driving: new AiPursuitDrivingOptions(
                Enabled: true,
                ContactEnabled: true,
                CatchUpDistanceMeters: 15,
                CloseDistanceMeters: 15,
                ContactDistanceMeters: 3,
                MaximumSpeedMetersPerSecond: 50,
                MaximumClosingSpeedMetersPerSecond: 10,
                ContactClosingSpeedMetersPerSecond: 2));

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            AiPursuitControl.ValidateTrackingOptions(options));
    }

    [Test]
    public void CoreRejectsPitWithoutAggressiveContact()
    {
        var options = DrivingOptions with
        {
            ContactEnabled = false,
            Pit = new AiPursuitPitOptions(true, 6, 10 / 3.6f, 0.8f, 1200, 3000)
        };

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            AiPursuitControl.ValidateDrivingOptions(options));
    }

    [Test]
    public void CoreRejectsPitOffsetThatExceedsConservativeLaneAllowance()
    {
        var options = DrivingOptions with
        {
            Pit = new AiPursuitPitOptions(true, 6, 10 / 3.6f, 1.2f, 1200, 3000)
        };

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            AiPursuitControl.ValidateDrivingOptions(options));
    }

    [TestCase((byte)10, true, true, (byte)10, true)]
    [TestCase((byte)10, true, true, (byte)11, false)]
    [TestCase(null, false, false, (byte)10, false)]
    [TestCase((byte)10, true, false, (byte)10, false)]
    public void CollisionDispositionRequiresLivePursuitForExactSender(
        byte? pursuitTargetSessionId,
        bool aggressiveDrivingEnabled,
        bool hasDrivingState,
        byte senderSessionId,
        bool expectedRecovery)
    {
        var disposition = AiPursuitControl.ResolveCollisionDisposition(
            pursuitTargetSessionId,
            aggressiveDrivingEnabled,
            hasDrivingState,
            senderSessionId);

        Assert.That(disposition, Is.EqualTo(expectedRecovery
            ? AiCollisionDisposition.PursuitRecovery
            : AiCollisionDisposition.NativeStop));
    }

    [Test]
    public void CollisionRecoveryPreservesRouteAndReturnsRecoveryDiagnostics()
    {
        var controller = new AiPursuitDrivingController();
        var active = controller.Update(DrivingRequest(2, 1, routeRevision: 7));
        var collision = controller.ReportCollision(active.State, nowMilliseconds: 1000);

        var recovery = controller.Update(DrivingRequest(
            2,
            1,
            routeRevision: 7,
            previous: collision));

        Assert.Multiple(() =>
        {
            Assert.That(recovery.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.Recovery));
            Assert.That(recovery.Diagnostics.Reason, Is.EqualTo(AiPursuitDrivingReason.CollisionRecovery));
            Assert.That(recovery.Diagnostics.CollisionReported, Is.False);
            Assert.That(recovery.State.RouteRevision, Is.EqualTo(7));
        });
    }

    [Test]
    public void RecoveryReturnsToClosePressureWhenPhysicalClearanceReopens()
    {
        var controller = new AiPursuitDrivingController();
        var active = controller.Update(DrivingRequest(2, 1));
        var collision = controller.ReportCollision(active.State, nowMilliseconds: 1000);

        var recovered = controller.Update(DrivingRequest(6, 6, previous: collision));

        Assert.Multiple(() =>
        {
            Assert.That(recovered.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.ClosePressure));
            Assert.That(recovered.Diagnostics.CollisionReported, Is.False);
        });
    }

    [Test]
    public void RecoveryReturnsDirectlyToCatchUpWhenTargetOpensDistance()
    {
        var controller = new AiPursuitDrivingController();
        var active = controller.Update(DrivingRequest(2, 1));
        var collision = controller.ReportCollision(active.State, nowMilliseconds: 1000);

        var recovered = controller.Update(DrivingRequest(150, 140, previous: collision));

        Assert.That(recovered.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.CatchUp));
    }

    [Test]
    public void CollisionEventIsDeliveredBeforeImmediateSeparationTransition()
    {
        var controller = new AiPursuitDrivingController();
        var active = controller.Update(DrivingRequest(2, 1));
        var collisionState = controller.ReportCollision(active.State, nowMilliseconds: 1000);
        var collision = active.Diagnostics with
        {
            Revision = collisionState.DiagnosticRevision,
            State = AiPursuitDrivingState.Recovery,
            Reason = AiPursuitDrivingReason.CollisionRecovery,
            CollisionReported = true
        };
        var separated = controller.Update(DrivingRequest(150, 140, previous: collisionState));

        Assert.Multiple(() =>
        {
            Assert.That(AiPursuitControl.SelectDrivingDiagnosticsForDelivery(
                collision,
                separated.Diagnostics), Is.EqualTo(collision));
            Assert.That(AiPursuitControl.SelectDrivingDiagnosticsForDelivery(
                separated.Diagnostics,
                separated.Diagnostics), Is.EqualTo(separated.Diagnostics));
            Assert.That(separated.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.CatchUp));
        });
    }

    [Test]
    public void CollisionUpdateCannotBeOverwrittenByAnEarlierTrackingRead()
    {
        var gate = new AiPursuitUpdateGate();
        var state = 1;
        using var trackingRead = new ManualResetEventSlim();
        using var resumeTracking = new ManualResetEventSlim();
        using var collisionAttempted = new ManualResetEventSlim();
        var tracking = Task.Run(() => gate.Run(() =>
        {
            var previous = state;
            trackingRead.Set();
            if (!resumeTracking.Wait(TimeSpan.FromSeconds(5)))
                throw new TimeoutException();
            state = previous + 2;
        }));

        try
        {
            Assert.That(trackingRead.Wait(TimeSpan.FromSeconds(5)), Is.True);
            var collision = Task.Run(() =>
            {
                collisionAttempted.Set();
                gate.Run(() => state = 2);
            });
            Assert.That(collisionAttempted.Wait(TimeSpan.FromSeconds(5)), Is.True);
            resumeTracking.Set();
            Assert.That(Task.WaitAll([tracking, collision], TimeSpan.FromSeconds(5)), Is.True);
            Assert.That(state, Is.EqualTo(2));
        }
        finally
        {
            resumeTracking.Set();
        }
    }

    [Test]
    public void PitContactIsDeliveredOnceAcrossNextTrackingUpdate()
    {
        var contact = new AiPursuitPitDiagnostics(3,
            AiPursuitPitEventKind.Contact, AiPursuitPitSide.Left,
            AiPursuitPitAbortReason.None, 3, 1, .8f);
        var next = new AiPursuitPitDiagnostics(4,
            AiPursuitPitEventKind.Armed, AiPursuitPitSide.Right,
            AiPursuitPitAbortReason.None, 4, 1, 0);
        Assert.Multiple(() =>
        {
            Assert.That(AiPursuitControl.SelectPitDiagnosticsForDelivery(contact, null),
                Is.EqualTo(contact));
            Assert.That(AiPursuitControl.SelectPitDiagnosticsForDelivery(contact, next),
                Is.EqualTo(contact));
            Assert.That(AiPursuitControl.SelectPitDiagnosticsForDelivery(null, next),
                Is.EqualTo(next));
        });
    }

    private static AiPursuitDrivingRequest DrivingRequest(
        float routeDistance,
        float clearance,
        long routeRevision = 1,
        AiPursuitDrivingControllerState? previous = null) =>
        new(
            DrivingOptions,
            routeDistance,
            clearance,
            TargetSpeedMetersPerSecond: 20,
            PoliceSpeedMetersPerSecond: 20,
            AiLaneChangePhase.None,
            routeRevision,
            NowMilliseconds: 1500,
            previous);
}
