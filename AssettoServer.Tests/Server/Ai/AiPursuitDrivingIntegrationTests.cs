using AssettoServer.Server.Ai;

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
            Assert.That(recovery.Diagnostics.CollisionReported, Is.True);
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
