using AssettoServer.Server.Ai;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitDrivingControllerTests
{
    private static readonly AiPursuitDrivingOptions DefaultOptions = new(
        Enabled: true,
        ContactEnabled: true,
        CatchUpDistanceMeters: 100,
        CloseDistanceMeters: 15,
        ContactDistanceMeters: 3,
        MaximumSpeedMetersPerSecond: 50,
        MaximumClosingSpeedMetersPerSecond: 35 / 3.6f,
        ContactClosingSpeedMetersPerSecond: 5 / 3.6f);

    [TestCase(150, 140, AiPursuitDrivingState.CatchUp)]
    [TestCase(60, 55, AiPursuitDrivingState.Approach)]
    [TestCase(12, 10, AiPursuitDrivingState.ClosePressure)]
    [TestCase(4, 2, AiPursuitDrivingState.Contact)]
    public void SelectsStateFromRouteAndPhysicalDistance(
        float routeDistance,
        float clearance,
        AiPursuitDrivingState expected)
    {
        var decision = CreateController().Update(Request(routeDistance, clearance));

        Assert.That(decision.Diagnostics.State, Is.EqualTo(expected));
    }

    [Test]
    public void HysteresisRetainsCatchUpUntilDistanceFallsTenPercentBelowThreshold()
    {
        var controller = CreateController();
        var catchUp = controller.Update(Request(105, 100));

        var retained = controller.Update(Request(95, 90, previous: catchUp.State));
        var released = controller.Update(Request(89, 85, previous: retained.State));

        Assert.Multiple(() =>
        {
            Assert.That(retained.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.CatchUp));
            Assert.That(released.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.Approach));
        });
    }

    [Test]
    public void HysteresisRetainsCloseAndContactUntilClearanceExceedsExitThreshold()
    {
        var controller = CreateController();
        var close = controller.Update(Request(14, 14));
        var retainedClose = controller.Update(Request(16, 16, previous: close.State));
        var releasedClose = controller.Update(Request(17, 17, previous: retainedClose.State));
        var contact = controller.Update(Request(3, 3));
        var retainedContact = controller.Update(Request(3.2f, 3.2f, previous: contact.State));
        var releasedContact = controller.Update(Request(3.4f, 3.4f, previous: retainedContact.State));

        Assert.Multiple(() =>
        {
            Assert.That(retainedClose.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.ClosePressure));
            Assert.That(releasedClose.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.Approach));
            Assert.That(retainedContact.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.Contact));
            Assert.That(releasedContact.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.ClosePressure));
        });
    }

    [Test]
    public void DistantFasterPlayerStillRequestsCatchUpAbovePlayerSpeed()
    {
        var decision = UpdateKph(routeDistance: 150, clearance: 140, policeKph: 80, playerKph: 120);

        Assert.That(decision.RequestedSpeedMetersPerSecond * 3.6f, Is.GreaterThan(120));
    }

    [Test]
    public void ExcessClosingSpeedRequestsBrakingDuringApproach()
    {
        var decision = UpdateKph(routeDistance: 20, clearance: 20, policeKph: 150, playerKph: 40);

        Assert.Multiple(() =>
        {
            Assert.That(decision.RequestedSpeedMetersPerSecond * 3.6f, Is.LessThan(150));
            Assert.That(decision.Diagnostics.Reason, Is.EqualTo(AiPursuitDrivingReason.ExcessClosingSpeed));
        });
    }

    [Test]
    public void ClosePressureTracksPlayerWithoutRestoringFiftyMeterGap()
    {
        var decision = UpdateKph(routeDistance: 7, clearance: 5, policeKph: 125, playerKph: 120);

        Assert.That(decision.RequestedSpeedMetersPerSecond * 3.6f, Is.InRange(120, 130));
    }

    [Test]
    public void ContactUsesConfiguredControlledClosingSpeed()
    {
        var decision = UpdateKph(routeDistance: 2, clearance: 1, policeKph: 125, playerKph: 120);

        Assert.Multiple(() =>
        {
            Assert.That(decision.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.Contact));
            Assert.That(decision.Diagnostics.DesiredClosingSpeedMetersPerSecond * 3.6f,
                Is.EqualTo(5).Within(0.01));
            Assert.That(decision.RequestedSpeedMetersPerSecond * 3.6f,
                Is.LessThanOrEqualTo(125.01f));
        });
    }

    [Test]
    public void DisabledContactRequestsZeroClosingSpeed()
    {
        var options = DefaultOptions with { ContactEnabled = false };
        var decision = CreateController().Update(Request(2, 1, options: options));

        Assert.Multiple(() =>
        {
            Assert.That(decision.Diagnostics.DesiredClosingSpeedMetersPerSecond, Is.Zero);
            Assert.That(decision.Diagnostics.Reason, Is.EqualTo(AiPursuitDrivingReason.ContactDisabled));
        });
    }

    [Test]
    public void StoppedTargetAndHighPoliceSpeedDoesNotRequestBlindAcceleration()
    {
        var decision = CreateController().Update(Request(
            routeDistance: 20,
            clearance: 20,
            targetSpeed: 0,
            policeSpeed: 150 / 3.6f));

        Assert.Multiple(() =>
        {
            Assert.That(decision.RequestedSpeedMetersPerSecond, Is.LessThan(150 / 3.6f));
            Assert.That(float.IsFinite(decision.RequestedSpeedMetersPerSecond), Is.True);
        });
    }

    [Test]
    public void NegativeClosingSpeedRequestsAcceleration()
    {
        var decision = UpdateKph(routeDistance: 60, clearance: 55, policeKph: 80, playerKph: 120);

        Assert.That(decision.RequestedSpeedMetersPerSecond, Is.GreaterThan(80 / 3.6f));
    }

    [Test]
    public void RequestedSpeedNeverExceedsConfiguredMaximum()
    {
        var options = DefaultOptions with { MaximumSpeedMetersPerSecond = 130 / 3.6f };
        var decision = CreateController().Update(Request(
            routeDistance: 200,
            clearance: 190,
            targetSpeed: 125 / 3.6f,
            policeSpeed: 100 / 3.6f,
            options: options));

        Assert.That(decision.RequestedSpeedMetersPerSecond * 3.6f, Is.EqualTo(130).Within(0.01));
    }

    [TestCase(float.NaN, 10, 10, 10)]
    [TestCase(10, float.PositiveInfinity, 10, 10)]
    [TestCase(10, 10, float.NaN, 10)]
    [TestCase(10, 10, 10, float.NegativeInfinity)]
    public void InvalidMeasurementsProduceBoundedConservativeRequest(
        float routeDistance,
        float clearance,
        float targetSpeed,
        float policeSpeed)
    {
        var decision = CreateController().Update(Request(
            routeDistance,
            clearance,
            targetSpeed,
            policeSpeed));

        Assert.Multiple(() =>
        {
            Assert.That(decision.RequestedSpeedMetersPerSecond, Is.Zero);
            Assert.That(float.IsFinite(decision.Diagnostics.RouteDistanceMeters), Is.True);
            Assert.That(float.IsFinite(decision.Diagnostics.PhysicalClearanceMeters), Is.True);
            Assert.That(float.IsFinite(decision.Diagnostics.ClosingSpeedMetersPerSecond), Is.True);
            Assert.That(decision.Diagnostics.Reason, Is.EqualTo(AiPursuitDrivingReason.InvalidMeasurement));
        });
    }

    [Test]
    public void ClosingSpeedDeadbandPreservesPreviousRequest()
    {
        var controller = CreateController();
        var initial = controller.Update(Request(2, 1, targetSpeed: 20, policeSpeed: 10));
        var withinDeadband = controller.Update(Request(
            2,
            1,
            targetSpeed: 20,
            policeSpeed: 20 + 5 / 3.6f + 0.2f,
            previous: initial.State));

        Assert.That(withinDeadband.RequestedSpeedMetersPerSecond,
            Is.EqualTo(initial.RequestedSpeedMetersPerSecond));
    }

    [Test]
    public void DiagnosticRevisionChangesOnlyForSemanticTransitions()
    {
        var controller = CreateController();
        var first = controller.Update(Request(150, 140));
        var measurementsOnly = controller.Update(Request(145, 135, previous: first.State));
        var stateChange = controller.Update(Request(60, 55, previous: measurementsOnly.State));
        var collision = controller.ReportCollision(stateChange.State, nowMilliseconds: 1000);

        Assert.Multiple(() =>
        {
            Assert.That(first.Diagnostics.Revision, Is.EqualTo(1));
            Assert.That(measurementsOnly.Diagnostics.Revision, Is.EqualTo(1));
            Assert.That(stateChange.Diagnostics.Revision, Is.EqualTo(2));
            Assert.That(collision.DiagnosticRevision, Is.EqualTo(3));
            Assert.That(collision.State, Is.EqualTo(AiPursuitDrivingState.Recovery));
        });
    }

    [Test]
    public void ChangingLaneKeepsClosingEnvelopeWithoutArtificialPenalty()
    {
        var normal = CreateController().Update(Request(150, 140));
        var changing = CreateController().Update(Request(
            150,
            140,
            laneChangePhase: AiLaneChangePhase.Changing));

        Assert.Multiple(() =>
        {
            Assert.That(changing.Diagnostics.DesiredClosingSpeedMetersPerSecond,
                Is.EqualTo(normal.Diagnostics.DesiredClosingSpeedMetersPerSecond).Within(0.001));
            Assert.That(changing.Diagnostics.DesiredClosingSpeedMetersPerSecond, Is.GreaterThan(0));
            Assert.That(changing.Diagnostics.Reason, Is.EqualTo(normal.Diagnostics.Reason));
        });
    }

    [TestCase(150)]
    [TestCase(300)]
    public void DistantCatchUpImmediatelyUsesConfiguredClosingCap(float distance)
    {
        var decision = CreateController().Update(Request(
            distance, distance - 10, targetSpeed: 40, policeSpeed: 25));

        Assert.Multiple(() =>
        {
            Assert.That(decision.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.CatchUp));
            Assert.That(decision.Diagnostics.DesiredClosingSpeedMetersPerSecond,
                Is.EqualTo(35 / 3.6f).Within(0.001));
            Assert.That(decision.RequestedSpeedMetersPerSecond, Is.LessThanOrEqualTo(50));
        });
    }

    [Test]
    public void SeventyMeterApproachFrontLoadsClosingIntent()
    {
        var decision = CreateController().Update(Request(70, 65));

        Assert.Multiple(() =>
        {
            Assert.That(decision.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.Approach));
            Assert.That(decision.Diagnostics.DesiredClosingSpeedMetersPerSecond, Is.GreaterThan(8.5f));
            Assert.That(decision.Diagnostics.DesiredClosingSpeedMetersPerSecond,
                Is.LessThanOrEqualTo(35 / 3.6f));
        });
    }

    [Test]
    public void TenMeterRearPressureStillConvergesToExistingContactSpeed()
    {
        var controller = CreateController();
        var close = controller.Update(Request(10, 10));
        var contact = controller.Update(Request(2.8f, 2.8f, previous: close.State));

        Assert.Multiple(() =>
        {
            Assert.That(close.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.ClosePressure));
            Assert.That(close.Diagnostics.DesiredClosingSpeedMetersPerSecond, Is.GreaterThan(3));
            Assert.That(contact.Diagnostics.State, Is.EqualTo(AiPursuitDrivingState.Contact));
            Assert.That(contact.Diagnostics.DesiredClosingSpeedMetersPerSecond,
                Is.EqualTo(5 / 3.6f).Within(0.001));
        });
    }

    [TestCase(AiLaneChangePhase.WaitingForGap)]
    [TestCase(AiLaneChangePhase.Cooldown)]
    public void NonChangingLanePhasesKeepNormalClosingEnvelope(AiLaneChangePhase phase)
    {
        var normal = CreateController().Update(Request(150, 140));
        var phased = CreateController().Update(Request(150, 140, laneChangePhase: phase));

        Assert.That(phased.Diagnostics.DesiredClosingSpeedMetersPerSecond,
            Is.EqualTo(normal.Diagnostics.DesiredClosingSpeedMetersPerSecond).Within(0.001));
    }

    [Test]
    public void RouteRevisionDoesNotResetLongitudinalState()
    {
        var controller = CreateController();
        var first = controller.Update(Request(60, 55, routeRevision: 1));
        var revised = controller.Update(Request(59, 54, routeRevision: 2, previous: first.State));

        Assert.Multiple(() =>
        {
            Assert.That(revised.Diagnostics.State, Is.EqualTo(first.Diagnostics.State));
            Assert.That(revised.Diagnostics.Revision, Is.EqualTo(first.Diagnostics.Revision));
            Assert.That(revised.State.RouteRevision, Is.EqualTo(2));
        });
    }

    private static AiPursuitDrivingController CreateController() => new();

    private static AiPursuitDrivingDecision UpdateKph(
        float routeDistance,
        float clearance,
        float policeKph,
        float playerKph) =>
        CreateController().Update(Request(
            routeDistance,
            clearance,
            playerKph / 3.6f,
            policeKph / 3.6f));

    private static AiPursuitDrivingRequest Request(
        float routeDistance,
        float clearance,
        float targetSpeed = 20,
        float policeSpeed = 20,
        AiLaneChangePhase laneChangePhase = AiLaneChangePhase.None,
        long routeRevision = 1,
        AiPursuitDrivingControllerState? previous = null,
        AiPursuitDrivingOptions? options = null) =>
        new(
            options ?? DefaultOptions,
            routeDistance,
            clearance,
            targetSpeed,
            policeSpeed,
            laneChangePhase,
            routeRevision,
            NowMilliseconds: 500,
            previous);
}
