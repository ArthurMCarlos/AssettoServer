using AssettoServer.Server.Ai;
using NUnit.Framework;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitPitControllerTests
{
    private static readonly AiPursuitPitOptions Options = new(true, 6, 10 / 3.6f, .8f, 1200, 3000);

    private static AiPursuitPitRequest Request(
        AiPursuitPitControllerState? previous = null,
        long time = 1000,
        float clearance = 4,
        float closing = 1,
        bool leftSafe = true,
        bool rightSafe = true,
        bool routeValid = true,
        bool sameLane = true,
        bool behind = true,
        bool junctionNear = false,
        AiLaneChangePhase lanePhase = AiLaneChangePhase.None,
        AiPursuitDrivingState driving = AiPursuitDrivingState.ClosePressure,
        byte target = 10,
        AiPursuitPitOptions? options = null,
        AiPursuitDrivingReason reason = AiPursuitDrivingReason.ClosePressure) =>
        new(options ?? Options, target, routeValid, sameLane, behind,
            clearance, closing, driving, reason, lanePhase, junctionNear,
            leftSafe, rightSafe, 1, time, previous);

    [Test]
    public void EligibleTargetArmsAndThenStartsStickySide()
    {
        var controller = new AiPursuitPitController();
        var armed = controller.Update(Request());
        Assert.Multiple(() =>
        {
            Assert.That(armed.State.Phase, Is.EqualTo(AiPursuitPitPhase.Armed));
            Assert.That(armed.State.Side, Is.EqualTo(AiPursuitPitSide.Left));
            Assert.That(armed.DesiredOffsetMeters, Is.Zero);
            Assert.That(armed.Diagnostics?.EventKind, Is.EqualTo(AiPursuitPitEventKind.Armed));
        });
        var started = controller.Update(Request(armed.State, 1100, rightSafe: false));
        Assert.Multiple(() =>
        {
            Assert.That(started.State.Phase, Is.EqualTo(AiPursuitPitPhase.Attempting));
            Assert.That(started.State.Side, Is.EqualTo(AiPursuitPitSide.Left));
            Assert.That(started.DesiredOffsetMeters, Is.EqualTo(.8f));
            Assert.That(started.Diagnostics?.EventKind, Is.EqualTo(AiPursuitPitEventKind.Started));
        });
        Assert.That(controller.Update(Request(started.State, 1200)).Diagnostics, Is.Null);
    }

    [Test]
    public void ChoosesRightOnlyWhenLeftBlockedAndNeverSwitchesDuringAttempt()
    {
        var controller = new AiPursuitPitController();
        var armed = controller.Update(Request(leftSafe: false));
        var attempt = controller.Update(Request(armed.State, 1100, leftSafe: false));
        Assert.That(attempt.DesiredOffsetMeters, Is.EqualTo(-.8f));
        var aborted = controller.Update(Request(attempt.State, 1200, leftSafe: true, rightSafe: false));
        Assert.Multiple(() =>
        {
            Assert.That(aborted.State.Phase, Is.EqualTo(AiPursuitPitPhase.Cooldown));
            Assert.That(aborted.DesiredOffsetMeters, Is.Zero);
            Assert.That(aborted.Diagnostics?.Reason, Is.EqualTo(AiPursuitPitAbortReason.BlockedSide));
        });
    }

    [TestCase(7, 1, true, true, true, false, AiLaneChangePhase.None, AiPursuitDrivingState.ClosePressure)]
    [TestCase(4, 4, true, true, true, false, AiLaneChangePhase.None, AiPursuitDrivingState.ClosePressure)]
    [TestCase(4, 1, false, true, true, false, AiLaneChangePhase.None, AiPursuitDrivingState.ClosePressure)]
    [TestCase(4, 1, true, false, true, false, AiLaneChangePhase.None, AiPursuitDrivingState.ClosePressure)]
    [TestCase(4, 1, true, true, false, false, AiLaneChangePhase.None, AiPursuitDrivingState.ClosePressure)]
    [TestCase(4, 1, true, true, true, true, AiLaneChangePhase.None, AiPursuitDrivingState.ClosePressure)]
    [TestCase(4, 1, true, true, true, false, AiLaneChangePhase.WaitingForGap, AiPursuitDrivingState.ClosePressure)]
    [TestCase(4, 1, true, true, true, false, AiLaneChangePhase.Changing, AiPursuitDrivingState.ClosePressure)]
    [TestCase(4, 1, true, true, true, false, AiLaneChangePhase.None, AiPursuitDrivingState.Recovery)]
    public void UnsafeConditionsDoNotArm(float clearance, float closing, bool routeValid,
        bool sameLane, bool behind, bool junctionNear, AiLaneChangePhase lanePhase,
        AiPursuitDrivingState driving)
    {
        var result = new AiPursuitPitController().Update(Request(clearance: clearance,
            closing: closing, routeValid: routeValid, sameLane: sameLane, behind: behind,
            junctionNear: junctionNear, lanePhase: lanePhase, driving: driving));
        Assert.That(result.State.Phase, Is.EqualTo(AiPursuitPitPhase.Idle));
    }

    [Test]
    public void IdleRejectionRetainsTheControllerReasonWithoutEmittingPitEvent()
    {
        var result = new AiPursuitPitController().Update(Request(
            clearance: 4, closing: 1, sameLane: false));

        Assert.Multiple(() =>
        {
            Assert.That(result.State.Phase, Is.EqualTo(AiPursuitPitPhase.Idle));
            Assert.That(result.Diagnostics, Is.Null);
            Assert.That(result.EligibilityRejection,
                Is.EqualTo(AiPursuitPitAbortReason.GeometryInvalid));
        });
    }

    [Test]
    public void RouteLossAndTargetSwitchAbortAndCooldown()
    {
        var controller = new AiPursuitPitController();
        var armed = controller.Update(Request());
        var attempt = controller.Update(Request(armed.State, 1100));
        var lost = controller.Update(Request(attempt.State, 1200, routeValid: false));
        Assert.That(lost.Diagnostics?.Reason, Is.EqualTo(AiPursuitPitAbortReason.RouteLost));
        Assert.That(controller.Update(Request(lost.State, 2000)).State.Phase,
            Is.EqualTo(AiPursuitPitPhase.Cooldown));
        var ready = controller.Update(Request(lost.State, 4300));
        Assert.That(ready.State.Phase, Is.EqualTo(AiPursuitPitPhase.Idle));
        Assert.That(controller.Update(Request(ready.State, 4400)).State.Phase,
            Is.EqualTo(AiPursuitPitPhase.Armed));
        var changed = controller.Update(Request(attempt.State, 1200, target: 11));
        Assert.That(changed.Diagnostics?.Reason, Is.EqualTo(AiPursuitPitAbortReason.TargetChanged));
    }

    [Test]
    public void CollisionEndsAttemptExactlyOnce()
    {
        var controller = new AiPursuitPitController();
        var armed = controller.Update(Request());
        var attempt = controller.Update(Request(armed.State, 1100));
        var contact = controller.ReportCollision(attempt.State, 1200);
        Assert.Multiple(() =>
        {
            Assert.That(contact.State.Phase, Is.EqualTo(AiPursuitPitPhase.Cooldown));
            Assert.That(contact.DesiredOffsetMeters, Is.Zero);
            Assert.That(contact.Diagnostics?.EventKind, Is.EqualTo(AiPursuitPitEventKind.Contact));
            Assert.That(contact.Diagnostics?.PhysicalClearanceMeters, Is.EqualTo(4));
            Assert.That(contact.Diagnostics?.OffsetMeters, Is.EqualTo(.8f));
            Assert.That(controller.ReportCollision(contact.State, 1300).Diagnostics, Is.Null);
            Assert.That(controller.Update(Request(contact.State, 2000)).State.Phase,
                Is.EqualTo(AiPursuitPitPhase.Cooldown));
        });
    }

    [Test]
    public void CommitExpiryEndsAttemptAndEnforcesCooldown()
    {
        var controller = new AiPursuitPitController();
        var armed = controller.Update(Request());
        var attempt = controller.Update(Request(armed.State, 1100));
        var expired = controller.Update(Request(attempt.State, 2300));
        Assert.Multiple(() =>
        {
            Assert.That(expired.State.Phase, Is.EqualTo(AiPursuitPitPhase.Cooldown));
            Assert.That(expired.Diagnostics?.Reason,
                Is.EqualTo(AiPursuitPitAbortReason.CommitElapsed));
            Assert.That(controller.Update(Request(expired.State, 4000)).State.Phase,
                Is.EqualTo(AiPursuitPitPhase.Cooldown));
            Assert.That(controller.Update(Request(expired.State, 5300)).State.Phase,
                Is.EqualTo(AiPursuitPitPhase.Idle));
        });
    }

    [Test]
    public void DisabledPitAndRouteRevisionChangeCannotContinueAnAttempt()
    {
        var controller = new AiPursuitPitController();
        var disabled = Options with { Enabled = false };
        Assert.That(controller.Update(Request(options: disabled)).State.Phase,
            Is.EqualTo(AiPursuitPitPhase.Idle));

        var armed = controller.Update(Request());
        var attempt = controller.Update(Request(armed.State, 1100));
        var revised = controller.Update(Request(attempt.State, 1200) with { RouteRevision = 2 });
        Assert.That(revised.Diagnostics?.Reason, Is.EqualTo(AiPursuitPitAbortReason.RouteLost));
    }

    [Test]
    public void ExcessClosingReasonPreventsPitEvenBelowPitSpeedCap()
    {
        var result = new AiPursuitPitController().Update(Request(
            closing: 2, reason: AiPursuitDrivingReason.ExcessClosingSpeed));
        Assert.That(result.State.Phase, Is.EqualTo(AiPursuitPitPhase.Idle));
    }

    [Test]
    public void ZeroCooldownStillDeliversContactBeforeRearming()
    {
        var controller = new AiPursuitPitController();
        var options = Options with { CooldownMilliseconds = 0 };
        var armed = controller.Update(Request(options: options));
        var attempt = controller.Update(Request(armed.State, 1100, options: options));
        var contact = controller.ReportCollision(attempt.State, 1200);
        var neutral = controller.Update(Request(contact.State, 1300, options: options));
        Assert.Multiple(() =>
        {
            Assert.That(neutral.State.Phase, Is.EqualTo(AiPursuitPitPhase.Idle));
            Assert.That(neutral.Diagnostics, Is.Null);
            Assert.That(AiPursuitControl.SelectPitDiagnosticsForDelivery(
                contact.Diagnostics, neutral.Diagnostics), Is.EqualTo(contact.Diagnostics));
        });
        Assert.That(controller.Update(Request(neutral.State, 1400, options: options))
            .Diagnostics?.EventKind, Is.EqualTo(AiPursuitPitEventKind.Armed));
    }

    [Test]
    public void OffsetMovesAndReturnsByBoundedSteps()
    {
        Assert.Multiple(() =>
        {
            Assert.That(AiPursuitPitController.ApproachOffset(0, .8f, .1f), Is.EqualTo(.1f).Within(.0001));
            Assert.That(AiPursuitPitController.ApproachOffset(.8f, 0, .1f), Is.EqualTo(.7f).Within(.0001));
            Assert.That(AiPursuitPitController.ApproachOffset(.05f, 0, .1f), Is.Zero);
        });
    }
}
