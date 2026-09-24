using System.Numerics;
using AssettoServer.Server.Ai;
using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitPitRouteContinuityTests
{
    private static AiPursuitPitRequest Request(AiPursuitPitControllerState? previous = null,
        long revision = 45, long time = 1000) =>
        new(new(true, 6, 10 / 3.6f, .8f, 1200, 3000), 10,
            true, true, true, 5.7f, 8.7f / 3.6f,
            AiPursuitDrivingState.ClosePressure, AiPursuitDrivingReason.ClosePressure,
            AiLaneChangePhase.None, false, true, true, revision, time, previous);

    [Test]
    public void StartedDeliversCurrentContinuityOnlyOnceWithoutChangingDecision()
    {
        var controller = new AiPursuitPitController();
        var armed = controller.Update(Request());
        var context = new AiPursuitPitContinuityDiagnostics(45, 46, AiPursuitPitPhase.Armed,
            "Active", true, true, 170960, 170962, 3, true, AiLaneChangePhase.Cooldown,
            false, true, true);
        var request = Request(armed.State, 46, 1100);
        var started = controller.Update(request with { Continuity = context });
        Assert.Multiple(() =>
        {
            Assert.That(started.Diagnostics?.Continuity, Is.EqualTo(context));
            Assert.That(started.State, Is.EqualTo(controller.Update(request).State));
            Assert.That(controller.Update(Request(started.State, 47, 1200) with
                { Continuity = context }).Diagnostics, Is.Null);
        });
    }

    [Test]
    public void RouteLostDeliversUnknownCurrentRouteRatherThanArmedSnapshot()
    {
        var controller = new AiPursuitPitController();
        var armed = controller.Update(Request());
        var context = new AiPursuitPitContinuityDiagnostics(45, null, AiPursuitPitPhase.Armed,
            "NoRoute", false, false, 170960, null, null, null,
            AiLaneChangePhase.None, null, null, null);
        var aborted = controller.Update(Request(armed.State, 45, 1100) with
            { RouteValid = false, Continuity = context });
        Assert.Multiple(() =>
        {
            Assert.That(aborted.Diagnostics?.Reason, Is.EqualTo(AiPursuitPitAbortReason.RouteLost));
            Assert.That(aborted.Diagnostics?.Continuity, Is.EqualTo(context));
        });
    }

    [Test]
    public void ValidRevisionChangeStartsArmedPitWithoutChangingArmedRevisionOrSide()
    {
        var controller = new AiPursuitPitController();
        var armed = controller.Update(Request());
        var started = controller.Update(Request(armed.State, 46, 1100));
        Assert.Multiple(() =>
        {
            Assert.That(started.Diagnostics?.EventKind, Is.EqualTo(AiPursuitPitEventKind.Started));
            Assert.That(started.State.Phase, Is.EqualTo(AiPursuitPitPhase.Attempting));
            Assert.That(started.State.RouteRevision, Is.EqualTo(45));
            Assert.That(started.State.Side, Is.EqualTo(AiPursuitPitSide.Left));
            Assert.That(started.DesiredOffsetMeters, Is.EqualTo(.8f));
        });
    }

    [Test]
    public void RepeatedRevisionsDoNotRestartCommitClockOrEmitRepeatedStarted()
    {
        var controller = new AiPursuitPitController();
        var armed = controller.Update(Request());
        var started = controller.Update(Request(armed.State, 46, 1100));
        var continuing = controller.Update(Request(started.State, 47, 2200));
        Assert.Multiple(() =>
        {
            Assert.That(continuing.State.Phase, Is.EqualTo(AiPursuitPitPhase.Attempting));
            Assert.That(continuing.State.AttemptStartedAtMilliseconds, Is.EqualTo(1100));
            Assert.That(continuing.Diagnostics, Is.Null);
            Assert.That(continuing.DesiredOffsetMeters, Is.EqualTo(.8f));
        });
        var expired = controller.Update(Request(continuing.State, 48, 2300));
        Assert.That(expired.Diagnostics?.Reason, Is.EqualTo(AiPursuitPitAbortReason.CommitElapsed));
    }

    [TestCase(AiPursuitPitAbortReason.RouteLost)]
    [TestCase(AiPursuitPitAbortReason.GeometryInvalid)]
    [TestCase(AiPursuitPitAbortReason.Junction)]
    [TestCase(AiPursuitPitAbortReason.LaneChange)]
    [TestCase(AiPursuitPitAbortReason.BlockedSide)]
    [TestCase(AiPursuitPitAbortReason.TargetChanged)]
    [TestCase(AiPursuitPitAbortReason.OutOfRange)]
    [TestCase(AiPursuitPitAbortReason.ClosingSpeedUnsafe)]
    [TestCase(AiPursuitPitAbortReason.Recovery)]
    public void NewRevisionStillAbortsForActualUnsafeCondition(AiPursuitPitAbortReason expected)
    {
        var controller = new AiPursuitPitController();
        var armed = controller.Update(Request());
        var request = Request(armed.State, 46, 1100);
        request = expected switch
        {
            AiPursuitPitAbortReason.RouteLost => request with { RouteValid = false },
            AiPursuitPitAbortReason.GeometryInvalid => request with { SameLane = false },
            AiPursuitPitAbortReason.Junction => request with { JunctionNear = true },
            AiPursuitPitAbortReason.LaneChange => request with { LaneChangePhase = AiLaneChangePhase.Changing },
            AiPursuitPitAbortReason.BlockedSide => request with { LeftSafe = false },
            AiPursuitPitAbortReason.TargetChanged => request with { TargetSessionId = 11 },
            AiPursuitPitAbortReason.OutOfRange => request with { PhysicalClearanceMeters = 6.1f },
            AiPursuitPitAbortReason.ClosingSpeedUnsafe => request with { ClosingSpeedMetersPerSecond = 3 },
            AiPursuitPitAbortReason.Recovery => request with { DrivingState = AiPursuitDrivingState.Recovery },
            _ => throw new ArgumentOutOfRangeException(nameof(expected))
        };
        var aborted = controller.Update(request);
        Assert.Multiple(() =>
        {
            Assert.That(aborted.Diagnostics?.EventKind, Is.EqualTo(AiPursuitPitEventKind.Aborted));
            Assert.That(aborted.Diagnostics?.Reason, Is.EqualTo(expected));
            Assert.That(aborted.State.Phase, Is.EqualTo(AiPursuitPitPhase.Cooldown));
            Assert.That(aborted.DesiredOffsetMeters, Is.Zero);
        });
    }

    [Test]
    public void NewRevisionCannotContinueWhenTargetBehindOrWaitingForLaneGap()
    {
        var controller = new AiPursuitPitController();
        var armed = controller.Update(Request());
        Assert.That(controller.Update(Request(armed.State, 46) with { PoliceBehindTarget = false })
            .Diagnostics?.Reason, Is.EqualTo(AiPursuitPitAbortReason.GeometryInvalid));
        Assert.That(controller.Update(Request(armed.State, 46) with
            { LaneChangePhase = AiLaneChangePhase.WaitingForGap }).Diagnostics?.Reason,
            Is.EqualTo(AiPursuitPitAbortReason.LaneChange));
    }

    [TestCase(1, 3, 3f, AiPursuitRouteUpdateKind.Extended)]
    [TestCase(4, 5, 1.5f, AiPursuitRouteUpdateKind.Recalculated)]
    [TestCase(4, 4, 0f, AiPursuitRouteUpdateKind.Recalculated)]
    public void AdvancingPointsAndShortActivePlanCanStartPit(int policePoint, int targetPoint,
        float expectedDistance, AiPursuitRouteUpdateKind expectedKind)
    {
        var target = 2;
        var navigator = Navigator(() => [new(target, 0, Vector3.UnitX)]);
        var initial = Navigate(navigator, 0, null, 1000);
        Assert.That(initial.State!.Plan.DistanceMeters, Is.EqualTo(3));
        var controller = new AiPursuitPitController();
        var armed = controller.Update(Request(revision: initial.State.Revision));
        target = targetPoint;
        var next = Navigate(navigator, policePoint, initial.State, 1100);
        Assert.Multiple(() =>
        {
            Assert.That(next.Status, Is.EqualTo(AiPursuitNavigationStatus.Active));
            Assert.That(next.UpdateKind, Is.EqualTo(expectedKind));
            Assert.That(next.State!.TargetPointId, Is.EqualTo(targetPoint));
            Assert.That(next.State.Revision, Is.EqualTo(2));
            Assert.That(next.State.Plan.DistanceMeters, Is.EqualTo(expectedDistance));
        });
        // Route anchors are discrete; physical alignment/clearance remain independently eligible.
        var started = controller.Update(Request(armed.State, next.State!.Revision, 1100) with
            { RouteValid = next.Status == AiPursuitNavigationStatus.Active });
        Assert.That(started.Diagnostics?.EventKind, Is.EqualTo(AiPursuitPitEventKind.Started));
    }

    [TestCase(1100, AiPursuitNavigationStatus.RouteTemporarilyUnavailable)]
    [TestCase(3200, AiPursuitNavigationStatus.NoRoute)]
    public void ActualNavigationFailureAbortsEvenWhileGraceRetainsOldPlan(long time,
        AiPursuitNavigationStatus expectedStatus)
    {
        var available = true;
        var navigator = Navigator(() => available ? [new(2, 0, Vector3.UnitX)] : []);
        var initial = Navigate(navigator, 0, null, 1000);
        var controller = new AiPursuitPitController();
        var armed = controller.Update(Request(revision: initial.State!.Revision));
        available = false;
        var transient = Navigate(navigator, 0, initial.State, 1100);
        var failed = Navigate(navigator, 0, transient.State, time);
        Assert.That(failed.Status, Is.EqualTo(expectedStatus));
        var result = controller.Update(Request(armed.State, failed.State?.Revision ?? 0, time) with
            { RouteValid = failed.Status == AiPursuitNavigationStatus.Active });
        Assert.That(result.Diagnostics?.Reason, Is.EqualTo(AiPursuitPitAbortReason.RouteLost));
    }

    private static AiPursuitNavigator Navigator(Func<IReadOnlyList<AiPursuitTargetCandidateSource>> targets)
    {
        var edges = Enumerable.Range(0, 8).ToDictionary(i => i,
            i => i < 7 ? new AiRouteEdge[] { new(i + 1, 1.5f, null, null) } : []);
        return new AiPursuitNavigator(new AiRoutePlanner(new AiRouteGraph(edges)),
            new AiPursuitTargetLocator((_, _) => targets()));
    }

    private static AiPursuitNavigationResult Navigate(AiPursuitNavigator navigator, int point,
        AiPursuitRouteState? previous, long time) =>
        navigator.Update(point, Vector3.Zero, Vector3.UnitX, 49, time, previous,
            new(new(100, 100), 2000));
}
