using System.Numerics;
using AssettoServer.Server.Ai;
using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiLaneChangeControllerTests
{
    [Test]
    public void WaitsForSafeGapThenCompletesOnDestinationCursor()
    {
        var controller = new AiLaneChangeController(60, 3000);
        controller.Request(Selection(), Cursor(0, 0), Cursor(10, 3), 0, 4);

        controller.UpdateWaiting(AiLaneChangeSafetyStatus.BlockedSide, 100);
        Assert.That(controller.Phase, Is.EqualTo(AiLaneChangePhase.WaitingForGap));

        controller.UpdateWaiting(AiLaneChangeSafetyStatus.Safe, 200);
        Assert.That(controller.Phase, Is.EqualTo(AiLaneChangePhase.Changing));

        Assert.That(controller.TryMove(60, 200, out var movement), Is.True);
        Assert.Multiple(() =>
        {
            Assert.That(movement.Completed, Is.True);
            Assert.That(movement.DestinationPointId, Is.EqualTo(10));
            Assert.That(movement.Pose.Position.Z, Is.EqualTo(3).Within(0.001));
            Assert.That(controller.Phase, Is.EqualTo(AiLaneChangePhase.Cooldown));
            Assert.That(controller.Event!.Kind, Is.EqualTo(AiPursuitLaneChangeEventKind.Completed));
        });
    }

    [Test]
    public void RouteRevisionCancelsWaitingButNotCommittedChange()
    {
        var waiting = new AiLaneChangeController(60, 3000);
        waiting.Request(Selection(), Cursor(0, 0), Cursor(10, 3), 0, 4);
        Assert.That(waiting.CancelWaiting(5), Is.True);

        var changing = new AiLaneChangeController(60, 3000);
        changing.Request(Selection(), Cursor(0, 0), Cursor(10, 3), 0, 4);
        changing.UpdateWaiting(AiLaneChangeSafetyStatus.Safe, 10);

        Assert.Multiple(() =>
        {
            Assert.That(changing.CancelWaiting(5), Is.False);
            Assert.That(changing.Phase, Is.EqualTo(AiLaneChangePhase.Changing));
        });
    }

    [Test]
    public void WaitingRequestCanBeRebasedToCurrentAdjacentPoints()
    {
        var controller = new AiLaneChangeController(60, 3000);
        controller.Request(Selection(), Cursor(0, 0), Cursor(10, 3), 0, 4);
        var revised = Selection() with { FromPointId = 1, ToPointId = 11 };

        controller.RefreshWaiting(revised, Cursor(1, 0), Cursor(11, 4), 5);
        controller.UpdateWaiting(AiLaneChangeSafetyStatus.Safe, 10);
        controller.TryMove(60, 10, out var movement);

        Assert.Multiple(() =>
        {
            Assert.That(movement.DestinationPointId, Is.EqualTo(11));
            Assert.That(movement.Pose.Position.Z, Is.EqualTo(4).Within(0.001));
        });
    }

    [Test]
    public void ExposesDestinationPoseForSafetyProjectionWhileWaiting()
    {
        var controller = new AiLaneChangeController(60, 3000);
        controller.Request(Selection(), Cursor(0, 0), Cursor(10, 3), 0, 4);

        Assert.That(controller.TryGetDestinationPose(out var pose), Is.True);
        Assert.That(pose.Position, Is.EqualTo(new Vector3(0, 0, 3)));
    }

    [Test]
    public void PreservesRequiredBeforeImmediateStartedEvent()
    {
        var controller = new AiLaneChangeController(60, 3000);
        controller.Request(Selection(), Cursor(0, 0), Cursor(10, 3), 0, 4);
        controller.UpdateWaiting(AiLaneChangeSafetyStatus.Safe, 0);

        Assert.Multiple(() =>
        {
            Assert.That(controller.ConsumeEvent()!.Kind,
                Is.EqualTo(AiPursuitLaneChangeEventKind.Required));
            Assert.That(controller.ConsumeEvent()!.Kind,
                Is.EqualTo(AiPursuitLaneChangeEventKind.Started));
            Assert.That(controller.ConsumeEvent(), Is.Null);
        });
    }

    [Test]
    public void ConcurrentResetSafetyAndMovementDoNotExposePartialState()
    {
        var controller = new AiLaneChangeController(60, 0);
        var failures = new System.Collections.Concurrent.ConcurrentQueue<Exception>();

        Parallel.Invoke(
            () => Repeat(() =>
            {
                controller.Request(Selection(), Cursor(0, 0), Cursor(10, 3), 0, 4);
                controller.Reset();
            }, failures),
            () => Repeat(() =>
            {
                controller.UpdateWaiting(AiLaneChangeSafetyStatus.Safe, 0);
                controller.TryGetDestinationPose(out _);
            }, failures),
            () => Repeat(() =>
            {
                controller.TryMove(1, 0, out _);
                controller.ConsumeEvent();
            }, failures));

        Assert.That(failures, Is.Empty);
    }

    [Test]
    public void ReconcileAtomicallyCancelsWaitingButRetainsCommittedChange()
    {
        var waiting = new AiLaneChangeController(60, 3000);
        waiting.Request(Selection(), Cursor(0, 0), Cursor(10, 3), 0, 4);
        var cancelled = waiting.ReconcileCurrentRoute(5);

        var changing = new AiLaneChangeController(60, 3000);
        changing.Request(Selection(), Cursor(0, 0), Cursor(10, 3), 0, 4);
        changing.UpdateWaiting(AiLaneChangeSafetyStatus.Safe, 0);
        var retained = changing.ReconcileCurrentRoute(5);

        Assert.Multiple(() =>
        {
            Assert.That(cancelled, Is.EqualTo(AiLaneChangeReconcileResult.Cancelled));
            Assert.That(waiting.Phase, Is.EqualTo(AiLaneChangePhase.None));
            Assert.That(retained, Is.EqualTo(AiLaneChangeReconcileResult.Changing));
            Assert.That(changing.Phase, Is.EqualTo(AiLaneChangePhase.Changing));
        });
    }

    [Test]
    public void FinalTickIsClampedToRemainingTransitionDistance()
    {
        var controller = new AiLaneChangeController(60, 3000);
        controller.Request(Selection(), Cursor(0, 0), Cursor(10, 3), 0, 4);
        controller.UpdateWaiting(AiLaneChangeSafetyStatus.Safe, 0);
        controller.TryMove(59.7f, 0, out _);

        Assert.That(controller.TryMove(0.8f, 1, out var movement), Is.True);
        Assert.That(movement.Completed, Is.True);
        Assert.That(movement.DestinationProgressMeters, Is.EqualTo(60).Within(0.001));
    }

    [Test]
    public void PhysicalRebaseDoesNotPublishRouteRevision()
    {
        var controller = new AiLaneChangeController(60, 3000);
        controller.Request(Selection(), Cursor(0, 0), Cursor(10, 3), 0, 4);
        controller.ConsumeEvent();
        var rebased = Selection() with { FromPointId = 1, ToPointId = 11 };

        controller.RefreshWaiting(rebased, Cursor(1, 0), Cursor(11, 3), 5);

        Assert.That(controller.ConsumeEvent(), Is.Null);
    }

    private static void Repeat(
        Action action,
        System.Collections.Concurrent.ConcurrentQueue<Exception> failures)
    {
        for (var i = 0; i < 1000; i++)
        {
            try
            {
                action();
            }
            catch (Exception exception)
            {
                failures.Enqueue(exception);
            }
        }
    }

    private static AiPursuitLaneSelection Selection() =>
        new(0, 10, AiLaneChangeDirection.Left,
            new AiRoutePlan(120, [new AiRouteNode(10, 0), new AiRouteNode(99, 120)],
                new Dictionary<int, bool> { [7] = true }), 120);

    private static AiSplineCursor Cursor(int pointId, float z) =>
        new(pointId, 0, _ => null, _ => 100,
            (_, progress) => new AiSplinePose(new Vector3(progress, 0, z), Vector3.UnitX));
}
