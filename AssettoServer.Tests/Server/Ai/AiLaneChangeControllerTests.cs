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

    private static AiPursuitLaneSelection Selection() =>
        new(0, 10, AiLaneChangeDirection.Left,
            new AiRoutePlan(120, [new AiRouteNode(10, 0), new AiRouteNode(99, 120)],
                new Dictionary<int, bool> { [7] = true }), 120);

    private static AiSplineCursor Cursor(int pointId, float z) =>
        new(pointId, 0, _ => null, _ => 100,
            (_, progress) => new AiSplinePose(new Vector3(progress, 0, z), Vector3.UnitX));
}
