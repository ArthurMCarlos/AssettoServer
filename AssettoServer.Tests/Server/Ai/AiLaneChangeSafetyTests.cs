using System.Numerics;
using AssettoServer.Server.Ai;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiLaneChangeSafetyTests
{
    [Test]
    public void ClearDestinationLaneIsSafe()
    {
        Assert.That(AiLaneChangeSafety.Evaluate(Request()).Status,
            Is.EqualTo(AiLaneChangeSafetyStatus.Safe));
    }

    [TestCase(8, 0, 10, AiLaneChangeSafetyStatus.BlockedFront)]
    [TestCase(0, 0, 10, AiLaneChangeSafetyStatus.BlockedSide)]
    [TestCase(-8, 0, 10, AiLaneChangeSafetyStatus.BlockedRear)]
    [TestCase(-18, 0, 40, AiLaneChangeSafetyStatus.BlockedRearClosing)]
    [TestCase(0, 5, 10, AiLaneChangeSafetyStatus.Safe)]
    public void ClassifiesObstacleByDestinationCorridor(
        float longitudinal,
        float lateral,
        float speed,
        AiLaneChangeSafetyStatus expected)
    {
        var obstacle = new AiLaneChangeObstacle(
            new Vector3(longitudinal, 0, lateral),
            Vector3.UnitX * speed,
            4);

        Assert.That(AiLaneChangeSafety.Evaluate(Request(obstacle)).Status,
            Is.EqualTo(expected));
    }

    [Test]
    public void BlocksSlowVehicleAheadUsingClosingSpeedGap()
    {
        var obstacle = new AiLaneChangeObstacle(
            new Vector3(40, 0, 0),
            Vector3.Zero,
            4);

        Assert.That(AiLaneChangeSafety.Evaluate(Request(
                policeSpeed: 50,
                obstacle)).Status,
            Is.EqualTo(AiLaneChangeSafetyStatus.BlockedFront));
    }

    private static AiLaneChangeSafetyRequest Request(
        params AiLaneChangeObstacle[] obstacles) =>
        Request(20, obstacles);

    private static AiLaneChangeSafetyRequest Request(
        float policeSpeed,
        params AiLaneChangeObstacle[] obstacles) =>
        new(
            Vector3.Zero,
            Vector3.UnitX,
            policeSpeed,
            4,
            4,
            obstacles);
}
