using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitTrafficReturnTests
{
    [Test]
    public void ReachableDownstreamMergeDoesNotMeanAlreadyInTargetsLane()
    {
        var selector = new AiPursuitLaneSelector(
            p => p == 0 ? new(10, -1) : new(-1, -1),
            (_, _) => true,
            (_, _, _) => AiPursuitLanePhysicalRelation.ImmediateLeft,
            (start, _, _) => new(new(start == 0 ? 100 : 2000,
                [new(start, 0), new(30, start == 0 ? 100 : 2000)], new Dictionary<int, bool>()), AiRouteSearchFailure.None, 2),
            _ => null,
            p => p switch { 0 => 1, 1 => -1, 10 => 11, 11 => 30, _ => -1 },
            _ => 1000);
        var limits = new AiRouteSearchLimits(20000, 50000);
        var alignment = selector.SelectForAlignment(0, new HashSet<int> { 30 }, limits, 60, 1000);
        Assert.That(alignment.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Stay));
        Assert.That(selector.IsOnTargetPhysicalLane(0, 30, limits), Is.False);
        Assert.That(selector.IsOnTargetPhysicalLane(10, 30, limits), Is.True);
        var returning = selector.SelectForTrafficReturn(0, 30, limits);
        Assert.That(returning!.ToPointId, Is.EqualTo(10));
        Assert.That(returning.Motivation, Is.EqualTo(AiPursuitLaneMotivation.TrafficReturn));
    }
}
