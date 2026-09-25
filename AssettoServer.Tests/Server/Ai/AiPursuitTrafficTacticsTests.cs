using AssettoServer.Server.Ai;
using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitTrafficTacticsTests
{
    static AiPursuitLaneSelection Lane(int id = 10) => new(0, id, AiLaneChangeDirection.Left,
        new AiRoutePlan(100, [new(0, 0), new(id, 100)], new Dictionary<int, bool>()), null)
        { PhysicalRelation = AiPursuitLanePhysicalRelation.ImmediateLeft };
    static AiTrafficTacticRequest Request => new(0, true, 256, 10 / 3.6f, 30, false, false,
        false, false, false, [new(Lane(), 150, true)], null, new(AiTrafficTacticPhase.Follow, null, null, null));
    static AiTrafficTacticDecision Update(AiTrafficTacticRequest request) =>
        AiPursuitTrafficTactics.Update(AiPursuitCloseOptionsTests.Defaults, request);

    [Test]
    public void RequiresContinuousBlockAndSameObstacleForOneSecond()
    {
        var start = Update(Request);
        Assert.That(Update(Request with { NowMilliseconds = 999, Previous = start.State }).Selection, Is.Null);
        var go = Update(Request with { NowMilliseconds = 1000, Previous = start.State });
        Assert.That(go.Selection!.Motivation, Is.EqualTo(AiPursuitLaneMotivation.TrafficBypass));
        Assert.That(go.State.Phase, Is.EqualTo(AiTrafficTacticPhase.Bypass));
        Assert.That(Update(Request with { NowMilliseconds = 1000, Previous = start.State, ObstacleIdentity = 512 }).Selection, Is.Null);
    }

    [Test]
    public void UnsafeOrUnhelpfulLanesNeverBypass()
    {
        var previous = Update(Request).State;
        foreach (var candidate in new[] { new AiTrafficLaneOption(Lane(), 150, false), new(Lane(), 20, true) })
            Assert.That(Update(Request with { NowMilliseconds = 1000, Previous = previous, Candidates = [candidate] }).Selection, Is.Null);
    }

    [Test]
    public void BypassWaitsUntilPassedAndReturnsToCurrentTargetLane()
    {
        var state = Update(Request with { NowMilliseconds = 1000, Previous = Update(Request).State }).State;
        Assert.That(Update(Request with { Previous = state, ReturnSelection = Lane(30) }).Selection, Is.Null);
        var returning = Update(Request with { Previous = state, ObstaclePassedWithClearance = true, ReturnSelection = Lane(30) });
        Assert.That(returning.Selection!.ToPointId, Is.EqualTo(30));
        Assert.That(returning.Selection.Motivation, Is.EqualTo(AiPursuitLaneMotivation.TrafficReturn));
        Assert.That(Update(Request with { Previous = returning.State, TargetLaneReached = true }).State.Phase,
            Is.EqualTo(AiTrafficTacticPhase.Follow));
    }

    [Test]
    public void CommittedTransitionPitAndJunctionSuppressOptionalTactic()
    {
        var ready = Request with { NowMilliseconds = 1000, Previous = Update(Request).State };
        Assert.That(Update(ready with { PitBusy = true }).Selection, Is.Null);
        Assert.That(Update(ready with { JunctionHasPriority = true }).Selection, Is.Null);
        Assert.That(Update(ready with { TransitionCommitted = true }).Selection, Is.Null);
    }
}
