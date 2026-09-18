using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitLaneSelectorTests
{
    private static readonly IReadOnlySet<int> Targets = new HashSet<int> { 99 };
    private static readonly AiRouteSearchLimits Limits = new(1000, 100);

    [Test]
    public void KeepsCurrentLaneWhenItStillReachesTarget()
    {
        var selector = CreateSelector(new Dictionary<int, AiRoutePlan?>
        {
            [0] = Plan(0, 99, 100),
            [10] = Plan(10, 99, 80)
        });

        var result = selector.Select(0, Targets, Limits, 60);

        Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Stay));
        Assert.That(result.Selection, Is.Null);
    }

    [Test]
    public void SelectsImmediateLeftLaneWhenItIsOnlyReachableRoute()
    {
        var selector = CreateSelector(new Dictionary<int, AiRoutePlan?>
        {
            [0] = null,
            [10] = Plan(10, 99, 120, junctionId: 7),
            [20] = null
        });

        var result = selector.Select(0, Targets, Limits, 60);

        Assert.Multiple(() =>
        {
            Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Change));
            Assert.That(result.Selection!.FromPointId, Is.EqualTo(0));
            Assert.That(result.Selection.ToPointId, Is.EqualTo(10));
            Assert.That(result.Selection.Direction, Is.EqualTo(AiLaneChangeDirection.Left));
            Assert.That(result.Selection.DistanceToDecisionMeters, Is.EqualTo(120));
        });
    }

    [Test]
    public void RejectsReachableLaneWhenDecisionIsTooClose()
    {
        var selector = CreateSelector(new Dictionary<int, AiRoutePlan?>
        {
            [0] = null,
            [10] = Plan(10, 99, 40, junctionId: 7),
            [20] = null
        });

        var result = selector.Select(0, Targets, Limits, 60);

        Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Unreachable));
        Assert.That(result.Candidates.Single(candidate => candidate.PointId == 10).Rejection,
            Is.EqualTo(AiPursuitLaneRejectionReason.InsufficientPreparationDistance));
    }

    [Test]
    public void DoesNotEvaluateLaneBeyondImmediateNeighbor()
    {
        var requestedStarts = new List<int>();
        var selector = new AiPursuitLaneSelector(
            pointId => pointId == 0 ? new AiAdjacentLanePoints(10, 20) : new AiAdjacentLanePoints(30, -1),
            (_, _) => true,
            (start, _, _) =>
            {
                requestedStarts.Add(start);
                return Search(start == 30 ? Plan(30, 99, 120, 7) : null);
            });

        var result = selector.Select(0, Targets, Limits, 60);

        Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Unreachable));
        Assert.That(requestedStarts, Is.EqualTo(new[] { 0, 10, 20 }));
    }

    private static AiPursuitLaneSelector CreateSelector(
        IReadOnlyDictionary<int, AiRoutePlan?> plans) =>
        new(
            _ => new AiAdjacentLanePoints(10, 20),
            (_, _) => true,
            (start, _, _) => Search(plans.GetValueOrDefault(start)));

    private static AiRouteSearchResult Search(AiRoutePlan? plan) =>
        new(plan, plan == null ? AiRouteSearchFailure.Unreachable : AiRouteSearchFailure.None, 1);

    private static AiRoutePlan Plan(
        int start,
        int target,
        float distance,
        int? junctionId = null) =>
        new(
            distance,
            [new AiRouteNode(start, 0), new AiRouteNode(target, distance)],
            junctionId.HasValue
                ? new Dictionary<int, bool> { [junctionId.Value] = true }
                : new Dictionary<int, bool>());
}
