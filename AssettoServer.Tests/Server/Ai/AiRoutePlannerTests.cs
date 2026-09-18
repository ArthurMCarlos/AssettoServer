using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiRoutePlannerTests
{
    [Test]
    public void FindsDirectPathWithLiteralDistance()
    {
        var planner = CreatePlanner(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 10, null, null)],
            [1] = [new(2, 15, null, null)],
            [2] = []
        });

        var plan = planner.TryPlan(0, new HashSet<int> { 2 }, 100);

        Assert.Multiple(() =>
        {
            Assert.That(plan, Is.Not.Null);
            Assert.That(plan!.DistanceMeters, Is.EqualTo(25));
            Assert.That(plan.JunctionDecisions, Is.Empty);
        });
    }

    [Test]
    public void ChoosesBranchThatReachesTarget()
    {
        var planner = CreatePlanner(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 10, 4, false), new(2, 12, 4, true)],
            [1] = [new(3, 10, null, null)],
            [2] = [new(5, 8, null, null)],
            [3] = [],
            [5] = []
        });

        var plan = planner.TryPlan(0, new HashSet<int> { 5 }, 100);

        Assert.Multiple(() =>
        {
            Assert.That(plan, Is.Not.Null);
            Assert.That(plan!.DistanceMeters, Is.EqualTo(20));
            Assert.That(plan.JunctionDecisions,
                Is.EqualTo(new Dictionary<int, bool> { [4] = true }));
        });
    }

    [Test]
    public void RejectsPathLongerThanMaximumDistance()
    {
        var planner = CreatePlanner(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 800, null, null)],
            [1] = [new(2, 800, null, null)],
            [2] = []
        });

        var plan = planner.TryPlan(0, new HashSet<int> { 2 }, 1500);

        Assert.That(plan, Is.Null);
    }

    [Test]
    public void ReturnsNullForUnreachableTarget()
    {
        var planner = CreatePlanner(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 10, null, null)],
            [1] = [],
            [2] = []
        });

        Assert.That(planner.TryPlan(0, new HashSet<int> { 2 }, 100), Is.Null);
    }

    [Test]
    public void AcceptsAnyEquivalentTargetPoint()
    {
        var planner = CreatePlanner(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 30, null, null)],
            [1] = []
        });

        var plan = planner.TryPlan(0, new HashSet<int> { 1, 9 }, 100);

        Assert.That(plan?.DistanceMeters, Is.EqualTo(30));
    }

    [Test]
    public void RejectsInvalidStartPoint()
    {
        var planner = CreatePlanner(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = []
        });

        Assert.That(planner.TryPlan(99, new HashSet<int> { 0 }, 100), Is.Null);
    }

    [Test]
    public void BuildsGraphLazilyAndOnlyOnce()
    {
        var buildCount = 0;
        var graph = new AiRouteGraph(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = []
        });
        var planner = new AiRoutePlanner(() =>
        {
            buildCount++;
            return graph;
        });

        Assert.That(buildCount, Is.Zero);

        planner.TryPlan(0, new HashSet<int> { 0 }, 100);
        planner.TryPlan(0, new HashSet<int> { 0 }, 100);

        Assert.That(buildCount, Is.EqualTo(1));
    }

    [Test]
    public void ReturnsOrderedCorridorAcrossMultipleJunctions()
    {
        var planner = CreatePlanner(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 10, 3, true)],
            [1] = [new(2, 20, 4, false)],
            [2] = [new(5, 30, null, null)],
            [5] = []
        });

        var result = planner.TryPlan(
            0,
            new HashSet<int> { 5 },
            new AiRouteSearchLimits(20_000, 50_000));

        Assert.Multiple(() =>
        {
            Assert.That(result.Failure, Is.EqualTo(AiRouteSearchFailure.None));
            Assert.That(result.Plan!.Nodes.Select(node => node.PointId),
                Is.EqualTo(new[] { 0, 1, 2, 5 }));
            Assert.That(result.Plan.Nodes.Select(node => node.DistanceFromStartMeters),
                Is.EqualTo(new[] { 0, 10, 30, 60 }));
            Assert.That(result.Plan.DistanceMeters, Is.EqualTo(60));
            Assert.That(result.Plan.JunctionDecisions,
                Is.EqualTo(new Dictionary<int, bool> { [3] = true, [4] = false }));
        });
    }

    [Test]
    public void StopsAtVisitedNodeBudget()
    {
        var planner = CreateLinearPlanner(pointCount: 10);

        var result = planner.TryPlan(
            0,
            new HashSet<int> { 9 },
            new AiRouteSearchLimits(20_000, 3));

        Assert.Multiple(() =>
        {
            Assert.That(result.Plan, Is.Null);
            Assert.That(result.Failure, Is.EqualTo(AiRouteSearchFailure.NodeLimit));
            Assert.That(result.VisitedNodes, Is.EqualTo(3));
        });
    }

    [Test]
    public void DistinguishesDistanceLimitFromUnreachableTarget()
    {
        var distanceLimited = CreatePlanner(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 60, null, null)],
            [1] = [new(2, 60, null, null)],
            [2] = []
        });
        var disconnected = CreatePlanner(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 10, null, null)],
            [1] = [],
            [2] = []
        });

        var distanceResult = distanceLimited.TryPlan(
            0,
            new HashSet<int> { 2 },
            new AiRouteSearchLimits(100, 50));
        var unreachableResult = disconnected.TryPlan(
            0,
            new HashSet<int> { 2 },
            new AiRouteSearchLimits(100, 50));

        Assert.Multiple(() =>
        {
            Assert.That(distanceResult.Failure, Is.EqualTo(AiRouteSearchFailure.DistanceLimit));
            Assert.That(unreachableResult.Failure, Is.EqualTo(AiRouteSearchFailure.Unreachable));
        });
    }

    private static AiRoutePlanner CreatePlanner(
        IReadOnlyDictionary<int, AiRouteEdge[]> edges) =>
        new(new AiRouteGraph(edges));

    private static AiRoutePlanner CreateLinearPlanner(int pointCount)
    {
        var edges = Enumerable.Range(0, pointCount)
            .ToDictionary(
                pointId => pointId,
                pointId => pointId + 1 < pointCount
                    ? new[] { new AiRouteEdge(pointId + 1, 1, null, null) }
                    : Array.Empty<AiRouteEdge>());
        return CreatePlanner(edges);
    }
}
