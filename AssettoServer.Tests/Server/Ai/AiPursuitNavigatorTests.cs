using System.Numerics;
using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitNavigatorTests
{
    [Test]
    public void SelectsReachableAlternativeWhenNearestCandidateHasNoPath()
    {
        var sources = new[]
        {
            Source(20, 1),
            Source(30, 4)
        };
        var navigator = CreateNavigator(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(30, 25, null, null)],
            [20] = [],
            [30] = []
        }, () => sources);

        var result = navigator.Update(
            policePointId: 0,
            Vector3.Zero,
            Vector3.Zero,
            maximumTargetDistanceSquared: 49,
            nowMilliseconds: 0,
            previous: null,
            Options());

        Assert.Multiple(() =>
        {
            Assert.That(result.Status, Is.EqualTo(AiPursuitNavigationStatus.Active));
            Assert.That(result.State!.TargetPointId, Is.EqualTo(30));
            Assert.That(result.State.Plan.DistanceMeters, Is.EqualTo(25));
            Assert.That(result.UpdateKind, Is.EqualTo(AiPursuitRouteUpdateKind.Selected));
        });
    }

    [Test]
    public void PlansAcrossMultipleJunctions()
    {
        var navigator = CreateNavigator(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 10, 3, true)],
            [1] = [new(2, 20, 4, false)],
            [2] = []
        }, () => [Source(2, 1)]);

        var result = Update(navigator, policePointId: 0, nowMilliseconds: 0);

        Assert.Multiple(() =>
        {
            Assert.That(result.State!.Plan.Nodes.Select(node => node.PointId),
                Is.EqualTo(new[] { 0, 1, 2 }));
            Assert.That(result.State.Plan.JunctionDecisions,
                Is.EqualTo(new Dictionary<int, bool> { [3] = true, [4] = false }));
        });
    }

    [Test]
    public void ReusesCorridorAndRebasesDistanceAfterPoliceAdvances()
    {
        var navigator = CreateNavigator(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 10, null, null)],
            [1] = [new(2, 20, null, null)],
            [2] = []
        }, () => [Source(2, 1)]);
        var initial = Update(navigator, policePointId: 0, nowMilliseconds: 0);

        var reused = navigator.Update(
            1,
            Vector3.Zero,
            Vector3.Zero,
            49,
            200,
            initial.State,
            Options());

        Assert.Multiple(() =>
        {
            Assert.That(reused.UpdateKind, Is.EqualTo(AiPursuitRouteUpdateKind.Reused));
            Assert.That(reused.State!.Revision, Is.EqualTo(initial.State!.Revision));
            Assert.That(reused.State.Plan.Nodes.Select(node => node.PointId),
                Is.EqualTo(new[] { 1, 2 }));
            Assert.That(reused.State.Plan.Nodes.Select(node => node.DistanceFromStartMeters),
                Is.EqualTo(new[] { 0, 20 }));
            Assert.That(reused.State.Plan.DistanceMeters, Is.EqualTo(20));
        });
    }

    [Test]
    public void ExtendsCachedCorridorWhenTargetMovesForward()
    {
        AiPursuitTargetCandidateSource[] sources = [Source(1, 1)];
        var navigator = CreateNavigator(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 10, null, null)],
            [1] = [new(2, 15, 7, true)],
            [2] = []
        }, () => sources);
        var initial = Update(navigator, policePointId: 0, nowMilliseconds: 0);
        sources = [Source(2, 1)];

        var extended = navigator.Update(
            0,
            Vector3.Zero,
            Vector3.Zero,
            49,
            200,
            initial.State,
            Options());

        Assert.Multiple(() =>
        {
            Assert.That(extended.UpdateKind, Is.EqualTo(AiPursuitRouteUpdateKind.Extended));
            Assert.That(extended.State!.TargetPointId, Is.EqualTo(2));
            Assert.That(extended.State.Plan.Nodes.Select(node => node.PointId),
                Is.EqualTo(new[] { 0, 1, 2 }));
            Assert.That(extended.State.Plan.DistanceMeters, Is.EqualTo(25));
            Assert.That(extended.State.Plan.JunctionDecisions,
                Is.EqualTo(new Dictionary<int, bool> { [7] = true }));
        });
    }

    [Test]
    public void KeepsLastRouteDuringGraceThenReturnsDefinitiveNoRoute()
    {
        AiPursuitTargetCandidateSource[] sources = [Source(1, 1)];
        var navigator = CreateNavigator(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 10, null, null)],
            [1] = [],
            [9] = []
        }, () => sources);
        var active = Update(navigator, policePointId: 0, nowMilliseconds: 0);
        sources = [Source(9, 1)];

        var temporary = navigator.Update(
            0, Vector3.Zero, Vector3.Zero, 49, 1000, active.State, Options());
        var definitive = navigator.Update(
            0, Vector3.Zero, Vector3.Zero, 49, 3001, temporary.State, Options());

        Assert.Multiple(() =>
        {
            Assert.That(temporary.Status,
                Is.EqualTo(AiPursuitNavigationStatus.RouteTemporarilyUnavailable));
            Assert.That(temporary.State!.Plan, Is.SameAs(active.State!.Plan));
            Assert.That(temporary.TargetLocationDiagnostics.SpatialPointIds,
                Is.EqualTo(new[] { 9 }));
            Assert.That(temporary.MaximumExploredDistanceMeters, Is.EqualTo(10));
            Assert.That(temporary.JunctionEdgesExamined, Is.Zero);
            Assert.That(definitive.Status, Is.EqualTo(AiPursuitNavigationStatus.NoRoute));
            Assert.That(definitive.State, Is.Null);
        });
    }

    [Test]
    public void RecoversRouteWithinGraceAndAdvancesRevision()
    {
        AiPursuitTargetCandidateSource[] sources = [Source(1, 1)];
        var navigator = CreateNavigator(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 10, null, null)],
            [1] = [],
            [9] = []
        }, () => sources);
        var active = Update(navigator, policePointId: 0, nowMilliseconds: 0);
        sources = [Source(9, 1)];
        var temporary = navigator.Update(
            0, Vector3.Zero, Vector3.Zero, 49, 1000, active.State, Options());
        sources = [Source(1, 1)];

        var recovered = navigator.Update(
            0, Vector3.Zero, Vector3.Zero, 49, 1500, temporary.State, Options());

        Assert.Multiple(() =>
        {
            Assert.That(recovered.Status, Is.EqualTo(AiPursuitNavigationStatus.Active));
            Assert.That(recovered.UpdateKind, Is.EqualTo(AiPursuitRouteUpdateKind.Recovered));
            Assert.That(recovered.State!.Revision, Is.EqualTo(active.State!.Revision + 1));
            Assert.That(recovered.State.FirstFailureMilliseconds, Is.Null);
        });
    }

    [Test]
    public void InitialNodeLimitIsDefinitiveWithoutCachedRoute()
    {
        var navigator = CreateNavigator(new Dictionary<int, AiRouteEdge[]>
        {
            [0] = [new(1, 1, null, null)],
            [1] = [new(2, 1, null, null)],
            [2] = [new(3, 1, null, null)],
            [3] = []
        }, () => [Source(3, 1)]);

        var result = navigator.Update(
            0,
            Vector3.Zero,
            Vector3.Zero,
            49,
            0,
            null,
            Options(maxVisitedNodes: 2));

        Assert.Multiple(() =>
        {
            Assert.That(result.Status, Is.EqualTo(AiPursuitNavigationStatus.NoRoute));
            Assert.That(result.SearchFailure, Is.EqualTo(AiRouteSearchFailure.NodeLimit));
            Assert.That(result.VisitedNodes, Is.EqualTo(2));
        });
    }

    private static AiPursuitNavigator CreateNavigator(
        IReadOnlyDictionary<int, AiRouteEdge[]> edges,
        Func<IReadOnlyList<AiPursuitTargetCandidateSource>> getSources)
    {
        var planner = new AiRoutePlanner(new AiRouteGraph(edges));
        var locator = new AiPursuitTargetLocator((_, _) => getSources());
        return new AiPursuitNavigator(planner, locator);
    }

    private static AiPursuitNavigationResult Update(
        AiPursuitNavigator navigator,
        int policePointId,
        long nowMilliseconds) =>
        navigator.Update(
            policePointId,
            Vector3.Zero,
            Vector3.Zero,
            maximumTargetDistanceSquared: 49,
            nowMilliseconds,
            previous: null,
            Options());

    private static AiPursuitNavigationOptions Options(
        int graceMilliseconds = 2000,
        int maxVisitedNodes = 50_000) =>
        new(
            new AiRouteSearchLimits(20_000, maxVisitedNodes),
            graceMilliseconds,
            MaximumTargetCandidates: 16,
            ExtensionDistanceMeters: 500,
            ExtensionMaxVisitedNodes: 5000);

    private static AiPursuitTargetCandidateSource Source(int pointId, float distanceSquared) =>
        new(pointId, distanceSquared, Vector3.UnitX);
}
