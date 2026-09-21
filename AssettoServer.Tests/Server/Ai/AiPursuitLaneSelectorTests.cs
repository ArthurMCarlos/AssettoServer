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

        var result = selector.Select(0, Targets, Limits, 60, 1000);

        Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Stay));
        Assert.That(result.Selection, Is.Null);
        Assert.That(result.Reason, Is.EqualTo(AiPursuitLaneEvaluationReason.CurrentLaneValid));
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

        var result = selector.Select(0, Targets, Limits, 60, 1000);

        Assert.Multiple(() =>
        {
            Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Change));
            Assert.That(result.Selection!.FromPointId, Is.EqualTo(0));
            Assert.That(result.Selection.ToPointId, Is.EqualTo(10));
            Assert.That(result.Selection.Direction, Is.EqualTo(AiLaneChangeDirection.Left));
            Assert.That(result.Selection.DistanceToDecisionMeters, Is.EqualTo(120));
            Assert.That(result.Selection.JunctionId, Is.EqualTo(7));
            Assert.That(result.Reason, Is.EqualTo(AiPursuitLaneEvaluationReason.RoutePreparation));
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

        var result = selector.Select(0, Targets, Limits, 60, 1000);

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

        var result = selector.Select(0, Targets, Limits, 60, 1000);

        Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Unreachable));
        Assert.That(requestedStarts, Is.EqualTo(new[] { 0, 10, 20 }));
    }

    [Test]
    public void RejectsAdjacentRouteWithoutRealJunction()
    {
        var selector = CreateSelector(new Dictionary<int, AiRoutePlan?>
        {
            [0] = null,
            [10] = Plan(10, 99, 120),
            [20] = null
        });

        var result = selector.Select(0, Targets, Limits, 60, 1000);

        Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Unreachable));
        Assert.That(result.Candidates.Single(candidate => candidate.PointId == 10).Rejection,
            Is.EqualTo(AiPursuitLaneRejectionReason.NoRealJunction));
    }

    [Test]
    public void SelectsDirectTargetLaneWithoutJunctionForAlignment()
    {
        var selector = CreateSelector(new Dictionary<int, AiRoutePlan?>
        {
            [0] = null,
            [10] = Plan(10, 99, 94.6f),
            [20] = null
        });

        var result = selector.SelectForAlignment(0, Targets, Limits, 60, 1000);

        Assert.Multiple(() =>
        {
            Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Change));
            Assert.That(result.Selection!.ToPointId, Is.EqualTo(10));
            Assert.That(result.Selection.JunctionId, Is.Null);
            Assert.That(result.Selection.DistanceToDecisionMeters, Is.EqualTo(94.6f));
            Assert.That(result.Selection.Motivation,
                Is.EqualTo(AiPursuitLaneMotivation.TargetLaneAlignment));
        });
    }

    [Test]
    public void RoutePreparationStillRejectsDirectPlanWithoutJunction()
    {
        var selector = CreateSelector(new Dictionary<int, AiRoutePlan?>
        {
            [0] = null,
            [10] = Plan(10, 99, 94.6f),
            [20] = null
        });

        var result = selector.SelectForRoutePreparation(0, Targets, Limits, 60, 1000);

        Assert.That(result.Reason, Is.EqualTo(AiPursuitLaneEvaluationReason.NoRealJunction));
    }

    [Test]
    public void AlignmentChoosesShorterAdjacentRouteWhenCurrentRouteIsValid()
    {
        var selector = CreateSelector(new Dictionary<int, AiRoutePlan?>
        {
            [0] = Plan(0, 99, 850),
            [10] = Plan(10, 99, 94.6f),
            [20] = null
        });

        var result = selector.SelectForAlignment(0, Targets, Limits, 60, 1000);

        Assert.That(result.Selection!.ToPointId, Is.EqualTo(10));
    }

    [Test]
    public void AlignmentStaysWhenCurrentRouteIsNotLongerThanAdjacentRoute()
    {
        var selector = CreateSelector(new Dictionary<int, AiRoutePlan?>
        {
            [0] = Plan(0, 99, 90),
            [10] = Plan(10, 99, 94.6f),
            [20] = null
        });

        var result = selector.SelectForAlignment(0, Targets, Limits, 60, 1000);

        Assert.Multiple(() =>
        {
            Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Stay));
            Assert.That(result.Selection, Is.Null);
            Assert.That(result.Reason, Is.EqualTo(AiPursuitLaneEvaluationReason.CurrentLaneValid));
        });
    }

    [Test]
    public void RejectsCandidateWhoseOppositeLinkDoesNotReturnToCurrentPoint()
    {
        var selector = new AiPursuitLaneSelector(
            _ => new AiAdjacentLanePoints(10, -1),
            (_, _) => true,
            (_, _, _) => AiPursuitLanePhysicalRelation.NonAdjacent,
            (start, _, _) => Search(start == 10 ? Plan(10, 99, 120) : null),
            _ => null);

        var result = selector.SelectForAlignment(0, Targets, Limits, 60, 1000);

        Assert.Multiple(() =>
        {
            Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Unreachable));
            Assert.That(result.CandidateLaneRoutes.Single(candidate => candidate.PointId == 10).Reason,
                Is.EqualTo(AiPursuitLaneEvaluationReason.NonAdjacent));
        });
    }

    [Test]
    public void RoutePreparationRejectsNonReciprocalNeighborEvenWithJunctionRoute()
    {
        var selector = new AiPursuitLaneSelector(
            _ => new AiAdjacentLanePoints(10, -1),
            (_, _) => true,
            (_, _, _) => AiPursuitLanePhysicalRelation.NonAdjacent,
            (start, _, _) => Search(start == 10 ? Plan(10, 99, 120, junctionId: 7) : null),
            plan => new AiPursuitLaneDecision(7, plan.DistanceMeters));

        var result = selector.SelectForRoutePreparation(0, Targets, Limits, 60, 1000);

        Assert.Multiple(() =>
        {
            Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Unreachable));
            Assert.That(result.Candidates.Single(candidate => candidate.PointId == 10).Rejection,
                Is.EqualTo(AiPursuitLaneRejectionReason.NonAdjacent));
        });
    }

    [Test]
    public void ReportsImmediatePhysicalRelationForReciprocalNeighbor()
    {
        var selector = new AiPursuitLaneSelector(
            _ => new AiAdjacentLanePoints(10, -1),
            (_, _) => true,
            (_, _, _) => AiPursuitLanePhysicalRelation.ImmediateLeft,
            (start, _, _) => Search(start == 10 ? Plan(10, 99, 120) : null),
            _ => null);

        var result = selector.SelectForAlignment(0, Targets, Limits, 60, 1000);

        Assert.That(result.Selection!.PhysicalRelation,
            Is.EqualTo(AiPursuitLanePhysicalRelation.ImmediateLeft));
    }

    [Test]
    public void RejectsCandidateWithInvalidPhysicalGeometryBeforePlanningIt()
    {
        var requestedStarts = new List<int>();
        var selector = new AiPursuitLaneSelector(
            _ => new AiAdjacentLanePoints(10, -1),
            (_, _) => true,
            (_, _, _) => AiPursuitLanePhysicalRelation.InvalidGeometry,
            (start, _, _) =>
            {
                requestedStarts.Add(start);
                return Search(start == 10 ? Plan(10, 99, 120) : null);
            },
            _ => null);

        var result = selector.SelectForAlignment(0, Targets, Limits, 60, 1000);

        Assert.Multiple(() =>
        {
            Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Unreachable));
            Assert.That(result.CandidateLaneRoutes.Single(candidate => candidate.PointId == 10).Reason,
                Is.EqualTo(AiPursuitLaneEvaluationReason.InvalidGeometry));
            Assert.That(requestedStarts, Is.EqualTo(new[] { 0 }));
        });
    }

    [Test]
    public void RejectsJunctionBeyondPreparationLookahead()
    {
        var selector = CreateSelector(new Dictionary<int, AiRoutePlan?>
        {
            [0] = null,
            [10] = Plan(10, 99, 1001, junctionId: 7),
            [20] = null
        });

        var result = selector.Select(0, Targets, Limits, 60, 1000);

        Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Unreachable));
        Assert.That(result.Candidates.Single(candidate => candidate.PointId == 10).Rejection,
            Is.EqualTo(AiPursuitLaneRejectionReason.BeyondLookahead));
    }

    [Test]
    public void ReportsRouteEvidenceForUnreachableCurrentAndAdjacentLanes()
    {
        var selector = new AiPursuitLaneSelector(
            _ => new AiAdjacentLanePoints(10, 20),
            (_, _) => true,
            (start, _, _) => new AiRouteSearchResult(
                null,
                start == 0
                    ? AiRouteSearchFailure.DistanceLimit
                    : AiRouteSearchFailure.Unreachable,
                3,
                start == 0 ? 999 : 40,
                start == 0 ? 12 : 0),
            _ => null);

        var result = selector.Select(0, Targets, Limits, 60, 1000);

        Assert.Multiple(() =>
        {
            Assert.That(result.CurrentLaneRoute.SearchFailure,
                Is.EqualTo(AiRouteSearchFailure.DistanceLimit));
            Assert.That(result.CurrentLaneRoute.MaximumExploredDistanceMeters, Is.EqualTo(999));
            Assert.That(result.CurrentLaneRoute.JunctionEdgesExamined, Is.EqualTo(12));
            Assert.That(result.CandidateLaneRoutes, Has.Count.EqualTo(2));
        });
    }

    private static AiPursuitLaneSelector CreateSelector(
        IReadOnlyDictionary<int, AiRoutePlan?> plans) =>
        new(
            _ => new AiAdjacentLanePoints(10, 20),
            (_, _) => true,
            (start, _, _) => Search(plans.GetValueOrDefault(start)),
            plan => plan.JunctionDecisions.Count == 0
                ? null
                : new AiPursuitLaneDecision(
                    plan.JunctionDecisions.Keys.Single(),
                    plan.DistanceMeters));

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
