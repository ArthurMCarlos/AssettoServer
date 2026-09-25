using System.Numerics;
using AssettoServer.Server.Ai;
using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitLaneChangePipelineTests
{
    [Test]
    public void NativeGapOpeningCommitsPendingBypassEvenAfterMovementCompletes()
    {
        var controller = new AiLaneChangeController(60, 3000);
        var pipeline = CreatePipeline(controller, currentReachesPhysicalTarget: true);
        var prepared = pipeline.Evaluate(0, 0, 100, ActiveFallback(), null,
            new(true, 60, 3000), new(20000, 50000), 100,
            alignment => alignment.Selection! with { Motivation = AiPursuitLaneMotivation.TrafficBypass }, true);
        var pending = new AiTrafficTacticCommit();
        pending.Stage(new(new(AiTrafficTacticPhase.Bypass, 0, 512, 10), prepared.Evaluation.Selection), controller.Event!);
        controller.UpdateWaiting(AiLaneChangeSafetyStatus.BlockedSide, 100);
        Assert.That(pending.Observe(controller.Event), Is.Null);
        // A native obstacle poll opens the gap, not a new tracking/preparation call.
        controller.UpdateWaiting(AiLaneChangeSafetyStatus.Safe, 200);
        Assert.That(controller.TryMove(60, 300, out var movement), Is.True);
        Assert.That(movement.Completed, Is.True);
        var committed = pending.Observe(controller.Event);
        Assert.That(committed!.State.Phase, Is.EqualTo(AiTrafficTacticPhase.Bypass));
        Assert.That(committed.State.ObstacleIdentity, Is.EqualTo(512));
        Assert.That(pending.Observe(controller.Event), Is.Null);
    }

    [Test]
    public void TrafficBypassUsesRealCursorsAndPreservesCommittedTransition()
    {
        var controller = new AiLaneChangeController(60, 3000);
        var pipeline = CreatePipeline(controller, currentReachesPhysicalTarget: true);
        var result = pipeline.Evaluate(0, 0, 100, ActiveFallback(), null,
            new(true, 60, 3000), new(20000, 50000), 100,
            alignment => alignment.Selection! with { Motivation = AiPursuitLaneMotivation.TrafficBypass }, true);
        Assert.That(result.RequestPrepared, Is.True);
        Assert.That(result.Evaluation.Selection!.Motivation, Is.EqualTo(AiPursuitLaneMotivation.TrafficBypass));
        controller.UpdateWaiting(AiLaneChangeSafetyStatus.Safe, 100);
        Assert.That(controller.TryMove(20, 200, out var movement), Is.True);
        Assert.That(movement.Completed, Is.False);
        bool called = false;
        pipeline.Evaluate(0, 20, 100, ActiveFallback(), result.EffectiveNavigation.State,
            new(true, 60, 3000), new(20000, 50000), 200, _ => { called = true; return null; }, true);
        Assert.That(called, Is.False, "Committed physical movement must not be replaced");
    }

    [Test]
    public void TacticalHoldSuppressesImmediateTargetAlignment()
    {
        var controller = new AiLaneChangeController(60, 3000);
        var pipeline = CreatePipeline(controller, currentReachesPhysicalTarget: true);
        var result = pipeline.Evaluate(0, 0, 100, ActiveFallback(), null,
            new(true, 60, 3000), new(20000, 50000), 100, _ => null, true);
        Assert.That(result.RequestPrepared, Is.False);
        Assert.That(controller.Phase, Is.EqualTo(AiLaneChangePhase.None));
    }

    [Test]
    public void ActiveFallbackStillPreparesRequestForPhysicalAnchor()
    {
        var controller = new AiLaneChangeController(60, 3000);
        var pipeline = CreatePipeline(controller);
        var fallback = ActiveFallback();

        var result = pipeline.Evaluate(
            currentPointId: 0,
            currentProgressMeters: 0,
            currentSegmentLengthMeters: 100,
            fallback,
            previousNavigation: null,
            new AiPursuitLaneChangeOptions(true, 60, 3000, 1000),
            new AiRouteSearchLimits(20_000, 50_000),
            nowMilliseconds: 100);

        Assert.Multiple(() =>
        {
            Assert.That(fallback.Status, Is.EqualTo(AiPursuitNavigationStatus.Active));
            Assert.That(fallback.State!.TargetPointId, Is.EqualTo(50));
            Assert.That(result.RequestPrepared, Is.True);
            Assert.That(result.EffectiveNavigation.State!.Plan.JunctionDecisions,
                Contains.Key(2));
            Assert.That(result.Evaluation.Selection!.ToPointId, Is.EqualTo(10));
            Assert.That(result.ControllerPhase, Is.EqualTo(AiLaneChangePhase.WaitingForGap));
            Assert.That(result.ControllerEvent!.Kind,
                Is.EqualTo(AiPursuitLaneChangeEventKind.Required));
        });
    }

    [Test]
    public void ShorterAdjacentTargetLanePreparesRequestWhenCurrentRouteIsValid()
    {
        var controller = new AiLaneChangeController(60, 3000);
        var pipeline = CreatePipeline(controller, currentReachesPhysicalTarget: true);

        var result = pipeline.Evaluate(
            0, 0, 100, ActiveFallback(), null,
            new AiPursuitLaneChangeOptions(true, 60, 3000, 1000),
            new AiRouteSearchLimits(20_000, 50_000),
            100);

        Assert.Multiple(() =>
        {
            Assert.That(result.RequestPrepared, Is.True);
            Assert.That(result.Evaluation.Selection!.Motivation,
                Is.EqualTo(AiPursuitLaneMotivation.TargetLaneAlignment));
            Assert.That(result.Evaluation.Selection.JunctionId, Is.Null);
            Assert.That(result.ControllerPhase, Is.EqualTo(AiLaneChangePhase.WaitingForGap));
        });
    }

    [Test]
    public void ShortAnchorAlignmentPreparesWhenBothSplinesHaveTransitionCapacity()
    {
        var controller = new AiLaneChangeController(60, 3000);
        var pipeline = CreateShortAnchorPipeline(controller, 30, 100, 100);

        var result = pipeline.Evaluate(
            0, 0, 100, ActiveFallback(), null,
            new AiPursuitLaneChangeOptions(true, 60, 3000, 1000),
            new AiRouteSearchLimits(20_000, 50_000),
            100);

        Assert.Multiple(() =>
        {
            Assert.That(result.RequestPrepared, Is.True);
            Assert.That(result.Evaluation.Selection!.Motivation,
                Is.EqualTo(AiPursuitLaneMotivation.TargetLaneAlignment));
            Assert.That(result.Evaluation.Selection.DistanceToDecisionMeters, Is.Null);
            Assert.That(result.ControllerPhase, Is.EqualTo(AiLaneChangePhase.WaitingForGap));
        });
    }

    [TestCase(50f, 100f)]
    [TestCase(100f, 50f)]
    public void ShortAnchorAlignmentRejectsInsufficientPhysicalTransitionCapacity(
        float sourceLength,
        float destinationLength)
    {
        var controller = new AiLaneChangeController(60, 3000);
        var pipeline = CreateShortAnchorPipeline(
            controller, 30, sourceLength, destinationLength);

        var result = pipeline.Evaluate(
            0, 0, sourceLength, ActiveFallback(), null,
            new AiPursuitLaneChangeOptions(true, 60, 3000, 1000),
            new AiRouteSearchLimits(20_000, 50_000),
            100);

        Assert.Multiple(() =>
        {
            Assert.That(result.RequestPrepared, Is.False);
            Assert.That(result.Evaluation.Reason,
                Is.EqualTo(AiPursuitLaneEvaluationReason.InsufficientPreparationDistance));
            Assert.That(result.Evaluation.RequiredTransitionDistanceMeters, Is.EqualTo(60));
            Assert.That(result.Evaluation.SourceAvailableDistanceMeters,
                Is.EqualTo(Math.Min(sourceLength, 60)));
            Assert.That(result.Evaluation.DestinationAvailableDistanceMeters,
                Is.EqualTo(Math.Min(destinationLength, 60)));
            Assert.That(result.ControllerPhase, Is.EqualTo(AiLaneChangePhase.None));
        });
    }

    [Test]
    public void PendingAlignmentDoesNotSkipPastImmediateNeighbor()
    {
        var plannedStarts = new List<int>();
        var selector = new AiPursuitLaneSelector(
            pointId => pointId == 0
                ? new AiAdjacentLanePoints(10, -1)
                : new AiAdjacentLanePoints(20, 0),
            (_, _) => true,
            (start, _, _) =>
            {
                plannedStarts.Add(start);
                return Search(start switch
                {
                    0 => Plan(0, 99, 800),
                    10 => Plan(10, 99, 400),
                    20 => Plan(20, 99, 100),
                    _ => null
                });
            });
        var pipeline = new AiPursuitLaneChangePipeline(
            selector, new AiLaneChangeController(60, 3000), Cursor, _ => 1000);

        var result = pipeline.Evaluate(
            0, 0, 100, ActiveFallback(), null,
            new AiPursuitLaneChangeOptions(true, 60, 3000, 1000),
            new AiRouteSearchLimits(20_000, 50_000),
            100);

        Assert.Multiple(() =>
        {
            Assert.That(result.Evaluation.Selection!.ToPointId, Is.EqualTo(10));
            Assert.That(plannedStarts, Does.Not.Contain(20));
        });
    }

    [Test]
    public void NearJunctionRoutePreparationWinsOverShorterAlignment()
    {
        var selector = new AiPursuitLaneSelector(
            _ => new AiAdjacentLanePoints(10, 20),
            (_, _) => true,
            (start, _, _) => Search(start switch
            {
                10 => Plan(10, 99, 420, junctionId: 2),
                20 => Plan(20, 99, 120),
                _ => null
            }),
            plan => plan.JunctionDecisions.ContainsKey(2)
                ? new AiPursuitLaneDecision(2, 420)
                : null);
        var pipeline = new AiPursuitLaneChangePipeline(
            selector, new AiLaneChangeController(60, 3000), Cursor, _ => 1000);

        var result = pipeline.Evaluate(
            0, 0, 100, ActiveFallback(), null,
            new AiPursuitLaneChangeOptions(true, 60, 3000, 1000),
            new AiRouteSearchLimits(20_000, 50_000),
            100);

        Assert.Multiple(() =>
        {
            Assert.That(result.Evaluation.Selection!.ToPointId, Is.EqualTo(10));
            Assert.That(result.Evaluation.Selection.Motivation,
                Is.EqualTo(AiPursuitLaneMotivation.FutureJunction));
        });
    }

    [Test]
    public void AlignmentCannotHideJunctionInsidePhysicalTransition()
    {
        var selector = new AiPursuitLaneSelector(
            _ => new AiAdjacentLanePoints(10, -1),
            (_, _) => true,
            (start, targets, _) => start == 10 && targets.Contains(99)
                ? Search(Plan(10, 99, 30, junctionId: 2))
                : Search(null),
            plan => plan.JunctionDecisions.ContainsKey(2)
                ? new AiPursuitLaneDecision(2, 20)
                : null);
        var pipeline = new AiPursuitLaneChangePipeline(
            selector, new AiLaneChangeController(60, 3000), Cursor, _ => 1000);

        var result = pipeline.Evaluate(
            0, 0, 100, ActiveFallback(), null,
            new AiPursuitLaneChangeOptions(true, 60, 3000, 1000),
            new AiRouteSearchLimits(20_000, 50_000),
            100);

        Assert.Multiple(() =>
        {
            Assert.That(result.RequestPrepared, Is.False);
            Assert.That(result.Evaluation.Selection, Is.Null);
            Assert.That(result.Evaluation.Reason,
                Is.EqualTo(AiPursuitLaneEvaluationReason.InsufficientPreparationDistance));
            Assert.That(result.ControllerPhase, Is.EqualTo(AiLaneChangePhase.None));
        });
    }

    [Test]
    public void CooldownReportsTypedReasonAndDoesNotPrepareRequest()
    {
        var controller = new AiLaneChangeController(60, 3000);
        var pipeline = CreatePipeline(controller);
        var navigation = ActiveFallback();
        var options = new AiPursuitLaneChangeOptions(true, 60, 3000, 1000);
        var limits = new AiRouteSearchLimits(20_000, 50_000);

        var prepared = pipeline.Evaluate(0, 0, 100, navigation, null,
            options, limits, 0);
        controller.UpdateWaiting(AiLaneChangeSafetyStatus.Safe, 0);
        Assert.That(controller.TryMove(60, 0, out var movement), Is.True);
        Assert.That(movement.Completed, Is.True);

        var result = pipeline.Evaluate(0, 0, 100, navigation, null,
            options, limits, 100);

        Assert.Multiple(() =>
        {
            Assert.That(result.RequestPrepared, Is.False);
            Assert.That(result.ControllerPhase, Is.EqualTo(AiLaneChangePhase.Cooldown));
            Assert.That(result.Evaluation.Reason,
                Is.EqualTo(AiPursuitLaneEvaluationReason.Cooldown));
        });
    }

    [Test]
    public void CooldownPreventsImmediateAlignmentReversal()
    {
        var reverse = false;
        var controller = new AiLaneChangeController(60, 3000);
        var selector = new AiPursuitLaneSelector(
            pointId => pointId == 0
                ? new AiAdjacentLanePoints(10, -1)
                : new AiAdjacentLanePoints(-1, 0),
            (_, _) => true,
            (start, _, _) => Search((start, reverse) switch
            {
                (0, false) => Plan(0, 99, 600),
                (10, false) => Plan(10, 99, 400),
                (10, true) => Plan(10, 99, 600),
                (0, true) => Plan(0, 99, 400),
                _ => null
            }));
        var pipeline = new AiPursuitLaneChangePipeline(
            selector, controller, Cursor, _ => 1000);
        var options = new AiPursuitLaneChangeOptions(true, 60, 3000, 1000);
        var limits = new AiRouteSearchLimits(20_000, 50_000);

        var first = pipeline.Evaluate(
            0, 0, 100, ActiveFallback(), null, options, limits, 0);
        Assert.That(first.RequestPrepared, Is.True);
        controller.UpdateWaiting(AiLaneChangeSafetyStatus.Safe, 0);
        Assert.That(controller.TryMove(60, 0, out var movement), Is.True);
        Assert.That(movement.Completed, Is.True);
        reverse = true;

        var result = pipeline.Evaluate(
            10, 0, 100, ActiveFallback(), null, options, limits, 1000);

        Assert.Multiple(() =>
        {
            Assert.That(result.RequestPrepared, Is.False);
            Assert.That(result.Evaluation.Selection!.ToPointId, Is.Zero);
            Assert.That(result.Evaluation.Reason,
                Is.EqualTo(AiPursuitLaneEvaluationReason.Cooldown));
        });
    }

    private static AiPursuitLaneChangePipeline CreatePipeline(
        AiLaneChangeController controller,
        bool currentReachesPhysicalTarget = false)
    {
        var destinationPlan = new AiRoutePlan(
            420,
            [new AiRouteNode(10, 0), new AiRouteNode(99, 420)],
            new Dictionary<int, bool> { [2] = true });
        var selector = new AiPursuitLaneSelector(
            _ => new AiAdjacentLanePoints(10, -1),
            (_, _) => true,
            (start, targets, _) =>
            {
                if (start == 0 && currentReachesPhysicalTarget)
                    return Search(new AiRoutePlan(500,
                        [new AiRouteNode(0, 0), new AiRouteNode(99, 500)],
                        new Dictionary<int, bool>()));
                return start == 10 && targets.Contains(99)
                    ? Search(destinationPlan)
                    : Search(null);
            },
            plan => plan.JunctionDecisions.ContainsKey(2)
                ? new AiPursuitLaneDecision(2, 420)
                : null);
        return new AiPursuitLaneChangePipeline(selector, controller, Cursor, _ => 1000);
    }

    private static AiPursuitLaneChangePipeline CreateShortAnchorPipeline(
        AiLaneChangeController controller,
        float routeDistance,
        float sourceLength,
        float destinationLength)
    {
        var selector = new AiPursuitLaneSelector(
            _ => new AiAdjacentLanePoints(10, -1),
            (_, _) => true,
            (start, targets, _) => start == 10 && targets.Contains(99)
                ? Search(Plan(10, 99, routeDistance))
                : Search(null));
        AiSplineCursor CreateCursor(int pointId, float progress) =>
            new(
                pointId,
                progress,
                _ => null,
                current => current == 0 ? sourceLength : destinationLength,
                (current, distance) => new AiSplinePose(
                    new Vector3(0, 0, current * 1000 + distance),
                    Vector3.UnitZ));

        return new AiPursuitLaneChangePipeline(
            selector,
            controller,
            CreateCursor,
            pointId => pointId == 0 ? sourceLength : destinationLength);
    }

    private static AiPursuitNavigationResult ActiveFallback()
    {
        var plan = new AiRoutePlan(
            100,
            [new AiRouteNode(0, 0), new AiRouteNode(50, 100)],
            new Dictionary<int, bool>());
        return new AiPursuitNavigationResult(
            AiPursuitNavigationStatus.Active,
            new AiPursuitRouteState(50, plan, 1, null)
            {
                PreferredPhysicalTargetPointId = 99
            },
            AiPursuitRouteUpdateKind.Selected,
            AiRouteSearchFailure.None,
            2,
            new AiPursuitTargetLocationDiagnostics([99], [50], []),
            100,
            0)
        {
            TargetPointIds = [50, 99],
            PreferredPhysicalTargetPointId = 99
        };
    }

    private static AiSplineCursor Cursor(int pointId, float progress) =>
        new(
            pointId,
            progress,
            current => current + 1,
            _ => 1000,
            (current, distance) => new AiSplinePose(
                new Vector3(0, 0, current * 1000 + distance),
                Vector3.UnitZ));

    private static AiRouteSearchResult Search(AiRoutePlan? plan) =>
        new(
            plan,
            plan == null ? AiRouteSearchFailure.Unreachable : AiRouteSearchFailure.None,
            1);

    private static AiRoutePlan Plan(
        int startPointId,
        int targetPointId,
        float distanceMeters,
        int? junctionId = null) =>
        new(
            distanceMeters,
            [new AiRouteNode(startPointId, 0), new AiRouteNode(targetPointId, distanceMeters)],
            junctionId.HasValue
                ? new Dictionary<int, bool> { [junctionId.Value] = true }
                : new Dictionary<int, bool>());
}
