using System.Numerics;
using AssettoServer.Server.Ai;
using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitLaneChangePipelineTests
{
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
    public void CurrentLaneValidDoesNotPrepareRequest()
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
            Assert.That(result.RequestPrepared, Is.False);
            Assert.That(result.Evaluation.Reason,
                Is.EqualTo(AiPursuitLaneEvaluationReason.CurrentLaneValid));
            Assert.That(result.ControllerPhase, Is.EqualTo(AiLaneChangePhase.None));
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
}
