using AssettoServer.Server.Ai;
using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitLaneDiagnosticsTests
{
    [Test]
    public void MovingDistanceDoesNotRepublishSameSemanticEvaluation()
    {
        var tracker = new AiPursuitLaneDiagnosticTracker();

        var first = tracker.PublishEvaluation(0, 99, 4, Evaluation(500));
        var duplicate = tracker.PublishEvaluation(0, 99, 4, Evaluation(498));

        Assert.Multiple(() =>
        {
            Assert.That(first, Is.Not.Null);
            Assert.That(first!.Revision, Is.EqualTo(1));
            Assert.That(first.Reason,
                Is.EqualTo(AiPursuitLaneChangeDiagnosticReason.RoutePreparation));
            Assert.That(duplicate, Is.Null);
        });
    }

    [Test]
    public void ChangedJunctionPublishesNewSemanticEvaluation()
    {
        var tracker = new AiPursuitLaneDiagnosticTracker();
        tracker.PublishEvaluation(0, 99, 4, Evaluation(500, junctionId: 2));

        var changed = tracker.PublishEvaluation(0, 99, 4, Evaluation(490, junctionId: 3));

        Assert.That(changed!.Revision, Is.EqualTo(2));
        Assert.That(changed.JunctionId, Is.EqualTo(3));
    }

    private static AiPursuitLaneSelectionResult Evaluation(
        float distanceToDecision,
        int junctionId = 2)
    {
        var current = new AiPursuitLaneRouteDiagnostic(
            0,
            null,
            AiRouteSearchFailure.Unreachable,
            null,
            80,
            0,
            null,
            null,
            AiPursuitLaneEvaluationReason.NoForwardRoute);
        var candidate = new AiPursuitLaneRouteDiagnostic(
            10,
            AiLaneChangeDirection.Left,
            AiRouteSearchFailure.None,
            600,
            600,
            1,
            junctionId,
            distanceToDecision,
            AiPursuitLaneEvaluationReason.RoutePreparation);
        var plan = new AiRoutePlan(
            600,
            [new AiRouteNode(10, 0), new AiRouteNode(99, 600)],
            new Dictionary<int, bool> { [junctionId] = true });
        return new AiPursuitLaneSelectionResult(
            AiPursuitLaneSelectionKind.Change,
            new AiPursuitLaneSelection(
                0,
                10,
                AiLaneChangeDirection.Left,
                plan,
                distanceToDecision)
            {
                JunctionId = junctionId
            },
            [new AiPursuitLaneCandidateDiagnostic(10, AiLaneChangeDirection.Left, null)])
        {
            Reason = AiPursuitLaneEvaluationReason.RoutePreparation,
            CurrentLaneRoute = current,
            CandidateLaneRoutes = [candidate]
        };
    }
}
