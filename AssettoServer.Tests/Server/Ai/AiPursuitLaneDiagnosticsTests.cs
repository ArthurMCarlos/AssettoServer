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
    public void MotivationChangeRepublishesEvaluationWithoutDistanceSpam()
    {
        var tracker = new AiPursuitLaneDiagnosticTracker();
        tracker.PublishEvaluation(0, 99, 4, Evaluation(
            500, motivation: AiPursuitLaneMotivation.TargetLaneAlignment));

        var duplicate = tracker.PublishEvaluation(0, 99, 4, Evaluation(
            498, motivation: AiPursuitLaneMotivation.TargetLaneAlignment));
        var changed = tracker.PublishEvaluation(0, 99, 4, Evaluation(
            498, motivation: AiPursuitLaneMotivation.FutureJunction));

        Assert.Multiple(() =>
        {
            Assert.That(duplicate, Is.Null);
            Assert.That(changed, Is.Not.Null);
            Assert.That(changed!.Motivation,
                Is.EqualTo(AiPursuitLaneMotivation.FutureJunction));
        });
    }

    [Test]
    public void DirectAlignmentDiagnosticHasNoJunctionButKeepsRouteEvidence()
    {
        var diagnostic = new AiPursuitLaneDiagnosticTracker().PublishEvaluation(
            171761,
            283943,
            1,
            Evaluation(
                94.6f,
                junctionId: null,
                motivation: AiPursuitLaneMotivation.TargetLaneAlignment,
                relation: AiPursuitLanePhysicalRelation.ImmediateLeft));

        Assert.Multiple(() =>
        {
            Assert.That(diagnostic!.JunctionId, Is.Null);
            Assert.That(diagnostic.Motivation,
                Is.EqualTo(AiPursuitLaneMotivation.TargetLaneAlignment));
            Assert.That(diagnostic.PhysicalRelation,
                Is.EqualTo(AiPursuitLanePhysicalRelation.ImmediateLeft));
            Assert.That(diagnostic.CandidateLaneRoutes.Single().RouteDistanceMeters,
                Is.EqualTo(600));
        });
    }

    [Test]
    public void PhysicalRelationChangeRepublishesEvaluation()
    {
        var tracker = new AiPursuitLaneDiagnosticTracker();
        tracker.PublishEvaluation(0, 99, 4, Evaluation(
            500, relation: AiPursuitLanePhysicalRelation.ImmediateLeft));

        var changed = tracker.PublishEvaluation(0, 99, 4, Evaluation(
            498, relation: AiPursuitLanePhysicalRelation.ImmediateRight));

        Assert.That(changed!.PhysicalRelation,
            Is.EqualTo(AiPursuitLanePhysicalRelation.ImmediateRight));
    }

    [TestCase(
        AiPursuitLaneEvaluationReason.NonAdjacent,
        AiPursuitLaneChangeDiagnosticReason.NonAdjacent)]
    [TestCase(
        AiPursuitLaneEvaluationReason.InvalidGeometry,
        AiPursuitLaneChangeDiagnosticReason.InvalidGeometry)]
    public void GeometryRejectionUsesPublicDiagnosticReason(
        AiPursuitLaneEvaluationReason evaluationReason,
        AiPursuitLaneChangeDiagnosticReason expectedReason)
    {
        var evaluation = Evaluation(500) with
        {
            Kind = AiPursuitLaneSelectionKind.Unreachable,
            Selection = null,
            Reason = evaluationReason
        };

        var diagnostic = new AiPursuitLaneDiagnosticTracker().PublishEvaluation(
            0, 99, 4, evaluation);

        Assert.That(diagnostic!.Reason, Is.EqualTo(expectedReason));
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

    [Test]
    public void MovingPolicePointAndRouteRevisionDoNotRepublishSameEvaluation()
    {
        var tracker = new AiPursuitLaneDiagnosticTracker();

        var first = tracker.PublishEvaluation(100, 99, 4, Evaluation(500));
        var duplicate = tracker.PublishEvaluation(101, 99, 5, Evaluation(498));

        Assert.That(first, Is.Not.Null);
        Assert.That(duplicate, Is.Null);
    }

    [Test]
    public void GateReasonPublishesOnceAndRetainsNullableLaneEvidence()
    {
        var tracker = new AiPursuitLaneDiagnosticTracker();

        var first = tracker.PublishGate(
            100, null, 4, AiPursuitLaneChangeDiagnosticReason.NoPhysicalTarget);
        var duplicate = tracker.PublishGate(
            101, null, 5, AiPursuitLaneChangeDiagnosticReason.NoPhysicalTarget);

        Assert.Multiple(() =>
        {
            Assert.That(first, Is.Not.Null);
            Assert.That(first!.FromPointId, Is.Null);
            Assert.That(first.ToPointId, Is.Null);
            Assert.That(first.Direction, Is.Null);
            Assert.That(duplicate, Is.Null);
        });
    }

    [Test]
    public void ChangeInSecondRejectedLanePublishesNewEvaluation()
    {
        var tracker = new AiPursuitLaneDiagnosticTracker();
        tracker.PublishEvaluation(0, 99, 4, EvaluationWithSecondCandidate(7));

        var changed = tracker.PublishEvaluation(1, 99, 5,
            EvaluationWithSecondCandidate(8));

        Assert.That(changed, Is.Not.Null);
        Assert.That(changed!.Revision, Is.EqualTo(2));
    }

    [Test]
    public void RejectionUsesCandidateWhoseEvidenceMatchesPublishedReason()
    {
        var evaluation = Evaluation(500) with
        {
            Kind = AiPursuitLaneSelectionKind.Unreachable,
            Selection = null,
            Reason = AiPursuitLaneEvaluationReason.InsufficientPreparationDistance,
            CandidateLaneRoutes =
            [
                new AiPursuitLaneRouteDiagnostic(
                    10,
                    AiLaneChangeDirection.Left,
                    AiRouteSearchFailure.DistanceLimit,
                    null,
                    500,
                    0,
                    null,
                    null,
                    AiPursuitLaneEvaluationReason.NoForwardRoute),
                new AiPursuitLaneRouteDiagnostic(
                    20,
                    AiLaneChangeDirection.Right,
                    AiRouteSearchFailure.None,
                    30,
                    30,
                    0,
                    null,
                    null,
                    AiPursuitLaneEvaluationReason.InsufficientPreparationDistance)
                {
                    Motivation = AiPursuitLaneMotivation.TargetLaneAlignment,
                    PhysicalRelation = AiPursuitLanePhysicalRelation.ImmediateRight
                }
            ]
        };

        var diagnostic = new AiPursuitLaneDiagnosticTracker().PublishEvaluation(
            0, 99, 4, evaluation);

        Assert.Multiple(() =>
        {
            Assert.That(diagnostic!.ToPointId, Is.EqualTo(20));
            Assert.That(diagnostic.Direction, Is.EqualTo(AiLaneChangeDirection.Right));
            Assert.That(diagnostic.Motivation,
                Is.EqualTo(AiPursuitLaneMotivation.TargetLaneAlignment));
            Assert.That(diagnostic.PhysicalRelation,
                Is.EqualTo(AiPursuitLanePhysicalRelation.ImmediateRight));
        });
    }

    [Test]
    public void PhysicalCapacityEvidenceIsPublishedForRejectedSelection()
    {
        var evaluation = Evaluation(
            0,
            junctionId: null,
            motivation: AiPursuitLaneMotivation.TargetLaneAlignment) with
        {
            Reason = AiPursuitLaneEvaluationReason.InsufficientPreparationDistance,
            RequiredTransitionDistanceMeters = 60,
            SourceAvailableDistanceMeters = 50,
            DestinationAvailableDistanceMeters = 60
        };

        var diagnostic = new AiPursuitLaneDiagnosticTracker().PublishEvaluation(
            0, 99, 4, evaluation);

        Assert.Multiple(() =>
        {
            Assert.That(diagnostic!.RequiredTransitionDistanceMeters, Is.EqualTo(60));
            Assert.That(diagnostic.SourceAvailableDistanceMeters, Is.EqualTo(50));
            Assert.That(diagnostic.DestinationAvailableDistanceMeters, Is.EqualTo(60));
            Assert.That(diagnostic.DistanceToDecisionMeters, Is.Null);
            Assert.That(diagnostic.Motivation,
                Is.EqualTo(AiPursuitLaneMotivation.TargetLaneAlignment));
        });
    }

    private static AiPursuitLaneSelectionResult EvaluationWithSecondCandidate(
        int secondJunction)
    {
        var evaluation = Evaluation(500);
        return evaluation with
        {
            CandidateLaneRoutes =
            [
                .. evaluation.CandidateLaneRoutes,
                new AiPursuitLaneRouteDiagnostic(
                    20,
                    AiLaneChangeDirection.Right,
                    AiRouteSearchFailure.None,
                    700,
                    700,
                    1,
                    secondJunction,
                    600,
                    AiPursuitLaneEvaluationReason.BeyondLookahead)
            ]
        };
    }

    private static AiPursuitLaneSelectionResult Evaluation(
        float distanceToDecision,
        int? junctionId = 2,
        AiPursuitLaneMotivation motivation = AiPursuitLaneMotivation.FutureJunction,
        AiPursuitLanePhysicalRelation relation = AiPursuitLanePhysicalRelation.ImmediateLeft)
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
            motivation == AiPursuitLaneMotivation.TargetLaneAlignment
                ? null
                : distanceToDecision,
            AiPursuitLaneEvaluationReason.RoutePreparation);
        var plan = new AiRoutePlan(
            600,
            [new AiRouteNode(10, 0), new AiRouteNode(99, 600)],
            junctionId.HasValue
                ? new Dictionary<int, bool> { [junctionId.Value] = true }
                : new Dictionary<int, bool>());
        return new AiPursuitLaneSelectionResult(
            AiPursuitLaneSelectionKind.Change,
            new AiPursuitLaneSelection(
                0,
                10,
                AiLaneChangeDirection.Left,
                plan,
                motivation == AiPursuitLaneMotivation.TargetLaneAlignment
                    ? null
                    : distanceToDecision)
            {
                JunctionId = junctionId,
                Motivation = motivation,
                PhysicalRelation = relation
            },
            [new AiPursuitLaneCandidateDiagnostic(10, AiLaneChangeDirection.Left, null)])
        {
            Reason = AiPursuitLaneEvaluationReason.RoutePreparation,
            CurrentLaneRoute = current,
            CandidateLaneRoutes = [candidate]
        };
    }
}
