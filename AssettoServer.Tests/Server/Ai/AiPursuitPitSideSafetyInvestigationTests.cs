using System.Numerics;
using AssettoServer.Server.Ai;
using NUnit.Framework;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitPitSideSafetyInvestigationTests
{
    private const byte TargetSessionId = 10;
    // Session 12's own AiState is excluded by reference in GetPitSideSafety.
    // The geometry helper receives its pose/dimensions, not its session ID.
    private static readonly AiPursuitPitObstacle Target = new(
        TargetSessionId, new Vector3(.8f, 0, 7.7f), Vector3.UnitZ * 20, 4);

    private static (bool Left, bool Right) Evaluate(params AiPursuitPitObstacle[] vehicles) =>
        AiPursuitPitGeometry.EvaluateSideSafety(Vector3.Zero, Vector3.UnitZ,
            20 + 9.6f / 3.6f, 4, 3, .8f, TargetSessionId, vehicles);

    private static AiPursuitPitDecision Decide((bool Left, bool Right) safety) =>
        new AiPursuitPitController().Update(new AiPursuitPitRequest(
            new AiPursuitPitOptions(true, 6, 10 / 3.6f, .8f, 1200, 3000),
            TargetSessionId, true, true, true, 5.8f, 9.6f / 3.6f,
            AiPursuitDrivingState.ClosePressure, AiPursuitDrivingReason.ClosePressure,
            AiLaneChangePhase.Cooldown, false, safety.Left, safety.Right, 1, 1000, null));

    [Test]
    public void ReportedEligibleWindowWithOnlyTargetArmsWithoutChangingThresholds()
    {
        Assert.That(AiPursuitPitGeometry.IsTargetAligned(Vector3.Zero,
            Vector3.UnitZ, Target.Position, Target.Velocity, 3), Is.True);
        var safety = Evaluate(Target);
        var decision = Decide(safety);
        Assert.Multiple(() =>
        {
            Assert.That(safety, Is.EqualTo((true, true)));
            Assert.That(decision.State.TargetSessionId, Is.EqualTo(10));
            Assert.That(decision.State.Phase, Is.EqualTo(AiPursuitPitPhase.Armed));
            Assert.That(decision.EligibilityRejection, Is.Null);
        });
    }

    [TestCase((byte)8, 3f, false, true)]
    [TestCase((byte)8, -3f, true, false)]
    [TestCase((byte)11, 3f, false, true)]
    [TestCase((byte)11, -3f, true, false)]
    public void NonTargetIdentityBlocksItsSide(byte sessionId, float lateral,
        bool left, bool right)
    {
        var safety = Evaluate(Target, new AiPursuitPitObstacle(sessionId,
            new Vector3(lateral, 0, 2), Vector3.UnitZ * 20, 4));
        Assert.That(safety, Is.EqualTo((left, right)));
        Assert.That(Decide(safety).State.Phase, Is.EqualTo(AiPursuitPitPhase.Armed));
    }

    [Test]
    public void ThirdPartiesOnBothSidesProduceValidBlockedSide()
    {
        var safety = Evaluate(Target,
            new AiPursuitPitObstacle(8, new Vector3(3, 0, 2), Vector3.UnitZ * 20, 4),
            new AiPursuitPitObstacle(11, new Vector3(-3, 0, 2), Vector3.UnitZ * 20, 4));
        Assert.That(safety, Is.EqualTo((false, false)));
        Assert.That(Decide(safety).EligibilityRejection,
            Is.EqualTo(AiPursuitPitAbortReason.BlockedSide));
    }

    [Test]
    public void TargetIsExcludedBeforeEvenItsDimensionValidation()
    {
        Assert.That(Evaluate(Target with { LengthMeters = 0 }), Is.EqualTo((true, true)));
    }

    [Test]
    public void DistantNonTargetWithNativeDefaultZeroLengthBlocksBothSides()
    {
        var farAway = new AiPursuitPitObstacle(8, new Vector3(1000, 0, 1000),
            Vector3.UnitZ * 20, 0);
        Assert.That(Evaluate(Target, farAway), Is.EqualTo((false, false)));
        Assert.That(Evaluate(Target, farAway with { LengthMeters = 4 }),
            Is.EqualTo((true, true)));
        Assert.That(Decide(Evaluate(Target, farAway)).EligibilityRejection,
            Is.EqualTo(AiPursuitPitAbortReason.BlockedSide));
    }

    [Test]
    public void PoliceWithNativeDefaultZeroLengthBlocksEvenAnEmptyObstacleList()
    {
        var safety = AiPursuitPitGeometry.EvaluateSideSafety(Vector3.Zero,
            Vector3.UnitZ, 20, 0, 3, .8f, TargetSessionId, []);
        Assert.That(safety, Is.EqualTo((false, false)));
    }

    [Test]
    public void CenteredThirdPartyCanLegitimatelyIntersectBothProbeCorridors()
    {
        Assert.That(Evaluate(Target, new AiPursuitPitObstacle(8,
            new Vector3(0, 0, 2), Vector3.UnitZ * 20, 4)),
            Is.EqualTo((false, false)));
    }

    [Test]
    public void SamePlanarCoordinatesOnAnotherHeightStillBlockCurrentEvaluator()
    {
        Assert.That(Evaluate(Target, new AiPursuitPitObstacle(8,
            new Vector3(0, 20, 2), Vector3.UnitZ * 20, 4)),
            Is.EqualTo((false, false)));
    }
}
