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

    private static AiPursuitPitDecision Decide((bool Left, bool Right) safety,
        AiPursuitPitControllerState? previous = null) =>
        new AiPursuitPitController().Update(new AiPursuitPitRequest(
            new AiPursuitPitOptions(true, 6, 10 / 3.6f, .8f, 1200, 3000),
            TargetSessionId, true, true, true, 5.8f, 9.6f / 3.6f,
            AiPursuitDrivingState.ClosePressure, AiPursuitDrivingReason.ClosePressure,
            AiLaneChangePhase.Cooldown, false, safety.Left, safety.Right, 1, 1000, previous));

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
    public void DistantNonTargetWithNativeDefaultZeroLengthDoesNotBlockBothSides()
    {
        var farAway = new AiPursuitPitObstacle(8, new Vector3(1000, 0, 1000),
            Vector3.UnitZ * 20, 0);
        Assert.That(Evaluate(Target, farAway), Is.EqualTo((true, true)));
        Assert.That(Evaluate(Target, farAway with { LengthMeters = 4 }),
            Is.EqualTo((true, true)));
        var safety = Evaluate(Target, farAway);
        var armed = Decide(safety);
        Assert.That(armed.State.Phase, Is.EqualTo(AiPursuitPitPhase.Armed));
        Assert.That(Decide(safety, armed.State).Diagnostics?.EventKind,
            Is.EqualTo(AiPursuitPitEventKind.Started));
    }

    [Test]
    public void PoliceWithNativeDefaultZeroLengthCanUseAnEmptyCorridor()
    {
        var safety = AiPursuitPitGeometry.EvaluateSideSafety(Vector3.Zero,
            Vector3.UnitZ, 20, 0, 3, .8f, TargetSessionId, []);
        Assert.That(safety, Is.EqualTo((true, true)));
    }

    [TestCase(3f, false, true)]
    [TestCase(-3f, true, false)]
    [TestCase(0f, false, false)]
    public void ZeroLengthThirdPartyStillBlocksOccupiedCorridor(float lateral, bool left, bool right)
    {
        Assert.That(Evaluate(Target, new AiPursuitPitObstacle(11,
            new Vector3(lateral, 0, 2), Vector3.UnitZ * 20, 0)),
            Is.EqualTo((left, right)));
    }

    [TestCase(-1f)]
    [TestCase(float.NaN)]
    [TestCase(float.PositiveInfinity)]
    public void InvalidLengthStillBlocksBothSides(float length)
    {
        Assert.That(Evaluate(Target, new AiPursuitPitObstacle(8,
            new Vector3(1000, 0, 1000), Vector3.UnitZ * 20, length)),
            Is.EqualTo((false, false)));
    }

    [TestCase(7f, 20f, "BlockedFront")]
    [TestCase(12f, 15f, "BlockedFront")]
    [TestCase(-7f, 20f, "BlockedRear")]
    [TestCase(-12f, 25f, "BlockedRearClosing")]
    public void NativeZeroLengthsKeepBaseGapAndClosingMargins(float z, float speed, string reason)
    {
        var result = AiPursuitPitGeometry.EvaluateSideSafetyDetailed(Vector3.Zero,
            Vector3.UnitZ, 20, 0, 3, .8f, 10,
            [new AiPursuitPitObstacle(11, new Vector3(0, 0, z), Vector3.UnitZ * speed, 0, "Player")]);
        Assert.Multiple(() =>
        {
            Assert.That(result.Left, Is.False);
            Assert.That(result.Right, Is.False);
            Assert.That(result.LeftReason, Is.EqualTo(reason));
            Assert.That(result.LeftBlocker?.SessionId, Is.EqualTo(11));
            Assert.That(result.RightBlocker?.SessionId, Is.EqualTo(11));
        });
    }

    [Test]
    public void DetailedEvidenceDistinguishesInvalidThirdPartyFromTargetAndGeometry()
    {
        var result = AiPursuitPitGeometry.EvaluateSideSafetyDetailed(Vector3.Zero,
            Vector3.UnitZ, 20, 4, 3, .8f, 10,
            [Target with { LengthMeters = float.NaN },
             new AiPursuitPitObstacle(8, new Vector3(1000, 0, 1000), Vector3.UnitZ * 20,
                 -1, "AI", "traffic", 3)]);
        Assert.Multiple(() =>
        {
            Assert.That(result.LeftReason, Is.EqualTo("InvalidObstacleLength"));
            Assert.That(result.RightReason, Is.EqualTo("InvalidObstacleLength"));
            Assert.That(result.LeftBlocker?.SessionId, Is.EqualTo(8));
            Assert.That(result.LeftBlocker?.Kind, Is.EqualTo("AI"));
            Assert.That(result.LeftBlocker?.SpawnCounter, Is.EqualTo(3));
        });
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
