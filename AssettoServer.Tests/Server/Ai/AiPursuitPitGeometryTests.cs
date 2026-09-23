using System.Numerics;
using AssettoServer.Server.Ai;
using NUnit.Framework;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitPitGeometryTests
{
    [Test]
    public void TargetMustBeAheadMovingForwardInTheCurrentPhysicalLane()
    {
        var position = Vector3.Zero;
        var forward = Vector3.UnitZ;
        Assert.Multiple(() =>
        {
            Assert.That(AiPursuitPitGeometry.IsTargetAligned(position, forward,
                new Vector3(0, 0, 6), new Vector3(0, 0, 10), 3), Is.True);
            Assert.That(AiPursuitPitGeometry.IsTargetAligned(position, forward,
                new Vector3(2, 0, 6), new Vector3(0, 0, 10), 3), Is.False);
            Assert.That(AiPursuitPitGeometry.IsTargetAligned(position, forward,
                new Vector3(0, 0, -6), new Vector3(0, 0, 10), 3), Is.False);
            Assert.That(AiPursuitPitGeometry.IsTargetAligned(position, forward,
                new Vector3(0, 0, 6), Vector3.Zero, 3), Is.False);
            Assert.That(AiPursuitPitGeometry.IsTargetAligned(position, forward,
                new Vector3(0, 0, 6), new Vector3(0, 0, -10), 3), Is.False);
        });
    }

    [Test]
    public void AlignmentMeasurementsExposeBehindAndLateralOffsetWithoutChangingDecision()
    {
        var measurements = AiPursuitPitGeometry.MeasureAlignment(
            Vector3.Zero, Vector3.UnitZ, new Vector3(2, 0, -6),
            new Vector3(0, 0, 10));
        Assert.Multiple(() =>
        {
            Assert.That(measurements.LongitudinalMeters, Is.EqualTo(-6));
            Assert.That(measurements.LateralMeters, Is.EqualTo(2));
            Assert.That(measurements.HeadingDot, Is.EqualTo(1));
        });
    }

    [Test]
    public void OverlayMovesLaterallyWithoutChangingSplineAnchorAndAddsLateralHeading()
    {
        var pose = AiPursuitPitGeometry.ApplyOffset(Vector3.Zero, Vector3.UnitZ,
            0, .1f, .1f, 20);
        Assert.Multiple(() =>
        {
            Assert.That(pose.Position.X, Is.GreaterThan(0));
            Assert.That(pose.Forward.X, Is.GreaterThan(0));
            Assert.That(pose.Forward.Length(), Is.EqualTo(1).Within(.0001));
            Assert.That(float.IsFinite(pose.Position.X), Is.True);
        });
        var returned = AiPursuitPitGeometry.ApplyOffset(Vector3.Zero, Vector3.UnitZ,
            .1f, 0, .1f, 20);
        Assert.That(returned.Forward.X, Is.LessThan(0));
    }

    [Test]
    public void SideProbeExcludesOnlyTargetAndBlocksAdjacentTraffic()
    {
        var obstacles = new[]
        {
            new AiPursuitPitObstacle(10, new Vector3(0, 0, 4), Vector3.UnitZ * 20, 4),
            new AiPursuitPitObstacle(8, new Vector3(3, 0, 2), Vector3.UnitZ * 20, 4)
        };
        var result = AiPursuitPitGeometry.EvaluateSideSafety(
            Vector3.Zero, Vector3.UnitZ, 20, 4, 3, .8f, 10, obstacles);
        Assert.Multiple(() =>
        {
            Assert.That(result.Left, Is.False);
            Assert.That(result.Right, Is.True);
        });
        var withSecondPlayer = AiPursuitPitGeometry.EvaluateSideSafety(
            Vector3.Zero, Vector3.UnitZ, 20, 4, 3, .8f, 10,
            [.. obstacles, new AiPursuitPitObstacle(11, new Vector3(-3, 0, 2),
                Vector3.UnitZ * 20, 4)]);
        Assert.That(withSecondPlayer.Right, Is.False);
        var unknownCar = AiPursuitPitGeometry.EvaluateSideSafety(
            Vector3.Zero, Vector3.UnitZ, 20, 4, 3, .8f, 10,
            [new AiPursuitPitObstacle(8, new Vector3(float.NaN, 0, 2),
                Vector3.Zero, 4)]);
        Assert.That(unknownCar, Is.EqualTo((false, false)));
    }

    [Test]
    public void JunctionDistanceUsesProgressWithinCurrentSegment()
    {
        Assert.Multiple(() =>
        {
            Assert.That(AiPursuitPitGeometry.IsJunctionWithinDistance(30, 20, 25), Is.True);
            Assert.That(AiPursuitPitGeometry.IsJunctionWithinDistance(30, 0, 25), Is.False);
            Assert.That(AiPursuitPitGeometry.IsJunctionWithinDistance(10, 20, 25), Is.False);
        });
    }

    [Test]
    public void NewAttemptWaitsForPreviousLateralOffsetToReturn()
    {
        Assert.Multiple(() =>
        {
            Assert.That(AiPursuitPitGeometry.CanStartOrContinue(null, .5f), Is.False);
            Assert.That(AiPursuitPitGeometry.CanStartOrContinue(
                AiPursuitPitPhase.Idle, .5f), Is.False);
            Assert.That(AiPursuitPitGeometry.CanStartOrContinue(
                AiPursuitPitPhase.Attempting, .5f), Is.True);
            Assert.That(AiPursuitPitGeometry.CanStartOrContinue(null, .01f), Is.True);
        });
    }
}
