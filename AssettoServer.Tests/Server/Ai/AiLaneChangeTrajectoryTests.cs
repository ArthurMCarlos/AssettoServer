using System.Numerics;
using AssettoServer.Server.Ai;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiLaneChangeTrajectoryTests
{
    [Test]
    public void CursorPreservesDistanceAcrossSegmentBoundary()
    {
        var lengths = new Dictionary<int, float> { [0] = 10, [1] = 20 };
        var cursor = new AiSplineCursor(
            0,
            8,
            point => point == 0 ? 1 : null,
            point => lengths[point],
            (point, progress) => new AiSplinePose(new Vector3(point * 10 + progress, 0, 0), Vector3.UnitX));

        Assert.That(cursor.TryAdvance(7), Is.True);
        Assert.Multiple(() =>
        {
            Assert.That(cursor.PointId, Is.EqualTo(1));
            Assert.That(cursor.SegmentProgressMeters, Is.EqualTo(5).Within(0.001));
            Assert.That(cursor.Evaluate().Position.X, Is.EqualTo(15).Within(0.001));
        });
    }

    [TestCase(0.0f, 0.0f)]
    [TestCase(0.5f, 1.5f)]
    [TestCase(1.0f, 3.0f)]
    public void BlendIsContinuousAndMatchesExpectedLateralPosition(float progress, float expectedZ)
    {
        var source = new AiSplinePose(Vector3.Zero, Vector3.UnitX);
        var destination = new AiSplinePose(new Vector3(0, 0, 3), Vector3.UnitX);

        var pose = AiLaneChangeTrajectory.Blend(source, destination, progress, 60);

        Assert.Multiple(() =>
        {
            Assert.That(pose.Position.Z, Is.EqualTo(expectedZ).Within(0.001));
            Assert.That(float.IsFinite(pose.Tangent.X), Is.True);
            Assert.That(pose.Tangent.Length(), Is.EqualTo(1).Within(0.001));
        });
    }

    [Test]
    public void BlendHasNoLateralTangentAtEndpoints()
    {
        var source = new AiSplinePose(Vector3.Zero, Vector3.UnitX);
        var destination = new AiSplinePose(new Vector3(0, 0, 3), Vector3.UnitX);

        Assert.Multiple(() =>
        {
            Assert.That(AiLaneChangeTrajectory.Blend(source, destination, 0, 60).Tangent.Z,
                Is.EqualTo(0).Within(0.001));
            Assert.That(AiLaneChangeTrajectory.Blend(source, destination, 1, 60).Tangent.Z,
                Is.EqualTo(0).Within(0.001));
        });
    }
}
