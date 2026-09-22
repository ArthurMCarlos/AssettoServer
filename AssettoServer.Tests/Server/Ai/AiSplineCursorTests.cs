using System.Numerics;
using AssettoServer.Server.Ai;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiSplineCursorTests
{
    [Test]
    public void CanAdvanceRejectsRouteShorterThanManeuverWithoutMutation()
    {
        var cursor = new AiSplineCursor(
            0,
            5,
            point => point == 0 ? 1 : null,
            _ => 20,
            (point, progress) => new AiSplinePose(
                new Vector3(point * 20 + progress, 0, 0),
                Vector3.UnitX));

        Assert.Multiple(() =>
        {
            Assert.That(cursor.CanAdvance(40), Is.False);
            Assert.That(cursor.PointId, Is.Zero);
            Assert.That(cursor.SegmentProgressMeters, Is.EqualTo(5));
        });
    }

    [Test]
    public void CanAdvanceAcceptsCompleteManeuverWithoutMutation()
    {
        var cursor = new AiSplineCursor(
            0,
            5,
            point => point < 3 ? point + 1 : null,
            _ => 20,
            (_, _) => new AiSplinePose(Vector3.Zero, Vector3.UnitX));

        Assert.That(cursor.CanAdvance(40), Is.True);
        Assert.That(cursor.PointId, Is.Zero);
        Assert.That(cursor.SegmentProgressMeters, Is.EqualTo(5));
    }

    [Test]
    public void MeasuresAvailableDistanceUpToRequestedLimitWithoutMutation()
    {
        var cursor = new AiSplineCursor(
            0,
            5,
            point => point == 0 ? 1 : null,
            _ => 20,
            (_, _) => new AiSplinePose(Vector3.Zero, Vector3.UnitX));

        var available = cursor.GetAvailableDistance(60);

        Assert.Multiple(() =>
        {
            Assert.That(available, Is.EqualTo(35));
            Assert.That(cursor.PointId, Is.Zero);
            Assert.That(cursor.SegmentProgressMeters, Is.EqualTo(5));
        });
    }

    [Test]
    public void AvailableDistanceIsCappedAtRequestedLimit()
    {
        var cursor = new AiSplineCursor(
            0,
            5,
            point => point < 10 ? point + 1 : null,
            _ => 20,
            (_, _) => new AiSplinePose(Vector3.Zero, Vector3.UnitX));

        Assert.That(cursor.GetAvailableDistance(60), Is.EqualTo(60));
    }
}
