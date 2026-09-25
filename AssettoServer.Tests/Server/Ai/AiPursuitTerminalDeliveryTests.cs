using AssettoServer.Server.Ai;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitTerminalDeliveryTests
{
    [TestCase(float.NaN)]
    [TestCase(float.PositiveInfinity)]
    [TestCase(-1)]
    public void InvalidMeasurementCannotPenalizeConnection(float distanceSquared)
    {
        var delivery = new AiPursuitTerminalDelivery();
        var connection = new object();
        delivery.CaptureHardCut(10, connection, distanceSquared, 1500);
        Assert.That(delivery.Take(10, connection), Is.Null);
    }

    [Test]
    public void NativeRepositionPublishesHardCutBeforeTrackingAndDeliversOnlyOnce()
    {
        var delivery = new AiPursuitTerminalDelivery();
        var connection = new object();
        Assert.That(AiPursuitControl.ShouldRetain(1501 * 1501, 1500), Is.False);
        delivery.CaptureHardCut(10, connection, 1501 * 1501, 1500);
        // Movement may now reset/despawn; delivery owns the terminal separately from pursuit state.
        Assert.That(delivery.Take(10, connection), Is.EqualTo(AiPursuitTrackingStatus.MaxDistanceExceeded));
        Assert.That(delivery.Take(10, connection), Is.Null);
    }

    [Test]
    public void SlotReuseCannotReceiveOldConnectionsTerminal()
    {
        var delivery = new AiPursuitTerminalDelivery();
        delivery.CaptureHardCut(10, new object(), 1600 * 1600, 1500);
        Assert.That(delivery.Take(10, new object()), Is.Null);
        delivery.CaptureHardCut(10, null, 100, 1500);
        Assert.That(delivery.Take(10, null), Is.Null);
    }
}
