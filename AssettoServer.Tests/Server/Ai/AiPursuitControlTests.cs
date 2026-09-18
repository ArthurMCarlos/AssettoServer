using AssettoServer.Server.Ai;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitControlTests
{
    [TestCase(1499, 1500, true)]
    [TestCase(1500, 1500, true)]
    [TestCase(1501, 1500, false)]
    public void RetentionUsesPursuitMaximumDistance(
        float distanceMeters,
        float maximumMeters,
        bool expected)
    {
        var actual = AiPursuitControl.ShouldRetain(
            distanceMeters * distanceMeters,
            maximumMeters);

        Assert.That(actual, Is.EqualTo(expected));
    }

    [Test]
    public void RequestedSpeedOverridesNativeTrafficSpeed()
    {
        var actual = AiPursuitControl.ResolveRequestedSpeed(
            80 / 3.6f,
            180 / 3.6f);

        Assert.That(actual, Is.EqualTo(180 / 3.6f).Within(0.001));
    }

    [Test]
    public void MissingRequestedSpeedPreservesNativeTrafficSpeed()
    {
        var actual = AiPursuitControl.ResolveRequestedSpeed(80 / 3.6f, null);

        Assert.That(actual, Is.EqualTo(80 / 3.6f).Within(0.001));
    }

    [Test]
    public void SafetyLimitCanReducePursuitRequest()
    {
        var actual = AiPursuitControl.ApplySafetyLimit(180 / 3.6f, 90 / 3.6f);

        Assert.That(actual, Is.EqualTo(90 / 3.6f).Within(0.001));
    }

    [TestCase(-1)]
    [TestCase(float.NaN)]
    [TestCase(float.PositiveInfinity)]
    public void RejectsInvalidDesiredSpeed(float speed)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            AiPursuitControl.ValidateDesiredSpeed(speed));
    }

    [TestCase(0)]
    [TestCase(-1)]
    [TestCase(float.NaN)]
    public void RejectsInvalidMaximumDistance(float maximumDistance)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            AiPursuitControl.ValidateMaximumDistance(maximumDistance));
    }
}
