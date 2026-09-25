using AssettoServer.Server.Ai;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitEscapePolicyTests
{
    [Test]
    public void RequiresFifteenContinuousSecondsBeyondEightHundred()
    {
        var first = AiPursuitEscapePolicy.Update(801, true, 1000, AiPursuitCloseOptionsTests.Defaults, new(null));
        Assert.That(AiPursuitEscapePolicy.Update(801, true, 15999, AiPursuitCloseOptionsTests.Defaults, first.State).Escaped, Is.False);
        Assert.That(AiPursuitEscapePolicy.Update(801, true, 16000, AiPursuitCloseOptionsTests.Defaults, first.State).Escaped, Is.True);
    }

    [TestCase(800, true)]
    [TestCase(float.NaN, true)]
    [TestCase(float.PositiveInfinity, true)]
    [TestCase(801, false)]
    public void LostContinuityRestartsTimer(float distance, bool navigation)
    {
        var reset = AiPursuitEscapePolicy.Update(distance, navigation, 15000,
            AiPursuitCloseOptionsTests.Defaults, new(1000));
        Assert.That(reset.State.FarSinceMilliseconds, Is.Null);
        Assert.That(reset.Escaped, Is.False);
    }
}
