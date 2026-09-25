using AssettoServer.Server.Ai;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitCloseOptionsTests
{
    internal static AiPursuitCloseOptions Defaults => new(
        60, 250, 80 / 3.6f, 10, 5, 400 / 3.6f, 1000, 10 / 3.6f,
        150, 12, 800, 15000, 60000);

    internal static AiPursuitTrackingOptions Tracking => new(
        1500, 20000, 50000, 2000, new(true, 60, 3000),
        new(true, true, 100, 15, 3, 50, 10, 1)) { ClosePursuit = Defaults };

    [Test]
    public void AcceptsClosePursuitWithoutPitAndPreservesLegacyWithoutIt()
    {
        Assert.DoesNotThrow(() => AiPursuitControl.ValidateTrackingOptions(Tracking));
        Assert.DoesNotThrow(() => AiPursuitControl.ValidateTrackingOptions(
            Tracking with { ClosePursuit = null, Driving = null, LaneChange = null }));
    }

    static IEnumerable<AiPursuitCloseOptions> Invalid()
    {
        foreach (var value in new[] { 0f, -1f, float.NaN, float.PositiveInfinity })
        {
            yield return Defaults with { AssistStartMeters = value };
            yield return Defaults with { AssistFullMeters = value };
            yield return Defaults with { MaxAdvantageMetersPerSecond = value };
            yield return Defaults with { MaxAccelerationMetersPerSecondSquared = value };
            yield return Defaults with { MaxJerkMetersPerSecondCubed = value };
            yield return Defaults with { MaxSpeedMetersPerSecond = value };
            yield return Defaults with { ObstacleDeficitMetersPerSecond = value };
            yield return Defaults with { BypassLookaheadMeters = value };
            yield return Defaults with { ReturnClearanceMeters = value };
            yield return Defaults with { EscapeDistanceMeters = value };
        }
        yield return Defaults with { AssistFullMeters = 60 };
        yield return Defaults with { AssistStartMeters = 15 };
        yield return Defaults with { EscapeDistanceMeters = 250 };
        yield return Defaults with { EscapeDistanceMeters = 1500 };
        yield return Defaults with { BypassLookaheadMeters = 59 };
        yield return Defaults with { MaxSpeedMetersPerSecond = 49 };
        yield return Defaults with { ObstacleHoldMilliseconds = 0 };
        yield return Defaults with { EscapeHoldMilliseconds = 0 };
        yield return Defaults with { RearmDelayMilliseconds = 0 };
    }

    [TestCaseSource(nameof(Invalid))]
    public void RejectsUnsafePublicCloseOptions(AiPursuitCloseOptions close) =>
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            AiPursuitControl.ValidateTrackingOptions(Tracking with { ClosePursuit = close }));

    [Test]
    public void RequiresDrivingAndLaneChange()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => AiPursuitControl.ValidateTrackingOptions(
            Tracking with { Driving = null }));
        Assert.Throws<ArgumentOutOfRangeException>(() => AiPursuitControl.ValidateTrackingOptions(
            Tracking with { LaneChange = new(false, 60, 3000) }));
    }
}
