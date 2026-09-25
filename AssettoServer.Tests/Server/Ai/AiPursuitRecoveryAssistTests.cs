using AssettoServer.Server.Ai;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitRecoveryAssistTests
{
    static AiRecoveryAssistInput Input => new(400, 400, 250 / 3.6f, 60, 250 / 3.6f, 2, 8, true, false);

    [TestCase(200, 280)]
    [TestCase(250, 330)]
    [TestCase(300, 380)]
    [TestCase(390, 400)]
    public void RecoversFastTargetsWithoutExceedingTechnicalCap(float target, float expected)
    {
        var result = AiPursuitRecoveryAssist.Evaluate(AiPursuitCloseOptionsTests.Defaults,
            Input with { TargetSpeedMetersPerSecond = target / 3.6f, BaseRequestedSpeedMetersPerSecond = target / 3.6f });
        Assert.That(result.RequestedSpeedMetersPerSecond * 3.6f, Is.EqualTo(expected).Within(.01));
        Assert.That(result.AccelerationLimit, Is.EqualTo(10));
    }

    [Test]
    public void BrakesBeforeEnteringUnassistedRange()
    {
        var result = AiPursuitRecoveryAssist.Evaluate(AiPursuitCloseOptionsTests.Defaults,
            Input with { PhysicalClearanceMeters = 300, TargetSpeedMetersPerSecond = 300 / 3.6f,
                BaseRequestedSpeedMetersPerSecond = 300 / 3.6f });
        Assert.That(result.RequestedSpeedMetersPerSecond, Is.EqualTo(103.848f).Within(.01));
    }

    static IEnumerable<AiRecoveryAssistInput> Inactive()
    {
        yield return Input with { PhysicalClearanceMeters = 10, RouteDistanceMeters = 1000 };
        yield return Input with { RecoveryActive = true };
        yield return Input with { NavigationActive = false };
        yield return Input with { PhysicalClearanceMeters = float.NaN };
        yield return Input with { PoliceSpeedMetersPerSecond = float.PositiveInfinity };
        yield return Input with { TargetSpeedMetersPerSecond = float.NaN };
        yield return Input with { Deceleration = 0 };
    }

    [TestCaseSource(nameof(Inactive))]
    public void InvalidUnavailableOrNearNeverGetsRecoveryBoost(AiRecoveryAssistInput input)
    {
        var result = AiPursuitRecoveryAssist.Evaluate(AiPursuitCloseOptionsTests.Defaults, input);
        Assert.That(result.Active, Is.False);
        Assert.That(result.RequestedSpeedMetersPerSecond, Is.EqualTo(Input.BaseRequestedSpeedMetersPerSecond));
    }

    [TestCase(.2f, 3)]
    [TestCase(.1f, 2.5f)]
    [TestCase(0, 2)]
    [TestCase(-1, 2)]
    [TestCase(100, 10)]
    public void JerkUsesElapsedTimeAndClampsAtDesired(float dt, float expected) =>
        Assert.That(AiPursuitRecoveryAssist.StepAcceleration(2, 10, 5, dt), Is.EqualTo(expected));

    [Test]
    public void EmergencyBrakingDoesNotWaitForPositiveRamp() =>
        Assert.That(AiPursuitRecoveryAssist.StepAcceleration(10, -8, 5, .1f), Is.EqualTo(-8));

    [TestCase(100, 50, 50)]
    [TestCase(100, 0, 0)]
    [TestCase(20, 50, 20)]
    public void NavigationLossCannotRetainPreviousRecoverySpeed(float previous, float target, float expected) =>
        Assert.That(AiPursuitRecoveryAssist.UnavailableSpeed(previous, target), Is.EqualTo(expected));
}
