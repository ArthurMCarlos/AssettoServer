using AssettoServer.Server.Ai;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitRecoveryAssistIntegrationTests
{
    [Test]
    public void ApproachEnvelopeCanActuallyBrakeWithDefaultHalfForce()
    {
        var input = new AiRecoveryAssistInput(300, 300, 0, 70, 0, 2, 8, true, false)
        { CorneringBrakeForceFactor = .5f };
        var options = AiPursuitCloseOptionsTests.Defaults with { MaxAdvantageMetersPerSecond = 100 };
        var decision = AiPursuitRecoveryAssist.Evaluate(options, input);
        Assert.That(decision.RequestedSpeedMetersPerSecond, Is.EqualTo(43.8178f).Within(.001));
        var actuator = new AiPursuitLongitudinalActuator();
        actuator.Command(0, -4);
        float speed = decision.RequestedSpeedMetersPerSecond;
        float traveled = 0;
        for (int i = 0; i < 120 && speed > 0; i++)
        {
            speed = actuator.Step(speed, options, null, .1f).Speed;
            traveled += speed * .1f;
        }
        Assert.That(speed, Is.Zero);
        Assert.That(traveled, Is.LessThanOrEqualTo(240));
    }

    [Test]
    public void DisablingAssistBetweenObstaclePollsImmediatelyRestoresNativeCommand()
    {
        var actuator = new AiPursuitLongitudinalActuator();
        actuator.Command(100, 2);
        var boosted = actuator.Step(60, AiPursuitCloseOptionsTests.Defaults, new(100, 10, true), 2);
        Assert.That(boosted.Acceleration, Is.EqualTo(10));
        var unassisted = actuator.Step(boosted.Speed, AiPursuitCloseOptionsTests.Defaults, null, .1f);
        Assert.That(unassisted.Acceleration, Is.EqualTo(2));
        Assert.That(unassisted.Speed, Is.EqualTo(80.2f).Within(.001));
    }

    [Test]
    public void MovementUsesAssistedAccelerationButOrdinaryMovementStaysNative()
    {
        var actuator = new AiPursuitLongitudinalActuator();
        var assist = AiPursuitRecoveryAssist.Evaluate(AiPursuitCloseOptionsTests.Defaults,
            new(400, 400, 70, 60, 70, 2, 8, true, false));
        actuator.Command(90, 2);
        var ordinary = actuator.Step(60, null, null, .2f);
        Assert.That(ordinary.Speed, Is.EqualTo(60.4f).Within(.001));
        var first = actuator.Step(60, AiPursuitCloseOptionsTests.Defaults, assist, .2f);
        Assert.That(first.Acceleration, Is.EqualTo(3));
        Assert.That(first.Speed, Is.EqualTo(60.6f).Within(.001));
        var second = actuator.Step(first.Speed, AiPursuitCloseOptionsTests.Defaults, assist, .1f);
        Assert.That(second.Acceleration, Is.EqualTo(3.5f));
        Assert.That(second.Speed, Is.EqualTo(60.95f).Within(.001));
        actuator.Reset();
        Assert.That(actuator.Step(60, AiPursuitCloseOptionsTests.Defaults, assist, .1f).Acceleration, Is.EqualTo(2.5f));
    }

    [TestCase(20)]
    [TestCase(0)]
    public void SafetyLimitBrakesImmediatelyDespiteActiveBoost(float safetySpeed)
    {
        var actuator = new AiPursuitLongitudinalActuator();
        var assist = new AiRecoveryAssistDecision(100, 10, true);
        actuator.Command(100, 2);
        actuator.Step(60, AiPursuitCloseOptionsTests.Defaults, assist, 1);
        actuator.Command(safetySpeed, -8);
        var step = actuator.Step(60, AiPursuitCloseOptionsTests.Defaults, assist, .2f);
        Assert.That(step.Acceleration, Is.EqualTo(-8));
        Assert.That(step.Speed, Is.EqualTo(58.4f).Within(.001));
    }

    [TestCase(0)]
    [TestCase(-1)]
    public void NonPositiveTimeCannotMoveBackward(float dt)
    {
        var actuator = new AiPursuitLongitudinalActuator();
        actuator.Command(100, 2);
        var result = actuator.Step(60, AiPursuitCloseOptionsTests.Defaults, new(100, 10, true), dt);
        Assert.That(result.Speed, Is.EqualTo(60));
    }
}
