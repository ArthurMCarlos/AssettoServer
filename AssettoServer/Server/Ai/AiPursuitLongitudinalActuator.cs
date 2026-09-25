using System;

namespace AssettoServer.Server.Ai;

// Used only by the existing movement writer; keeps the positive ramp across obstacle polls.
internal sealed class AiPursuitLongitudinalActuator
{
    private float? _positiveAcceleration;
    private float _targetSpeed;
    private float _nativeAcceleration;
    public void Reset() => _positiveAcceleration = null;

    public void Command(float targetSpeed, float acceleration)
    {
        _targetSpeed = targetSpeed;
        _nativeAcceleration = acceleration;
    }

    public (float Speed, float Acceleration) Step(float speed,
        AiPursuitCloseOptions? options, AiRecoveryAssistDecision? assist, float dt)
    {
        float targetSpeed = _targetSpeed;
        float nativeAcceleration = _nativeAcceleration;
        dt = float.IsFinite(dt) ? Math.Max(0, dt) : 0;
        float acceleration = nativeAcceleration;
        if (options != null && assist is { Active: true } && acceleration > 0 && speed < targetSpeed)
        {
            acceleration = AiPursuitRecoveryAssist.StepAcceleration(
                _positiveAcceleration ?? Math.Min(acceleration, assist.AccelerationLimit),
                assist.AccelerationLimit, options.MaxJerkMetersPerSecondCubed, dt);
            _positiveAcceleration = acceleration;
        }
        else Reset();
        float nextSpeed = (float)Math.Clamp((double)speed + acceleration * (double)dt, 0, float.MaxValue);
        if ((acceleration < 0 && nextSpeed < targetSpeed) || (acceleration > 0 && nextSpeed > targetSpeed))
            return (targetSpeed, 0);
        return (nextSpeed, acceleration);
    }
}
