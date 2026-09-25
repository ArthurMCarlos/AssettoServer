using System.Threading;

namespace AssettoServer.Server.Ai;

// Kept outside the pursuit snapshot because native reposition resets that snapshot before plugin polling.
internal sealed class AiPursuitTerminalDelivery
{
    private sealed record Terminal(byte SessionId, object? Connection);
    private Terminal? _pending;

    public void CaptureHardCut(byte session, object? connection, float distanceSquared, float maximumDistance)
    {
        if (float.IsFinite(distanceSquared) && distanceSquared >= 0
            && float.IsFinite(maximumDistance) && maximumDistance > 0
            && !AiPursuitControl.ShouldRetain(distanceSquared, maximumDistance))
            Interlocked.Exchange(ref _pending, new(session, connection));
    }

    public AiPursuitTrackingStatus? Take(byte session, object? connection)
    {
        var pending = Interlocked.Exchange(ref _pending, null);
        return pending?.SessionId == session && ReferenceEquals(pending.Connection, connection)
            ? AiPursuitTrackingStatus.MaxDistanceExceeded : null;
    }
}
