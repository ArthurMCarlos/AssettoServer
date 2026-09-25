namespace AssettoServer.Server.Ai.Routing;

// Preparation and native safety polling can run separately. Retain intention until the real event.
internal sealed class AiTrafficTacticCommit
{
    private AiTrafficTacticDecision? _decision;
    private AiLaneChangeEvent? _prepared;
    public void Clear() { _decision = null; _prepared = null; }
    public void Stage(AiTrafficTacticDecision decision, AiLaneChangeEvent prepared)
    {
        _decision = decision;
        _prepared = prepared;
    }
    public AiTrafficTacticDecision? Observe(AiLaneChangeEvent? current)
    {
        if (_prepared == null || current == null) return null;
        if (current.FromPointId != _prepared.FromPointId || current.ToPointId != _prepared.ToPointId
            || current.RouteRevision != _prepared.RouteRevision || current.Motivation != _prepared.Motivation
            || current.Kind == AiPursuitLaneChangeEventKind.Cancelled)
        { Clear(); return null; }
        if (current.Kind is not (AiPursuitLaneChangeEventKind.Started or AiPursuitLaneChangeEventKind.Completed)) return null;
        var result = _decision;
        Clear();
        return result;
    }
}
