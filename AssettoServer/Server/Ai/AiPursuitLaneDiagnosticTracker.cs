using System;
using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Server.Ai;

public sealed class AiPursuitLaneDiagnosticTracker
{
    private SemanticKey? _lastKey;
    private long _revision;

    public AiPursuitLaneChangeDiagnostics? PublishEvaluation(
        int policePointId,
        int preferredPhysicalTargetPointId,
        long routeRevision,
        AiPursuitLaneSelectionResult evaluation)
    {
        ArgumentNullException.ThrowIfNull(evaluation);
        var selection = evaluation.Selection;
        var reason = MapReason(evaluation.Reason);
        var key = new SemanticKey(
            reason,
            policePointId,
            preferredPhysicalTargetPointId,
            selection?.FromPointId,
            selection?.ToPointId,
            selection?.Direction,
            selection?.JunctionId,
            null,
            AiPursuitLaneChangeEventKind.Evaluated);
        if (_lastKey == key)
            return null;

        _lastKey = key;
        return new AiPursuitLaneChangeDiagnostics(
            ++_revision,
            AiPursuitLaneChangeEventKind.Evaluated,
            selection?.FromPointId ?? policePointId,
            selection?.ToPointId ?? policePointId,
            selection?.Direction ?? AiLaneChangeDirection.Left,
            routeRevision,
            selection?.DistanceToDecisionMeters,
            null)
        {
            Reason = reason,
            PolicePointId = policePointId,
            PreferredPhysicalTargetPointId = preferredPhysicalTargetPointId,
            JunctionId = selection?.JunctionId,
            CurrentLaneRoute = evaluation.CurrentLaneRoute,
            CandidateLaneRoutes = evaluation.CandidateLaneRoutes
        };
    }

    public void Reset()
    {
        _lastKey = null;
        _revision = 0;
    }

    private static AiPursuitLaneChangeDiagnosticReason MapReason(
        AiPursuitLaneEvaluationReason reason) =>
        reason switch
        {
            AiPursuitLaneEvaluationReason.CurrentLaneValid =>
                AiPursuitLaneChangeDiagnosticReason.CurrentLaneValid,
            AiPursuitLaneEvaluationReason.NoAdjacentLane =>
                AiPursuitLaneChangeDiagnosticReason.NoAdjacentLane,
            AiPursuitLaneEvaluationReason.OppositeDirection =>
                AiPursuitLaneChangeDiagnosticReason.OppositeDirection,
            AiPursuitLaneEvaluationReason.NoForwardRoute =>
                AiPursuitLaneChangeDiagnosticReason.NoForwardRoute,
            AiPursuitLaneEvaluationReason.NoRealJunction =>
                AiPursuitLaneChangeDiagnosticReason.NoRealJunction,
            AiPursuitLaneEvaluationReason.BeyondLookahead =>
                AiPursuitLaneChangeDiagnosticReason.BeyondLookahead,
            AiPursuitLaneEvaluationReason.InsufficientPreparationDistance =>
                AiPursuitLaneChangeDiagnosticReason.InsufficientPreparationDistance,
            AiPursuitLaneEvaluationReason.RoutePreparation =>
                AiPursuitLaneChangeDiagnosticReason.RoutePreparation,
            _ => throw new ArgumentOutOfRangeException(nameof(reason), reason, null)
        };

    private sealed record SemanticKey(
        AiPursuitLaneChangeDiagnosticReason Reason,
        int PolicePointId,
        int? PreferredPhysicalTargetPointId,
        int? FromPointId,
        int? ToPointId,
        AiLaneChangeDirection? Direction,
        int? JunctionId,
        AiLaneChangeSafetyStatus? SafetyStatus,
        AiPursuitLaneChangeEventKind EventKind);
}
