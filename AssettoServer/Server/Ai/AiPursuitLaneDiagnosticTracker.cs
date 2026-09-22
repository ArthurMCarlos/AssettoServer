using System;
using System.Linq;
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
        var candidate = selection == null
            ? evaluation.CandidateLaneRoutes.FirstOrDefault(route =>
                  route.Reason == evaluation.Reason)
              ?? evaluation.CandidateLaneRoutes.FirstOrDefault()
            : null;
        var reason = MapReason(evaluation.Reason);
        var key = new SemanticKey(
            reason,
            preferredPhysicalTargetPointId,
            selection?.FromPointId,
            selection?.ToPointId ?? candidate?.PointId,
            selection?.Direction ?? candidate?.Direction,
            selection?.JunctionId ?? candidate?.JunctionId,
            selection?.Motivation ?? candidate?.Motivation,
            selection?.PhysicalRelation ?? candidate?.PhysicalRelation,
            null,
            AiPursuitLaneChangeEventKind.Evaluated,
            CreateCandidateEvidence(evaluation));
        if (_lastKey == key)
            return null;

        _lastKey = key;
        return new AiPursuitLaneChangeDiagnostics(
            ++_revision,
            AiPursuitLaneChangeEventKind.Evaluated,
            selection?.FromPointId,
            selection?.ToPointId ?? candidate?.PointId,
            selection?.Direction ?? candidate?.Direction,
            routeRevision,
            selection?.DistanceToDecisionMeters ?? candidate?.DistanceToDecisionMeters)
        {
            Reason = reason,
            PolicePointId = policePointId,
            PreferredPhysicalTargetPointId = preferredPhysicalTargetPointId,
            JunctionId = selection?.JunctionId ?? candidate?.JunctionId,
            Motivation = selection?.Motivation ?? candidate?.Motivation,
            PhysicalRelation = selection?.PhysicalRelation ?? candidate?.PhysicalRelation,
            CurrentLaneRoute = evaluation.CurrentLaneRoute,
            CandidateLaneRoutes = evaluation.CandidateLaneRoutes,
            RequiredTransitionDistanceMeters = evaluation.RequiredTransitionDistanceMeters,
            SourceAvailableDistanceMeters = evaluation.SourceAvailableDistanceMeters,
            DestinationAvailableDistanceMeters = evaluation.DestinationAvailableDistanceMeters
        };
    }

    public AiPursuitLaneChangeDiagnostics? PublishGate(
        int policePointId,
        int? preferredPhysicalTargetPointId,
        long routeRevision,
        AiPursuitLaneChangeDiagnosticReason reason)
    {
        if (reason is not (AiPursuitLaneChangeDiagnosticReason.Disabled
            or AiPursuitLaneChangeDiagnosticReason.NoPhysicalTarget))
        {
            throw new ArgumentOutOfRangeException(nameof(reason));
        }

        var key = new SemanticKey(
            reason,
            preferredPhysicalTargetPointId,
            null,
            null,
            null,
            null,
            null,
            null,
            null,
            AiPursuitLaneChangeEventKind.Evaluated);
        if (_lastKey == key)
            return null;

        _lastKey = key;
        return new AiPursuitLaneChangeDiagnostics(
            ++_revision,
            AiPursuitLaneChangeEventKind.Evaluated,
            null,
            null,
            null,
            routeRevision,
            null)
        {
            Reason = reason,
            PolicePointId = policePointId,
            PreferredPhysicalTargetPointId = preferredPhysicalTargetPointId
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
            AiPursuitLaneEvaluationReason.NonAdjacent =>
                AiPursuitLaneChangeDiagnosticReason.NonAdjacent,
            AiPursuitLaneEvaluationReason.OppositeDirection =>
                AiPursuitLaneChangeDiagnosticReason.OppositeDirection,
            AiPursuitLaneEvaluationReason.InvalidGeometry =>
                AiPursuitLaneChangeDiagnosticReason.InvalidGeometry,
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
            AiPursuitLaneEvaluationReason.Cooldown =>
                AiPursuitLaneChangeDiagnosticReason.Cooldown,
            _ => throw new ArgumentOutOfRangeException(nameof(reason), reason, null)
        };

    private static string CreateCandidateEvidence(
        AiPursuitLaneSelectionResult evaluation) =>
        string.Join("|", evaluation.CandidateLaneRoutes
            .OrderBy(candidate => candidate.Direction)
            .ThenBy(candidate => candidate.PointId)
            .Select(candidate =>
                $"{candidate.PointId}:{candidate.Direction}:{candidate.SearchFailure}:" +
                $"{candidate.JunctionId}:{candidate.Reason}:{candidate.PhysicalRelation}"));

    private sealed record SemanticKey(
        AiPursuitLaneChangeDiagnosticReason Reason,
        int? PreferredPhysicalTargetPointId,
        int? FromPointId,
        int? ToPointId,
        AiLaneChangeDirection? Direction,
        int? JunctionId,
        AiPursuitLaneMotivation? Motivation,
        AiPursuitLanePhysicalRelation? PhysicalRelation,
        AiLaneChangeSafetyStatus? SafetyStatus,
        AiPursuitLaneChangeEventKind EventKind,
        string CandidateEvidence = "");
}
