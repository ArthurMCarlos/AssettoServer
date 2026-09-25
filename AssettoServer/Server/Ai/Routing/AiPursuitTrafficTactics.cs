using System;
using System.Collections.Generic;
using System.Linq;

namespace AssettoServer.Server.Ai.Routing;

internal enum AiTrafficTacticPhase { Follow, Bypass, Return }
internal sealed record AiTrafficTacticState(AiTrafficTacticPhase Phase,
    long? BlockedSinceMilliseconds, long? ObstacleIdentity, int? CommittedDestinationPoint);
internal sealed record AiTrafficLaneOption(AiPursuitLaneSelection Selection, float FreeDistanceMeters, bool SafeToEnter);
internal sealed record AiTrafficTacticRequest(
    long NowMilliseconds, bool BlockedByNonTarget, long? ObstacleIdentity,
    float SpeedDeficitMetersPerSecond, float CurrentFreeDistanceMeters,
    bool ObstaclePassedWithClearance, bool TargetLaneReached,
    bool PitBusy, bool JunctionHasPriority, bool TransitionCommitted,
    IReadOnlyList<AiTrafficLaneOption> Candidates,
    AiPursuitLaneSelection? ReturnSelection, AiTrafficTacticState Previous);
internal sealed record AiTrafficTacticDecision(AiTrafficTacticState State, AiPursuitLaneSelection? Selection);

internal static class AiPursuitTrafficTactics
{
    public static AiTrafficTacticState Initial => new(AiTrafficTacticPhase.Follow, null, null, null);

    public static AiTrafficTacticDecision Update(AiPursuitCloseOptions options, AiTrafficTacticRequest request)
    {
        var state = request.Previous;
        if (request.TransitionCommitted) return new(state, null);
        if (request.JunctionHasPriority) return new(Initial, null);
        if (request.PitBusy) return new(state with { BlockedSinceMilliseconds = null }, null);
        if (state.Phase != AiTrafficTacticPhase.Follow)
        {
            if (request.TargetLaneReached && (state.Phase == AiTrafficTacticPhase.Return || request.ObstaclePassedWithClearance))
                return new(Initial, null);
            if (state.Phase == AiTrafficTacticPhase.Bypass && !request.ObstaclePassedWithClearance)
                return new(state, null);
            var selection = request.ReturnSelection;
            return selection == null ? new(state, null)
                : new(state with { Phase = AiTrafficTacticPhase.Return, CommittedDestinationPoint = selection.ToPointId },
                    selection with { Motivation = AiPursuitLaneMotivation.TrafficReturn });
        }
        if (!request.BlockedByNonTarget || request.ObstacleIdentity == null
            || !float.IsFinite(request.SpeedDeficitMetersPerSecond)
            || request.SpeedDeficitMetersPerSecond < options.ObstacleDeficitMetersPerSecond)
            return new(Initial, null);
        if (state.ObstacleIdentity != request.ObstacleIdentity || !state.BlockedSinceMilliseconds.HasValue
            || request.NowMilliseconds < state.BlockedSinceMilliseconds)
            state = new(AiTrafficTacticPhase.Follow, request.NowMilliseconds, request.ObstacleIdentity, null);
        if (request.NowMilliseconds - state.BlockedSinceMilliseconds < options.ObstacleHoldMilliseconds)
            return new(state, null);
        var best = request.Candidates.Take(2)
            .Where(c => c.SafeToEnter && float.IsFinite(c.FreeDistanceMeters)
                        && c.FreeDistanceMeters > request.CurrentFreeDistanceMeters
                        && c.Selection.PhysicalRelation is AiPursuitLanePhysicalRelation.ImmediateLeft or AiPursuitLanePhysicalRelation.ImmediateRight)
            .OrderBy(c => c.Selection.DestinationPlan.DistanceMeters)
            .ThenByDescending(c => c.FreeDistanceMeters)
            .ThenBy(c => c.Selection.Direction).FirstOrDefault();
        return best == null ? new(state, null)
            : new(state with { Phase = AiTrafficTacticPhase.Bypass, CommittedDestinationPoint = best.Selection.ToPointId },
                best.Selection with { Motivation = AiPursuitLaneMotivation.TrafficBypass });
    }
}
