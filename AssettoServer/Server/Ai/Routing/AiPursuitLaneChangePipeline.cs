using System;
using System.Collections.Generic;

namespace AssettoServer.Server.Ai.Routing;

internal sealed record AiPursuitLanePreparationResult(
    AiPursuitNavigationResult EffectiveNavigation,
    AiPursuitLaneSelectionResult Evaluation,
    bool RequestPrepared,
    AiLaneChangePhase ControllerPhase,
    AiLaneChangeEvent? ControllerEvent);

internal sealed class AiPursuitLaneChangePipeline
{
    private readonly AiPursuitLaneSelector _selector;
    private readonly AiLaneChangeController _controller;
    private readonly Func<int, float, AiSplineCursor> _createCursor;
    private readonly Func<int, float> _getSegmentLength;

    public AiPursuitLaneChangePipeline(
        AiPursuitLaneSelector selector,
        AiLaneChangeController controller,
        Func<int, float, AiSplineCursor> createCursor,
        Func<int, float> getSegmentLength)
    {
        _selector = selector ?? throw new ArgumentNullException(nameof(selector));
        _controller = controller ?? throw new ArgumentNullException(nameof(controller));
        _createCursor = createCursor ?? throw new ArgumentNullException(nameof(createCursor));
        _getSegmentLength = getSegmentLength
            ?? throw new ArgumentNullException(nameof(getSegmentLength));
    }

    public AiPursuitLanePreparationResult Evaluate(
        int currentPointId,
        float currentProgressMeters,
        float currentSegmentLengthMeters,
        AiPursuitNavigationResult navigation,
        AiPursuitRouteState? previousNavigation,
        AiPursuitLaneChangeOptions options,
        AiRouteSearchLimits limits,
        long nowMilliseconds)
    {
        ArgumentNullException.ThrowIfNull(navigation);
        ArgumentNullException.ThrowIfNull(options);
        ArgumentNullException.ThrowIfNull(limits);
        if (!navigation.PreferredPhysicalTargetPointId.HasValue)
            throw new ArgumentException("Physical target anchor is required", nameof(navigation));

        if (_controller.TryGetCommittedRoute(out var committed, out var committedRevision))
        {
            return Result(
                CreateEffectiveNavigation(navigation, committed, committedRevision, false),
                _selector.Select(
                    currentPointId,
                    new HashSet<int> { navigation.PreferredPhysicalTargetPointId.Value },
                    limits,
                    options.DistanceMeters,
                    options.LookaheadMeters),
                false);
        }

        var physicalTargets = new HashSet<int>
        {
            navigation.PreferredPhysicalTargetPointId.Value
        };
        var evaluation = _selector.Select(
            currentPointId,
            physicalTargets,
            limits,
            options.DistanceMeters,
            options.LookaheadMeters);
        if (evaluation.Kind != AiPursuitLaneSelectionKind.Change
            || evaluation.Selection == null)
        {
            _controller.ReconcileCurrentRoute(
                navigation.State?.Revision ?? previousNavigation?.Revision ?? 0);
            return Result(navigation, evaluation, false);
        }

        var selection = evaluation.Selection;
        var sourceCursor = _createCursor(currentPointId, currentProgressMeters);
        var destinationLength = _getSegmentLength(selection.ToPointId);
        var destinationProgress = currentSegmentLengthMeters > 0
            ? destinationLength * (currentProgressMeters / currentSegmentLengthMeters)
            : 0;
        var destinationCursor = _createCursor(selection.ToPointId, destinationProgress);
        if (selection.DistanceToDecisionMeters - destinationProgress < options.DistanceMeters
            || !sourceCursor.CanAdvance(options.DistanceMeters)
            || !destinationCursor.CanAdvance(options.DistanceMeters))
        {
            return Result(navigation, evaluation, false);
        }

        var sameRoute = previousNavigation != null
                        && HasSameRoute(previousNavigation.Plan, selection.DestinationPlan);
        var revision = sameRoute
            ? previousNavigation!.Revision
            : Math.Max(
                  navigation.State?.Revision ?? 0,
                  previousNavigation?.Revision ?? 0) + 1;
        var prepared = _controller.Prepare(
            selection,
            sourceCursor,
            destinationCursor,
            nowMilliseconds,
            revision);
        if (!prepared)
            return Result(navigation, evaluation, false);

        return Result(
            CreateEffectiveNavigation(navigation, selection, revision, sameRoute),
            evaluation,
            true);
    }

    private AiPursuitLanePreparationResult Result(
        AiPursuitNavigationResult navigation,
        AiPursuitLaneSelectionResult evaluation,
        bool requestPrepared) =>
        new(
            navigation,
            evaluation,
            requestPrepared,
            _controller.Phase,
            _controller.Event);

    private static AiPursuitNavigationResult CreateEffectiveNavigation(
        AiPursuitNavigationResult navigation,
        AiPursuitLaneSelection selection,
        long revision,
        bool sameRoute)
    {
        var state = new AiPursuitRouteState(
            selection.DestinationPlan.Nodes[^1].PointId,
            selection.DestinationPlan,
            revision,
            null)
        {
            PreferredPhysicalTargetPointId = navigation.PreferredPhysicalTargetPointId
        };
        return navigation with
        {
            Status = AiPursuitNavigationStatus.Active,
            State = state,
            UpdateKind = sameRoute
                ? AiPursuitRouteUpdateKind.Reused
                : AiPursuitRouteUpdateKind.Recalculated,
            SearchFailure = AiRouteSearchFailure.None
        };
    }

    private static bool HasSameRoute(AiRoutePlan left, AiRoutePlan right)
    {
        if (left.Nodes[^1].PointId != right.Nodes[^1].PointId
            || left.JunctionDecisions.Count != right.JunctionDecisions.Count)
        {
            return false;
        }

        foreach (var decision in left.JunctionDecisions)
        {
            if (!right.JunctionDecisions.TryGetValue(decision.Key, out var takeBranch)
                || takeBranch != decision.Value)
            {
                return false;
            }
        }
        return true;
    }
}
