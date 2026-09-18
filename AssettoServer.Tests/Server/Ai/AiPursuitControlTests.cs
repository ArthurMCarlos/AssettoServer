using AssettoServer.Server.Ai;
using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitControlTests
{
    [Test]
    public void RejectsInvalidLaneChangeDistance()
    {
        var options = new AiPursuitTrackingOptions(
            1500,
            20_000,
            50_000,
            2000,
            new AiPursuitLaneChangeOptions(true, 0, 3000));

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            AiPursuitControl.ValidateTrackingOptions(options));
    }

    [TestCase(1499, 1500, true)]
    [TestCase(1500, 1500, true)]
    [TestCase(1501, 1500, false)]
    public void RetentionUsesPursuitMaximumDistance(
        float distanceMeters,
        float maximumMeters,
        bool expected)
    {
        var actual = AiPursuitControl.ShouldRetain(
            distanceMeters * distanceMeters,
            maximumMeters);

        Assert.That(actual, Is.EqualTo(expected));
    }

    [Test]
    public void RequestedSpeedOverridesNativeTrafficSpeed()
    {
        var actual = AiPursuitControl.ResolveRequestedSpeed(
            80 / 3.6f,
            180 / 3.6f);

        Assert.That(actual, Is.EqualTo(180 / 3.6f).Within(0.001));
    }

    [Test]
    public void MissingRequestedSpeedPreservesNativeTrafficSpeed()
    {
        var actual = AiPursuitControl.ResolveRequestedSpeed(80 / 3.6f, null);

        Assert.That(actual, Is.EqualTo(80 / 3.6f).Within(0.001));
    }

    [Test]
    public void SafetyLimitCanReducePursuitRequest()
    {
        var actual = AiPursuitControl.ApplySafetyLimit(180 / 3.6f, 90 / 3.6f);

        Assert.That(actual, Is.EqualTo(90 / 3.6f).Within(0.001));
    }

    [TestCase(-1)]
    [TestCase(float.NaN)]
    [TestCase(float.PositiveInfinity)]
    public void RejectsInvalidDesiredSpeed(float speed)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            AiPursuitControl.ValidateDesiredSpeed(speed));
    }

    [TestCase(0)]
    [TestCase(-1)]
    [TestCase(float.NaN)]
    public void RejectsInvalidMaximumDistance(float maximumDistance)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            AiPursuitControl.ValidateMaximumDistance(maximumDistance));
    }

    [Test]
    public void RejectsInvalidTrackingRouteBudget()
    {
        var options = new AiPursuitTrackingOptions(
            MaximumSpatialDistanceMeters: 1500,
            MaximumRouteDistanceMeters: 20_000,
            MaximumVisitedNodes: 0,
            RouteGraceMilliseconds: 2000);

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            AiPursuitControl.ValidateTrackingOptions(options));
    }

    [Test]
    public void CreatesDiagnosticsWithRealJunctionDestination()
    {
        var plan = new AiRoutePlan(
            30,
            [new AiRouteNode(10, 0), new AiRouteNode(20, 30)],
            new Dictionary<int, bool> { [7] = true });
        var navigation = new AiPursuitNavigationResult(
            AiPursuitNavigationStatus.Active,
            new AiPursuitRouteState(20, plan, 3, null),
            AiPursuitRouteUpdateKind.Recalculated,
            AiRouteSearchFailure.None,
            12,
            new AiPursuitTargetLocationDiagnostics([], [], []),
            30,
            1);

        var diagnostics = AiPursuitControl.CreateRouteDiagnostics(
            navigation,
            policePointId: 10,
            junctionId => junctionId == 7 ? 300 : -1);

        Assert.Multiple(() =>
        {
            Assert.That(diagnostics.Revision, Is.EqualTo(3));
            Assert.That(diagnostics.PolicePointId, Is.EqualTo(10));
            Assert.That(diagnostics.TargetPointId, Is.EqualTo(20));
            Assert.That(diagnostics.RouteDistanceMeters, Is.EqualTo(30));
            Assert.That(diagnostics.VisitedNodes, Is.EqualTo(12));
            Assert.That(diagnostics.JunctionDecisions,
                Is.EqualTo(new[] { new AiPursuitJunctionDecision(7, true, 300) }));
        });
    }

    [Test]
    public void CreatesSearchDiagnosticsFromFailedNavigation()
    {
        var navigation = new AiPursuitNavigationResult(
            AiPursuitNavigationStatus.NoRoute,
            null,
            null,
            AiRouteSearchFailure.DistanceLimit,
            1234,
            new AiPursuitTargetLocationDiagnostics(
                [227470, 57704],
                [171048],
                [new AiPursuitTargetRejection(
                    265571,
                    AiPursuitTargetRejectionReason.OppositeDirection)]),
            MaximumExploredDistanceMeters: 19_999,
            JunctionEdgesExamined: 2);

        var diagnostics = AiPursuitControl.CreateSearchDiagnostics(
            navigation,
            policePointId: 171036,
            previousTargetPointId: 171048);

        Assert.Multiple(() =>
        {
            Assert.That(diagnostics.PolicePointId, Is.EqualTo(171036));
            Assert.That(diagnostics.PreviousTargetPointId, Is.EqualTo(171048));
            Assert.That(diagnostics.SelectedTargetPointId, Is.Null);
            Assert.That(diagnostics.SpatialPointIds, Is.EqualTo(new[] { 227470, 57704 }));
            Assert.That(diagnostics.LaneEquivalentPointIds, Is.EqualTo(new[] { 171048 }));
            Assert.That(diagnostics.SearchFailure, Is.EqualTo(AiRouteSearchFailure.DistanceLimit));
            Assert.That(diagnostics.VisitedNodes, Is.EqualTo(1234));
            Assert.That(diagnostics.MaximumExploredDistanceMeters, Is.EqualTo(19_999));
            Assert.That(diagnostics.JunctionEdgesExamined, Is.EqualTo(2));
        });
    }
}
