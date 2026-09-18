using AssettoServer.Server.Ai;
using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitControlTests
{
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
            12);

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
}
