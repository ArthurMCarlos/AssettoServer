using System.Numerics;
using AssettoServer.Server.Ai.Routing;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiPursuitTargetLocatorTests
{
    [Test]
    public void RejectsNearestPointDrivingInOppositeDirection()
    {
        var locator = CreateLocator(
            new AiPursuitTargetCandidateSource(10, 1, Vector3.UnitX),
            new AiPursuitTargetCandidateSource(20, 4, -Vector3.UnitX));

        var candidates = locator.FindCandidates(
            Vector3.Zero,
            -Vector3.UnitX * 20,
            maximumDistanceSquared: 49,
            maximumCandidates: 16);

        Assert.That(candidates.Select(candidate => candidate.PointId),
            Is.EqualTo(new[] { 20 }));
    }

    [Test]
    public void KeepsNearbyCandidatesWhenTargetIsStopped()
    {
        var locator = CreateLocator(
            new AiPursuitTargetCandidateSource(10, 1, Vector3.UnitX),
            new AiPursuitTargetCandidateSource(20, 4, -Vector3.UnitX));

        var candidates = locator.FindCandidates(
            Vector3.Zero,
            Vector3.Zero,
            maximumDistanceSquared: 49,
            maximumCandidates: 16);

        Assert.That(candidates.Select(candidate => candidate.PointId),
            Is.EqualTo(new[] { 10, 20 }));
    }

    [Test]
    public void DiscardsPointsOutsideMaximumDistance()
    {
        var locator = CreateLocator(
            new AiPursuitTargetCandidateSource(10, 9, Vector3.UnitX),
            new AiPursuitTargetCandidateSource(20, 25, Vector3.UnitX));

        var candidates = locator.FindCandidates(
            Vector3.Zero,
            Vector3.UnitX * 10,
            maximumDistanceSquared: 16,
            maximumCandidates: 16);

        Assert.That(candidates.Select(candidate => candidate.PointId),
            Is.EqualTo(new[] { 10 }));
    }

    [Test]
    public void LimitsAndOrdersCandidatesDeterministically()
    {
        var locator = CreateLocator(
            new AiPursuitTargetCandidateSource(30, 4, Vector3.UnitX),
            new AiPursuitTargetCandidateSource(20, 1, Vector3.UnitX),
            new AiPursuitTargetCandidateSource(10, 1, Vector3.UnitX));

        var candidates = locator.FindCandidates(
            Vector3.Zero,
            Vector3.Zero,
            maximumDistanceSquared: 100,
            maximumCandidates: 2);

        Assert.That(candidates.Select(candidate => candidate.PointId),
            Is.EqualTo(new[] { 10, 20 }));
    }

    private static AiPursuitTargetLocator CreateLocator(
        params AiPursuitTargetCandidateSource[] sources) =>
        new((_, _) => sources);
}
