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

    [Test]
    public void ExpandsAcceptedSpatialSeedsToLaneEquivalentPoints()
    {
        var spatialSeeds = Enumerable.Range(10, 16)
            .Select(pointId => new AiPursuitTargetCandidateSource(
                pointId,
                pointId - 9,
                Vector3.UnitX))
            .ToArray();
        var locator = new AiPursuitTargetLocator(
            (_, _) => spatialSeeds,
            (pointId, _) => pointId == 10
                ? [new AiPursuitTargetCandidateSource(99, 64, Vector3.UnitX)]
                : []);

        var candidates = locator.FindCandidates(
            Vector3.Zero,
            Vector3.UnitX * 20,
            maximumDistanceSquared: 49,
            maximumCandidates: 16);

        Assert.That(candidates.Select(candidate => candidate.PointId),
            Does.Contain(99));
    }

    [Test]
    public void ReportsSpatialLaneAndRejectedCandidates()
    {
        var locator = new AiPursuitTargetLocator(
            (_, _) =>
            [
                new AiPursuitTargetCandidateSource(10, 1, Vector3.UnitX),
                new AiPursuitTargetCandidateSource(20, 64, Vector3.UnitX),
                new AiPursuitTargetCandidateSource(30, 4, -Vector3.UnitX)
            ],
            (pointId, _) => pointId == 10
                ? [new AiPursuitTargetCandidateSource(99, 64, Vector3.UnitX)]
                : []);

        var result = locator.LocateCandidates(
            Vector3.Zero,
            Vector3.UnitX * 20,
            maximumDistanceSquared: 49,
            maximumCandidates: 16);

        Assert.Multiple(() =>
        {
            Assert.That(result.Candidates.Select(candidate => candidate.PointId),
                Is.EqualTo(new[] { 10, 99 }));
            Assert.That(result.Diagnostics.SpatialPointIds,
                Is.EqualTo(new[] { 10, 20, 30 }));
            Assert.That(result.Diagnostics.LaneEquivalentPointIds,
                Is.EqualTo(new[] { 99 }));
            Assert.That(result.Diagnostics.Rejections,
                Is.EqualTo(new[]
                {
                    new AiPursuitTargetRejection(
                        20,
                        AiPursuitTargetRejectionReason.OutsideMaximumDistance),
                    new AiPursuitTargetRejection(
                        30,
                        AiPursuitTargetRejectionReason.OppositeDirection)
                }));
        });
    }

    [Test]
    public void ExpandsLaneEquivalentsFromDirectionRejectedSpatialSeed()
    {
        var locator = new AiPursuitTargetLocator(
            (_, _) =>
            [
                new AiPursuitTargetCandidateSource(10, 1, -Vector3.UnitX)
            ],
            (pointId, _) => pointId == 10
                ? [new AiPursuitTargetCandidateSource(99, 16, Vector3.UnitX)]
                : []);

        var result = locator.LocateCandidates(
            Vector3.Zero,
            Vector3.UnitX * 20,
            maximumDistanceSquared: 49,
            maximumCandidates: 16);

        Assert.That(result.Candidates.Select(candidate => candidate.PointId),
            Is.EqualTo(new[] { 99 }));
    }

    private static AiPursuitTargetLocator CreateLocator(
        params AiPursuitTargetCandidateSource[] sources) =>
        new((_, _) => sources);
}
