using AssettoServer.Server.Ai;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiOverbookingPolicyTests
{
    [Test]
    public void ReservedSlotDoesNotReduceDynamicTrafficAllocation()
    {
        var minimums = Enumerable.Repeat(0, 10).Append(1).ToArray();

        var targets = AiOverbookingPolicy.CalculateTargets(10, minimums);

        Assert.That(targets, Is.EqualTo(Enumerable.Repeat(1, 11)));
    }

    [TestCase(0)]
    [TestCase(5)]
    [TestCase(10)]
    public void ReservedSlotPositionDoesNotChangeDynamicTrafficAllocation(int reservedIndex)
    {
        var minimums = new int[11];
        minimums[reservedIndex] = 1;

        var targets = AiOverbookingPolicy.CalculateTargets(10, minimums);

        Assert.Multiple(() =>
        {
            Assert.That(targets[reservedIndex], Is.EqualTo(1));
            Assert.That(targets.Where((_, index) => index != reservedIndex), Is.All.EqualTo(1));
        });
    }

    [Test]
    public void ExistingDistributionIsUnchangedWithoutReservations()
    {
        var targets = AiOverbookingPolicy.CalculateTargets(5, new int[10]);

        Assert.That(targets, Is.EqualTo(new[] { 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 }));
    }

    [Test]
    public void MinimumReservationSurvivesAZeroOverbookingRequest()
    {
        var target = AiOverbookingPolicy.ClampTarget(0, minimum: 1, maximum: 1);

        Assert.That(target, Is.EqualTo(1));
    }

    [Test]
    public void MinimumReservationWinsAConflictingMaximum()
    {
        var target = AiOverbookingPolicy.ClampTarget(0, minimum: 1, maximum: 0);

        Assert.That(target, Is.EqualTo(1));
    }

    [TestCase(-1, 0, null)]
    [TestCase(0, -1, null)]
    [TestCase(0, 0, -1)]
    public void ClampTargetRejectsNegativeInputs(int requested, int minimum, int? maximum)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            AiOverbookingPolicy.ClampTarget(requested, minimum, maximum));
    }

    [Test]
    public void CalculateTargetsRejectsNegativeDynamicTarget()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            AiOverbookingPolicy.CalculateTargets(-1, new[] { 0 }));
    }

    [Test]
    public void CalculateTargetsRejectsNegativeMinimum()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            AiOverbookingPolicy.CalculateTargets(0, new[] { -1 }));
    }

    [Test]
    public void ReservedOnlyAllocationSurvivesWithoutPlayers()
    {
        var targets = AiOverbookingPolicy.CalculateTargets(0, new[] { 1 });

        Assert.That(targets, Is.EqualTo(new[] { 1 }));
    }
}
