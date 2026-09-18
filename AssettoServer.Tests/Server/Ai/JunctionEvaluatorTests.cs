using AssettoServer.Server.Ai.Splines;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class JunctionEvaluatorTests
{
    [Test]
    public void ExplicitDecisionWinsProbabilityAndClearingRestoresNativeChoice()
    {
        var evaluator = new JunctionEvaluator(_ => 0.0f, savesState: true);
        evaluator.SetExplicitDecisions(new Dictionary<int, bool> { [7] = true });

        Assert.That(evaluator.WillTakeJunction(7), Is.True);

        evaluator.SetExplicitDecisions(null);
        Assert.That(evaluator.WillTakeJunction(7), Is.False);
    }

    [Test]
    public void ExplicitDecisionOnlyAffectsNamedJunction()
    {
        var evaluator = new JunctionEvaluator(_ => 0.0f, savesState: true);
        evaluator.SetExplicitDecisions(new Dictionary<int, bool> { [7] = true });

        Assert.Multiple(() =>
        {
            Assert.That(evaluator.WillTakeJunction(7), Is.True);
            Assert.That(evaluator.WillTakeJunction(8), Is.False);
        });
    }

    [Test]
    public void CopiesExplicitDecisionsBeforePublishingThem()
    {
        var decisions = new Dictionary<int, bool> { [7] = true };
        var evaluator = new JunctionEvaluator(_ => 0.0f, savesState: true);
        evaluator.SetExplicitDecisions(decisions);

        decisions[7] = false;

        Assert.That(evaluator.WillTakeJunction(7), Is.True);
    }

    [Test]
    public void ReplacingExplicitDecisionsRestoresFallbackForRemovedJunction()
    {
        var evaluator = new JunctionEvaluator(_ => 0.0f, savesState: true);
        evaluator.SetExplicitDecisions(new Dictionary<int, bool> { [7] = true });

        evaluator.SetExplicitDecisions(new Dictionary<int, bool> { [8] = true });

        Assert.Multiple(() =>
        {
            Assert.That(evaluator.WillTakeJunction(7), Is.False);
            Assert.That(evaluator.WillTakeJunction(8), Is.True);
        });
    }
}
