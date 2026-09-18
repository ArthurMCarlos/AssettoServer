using AssettoServer.Server.Ai;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class AiSpatialLifecycleTests
{
    [TestCase(40_001, 40_000, 9_000, 8_000, false, true)]
    [TestCase(40_001, 40_000, 9_000, 8_000, true, false)]
    [TestCase(39_999, 40_000, 9_000, 8_000, false, false)]
    [TestCase(40_001, 40_000, 7_999, 8_000, false, false)]
    public void QueueDecisionPreservesNativeRulesExceptRetainedPursuit(
        float distanceSquared,
        float radiusSquared,
        long now,
        long protectionEnds,
        bool retainPursuit,
        bool expected)
    {
        var actual = AiSpatialLifecycle.ShouldQueueForReposition(
            distanceSquared,
            radiusSquared,
            now,
            protectionEnds,
            retainPursuit);

        Assert.That(actual, Is.EqualTo(expected));
    }
}
