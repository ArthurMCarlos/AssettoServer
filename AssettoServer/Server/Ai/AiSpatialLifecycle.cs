namespace AssettoServer.Server.Ai;

public static class AiSpatialLifecycle
{
    public static bool ShouldQueueForReposition(
        float distanceSquared,
        float playerRadiusSquared,
        long serverTimeMilliseconds,
        long spawnProtectionEnds,
        bool retainPursuit) =>
        !retainPursuit
        && distanceSquared > playerRadiusSquared
        && serverTimeMilliseconds > spawnProtectionEnds;
}
