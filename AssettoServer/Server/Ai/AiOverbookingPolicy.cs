using System;
using System.Collections.Generic;
using System.Linq;

namespace AssettoServer.Server.Ai;

internal static class AiOverbookingPolicy
{
    public static int ClampTarget(int requested, int minimum, int? maximum)
    {
        if (requested < 0)
            throw new ArgumentOutOfRangeException(nameof(requested));
        if (minimum < 0)
            throw new ArgumentOutOfRangeException(nameof(minimum));
        if (maximum < 0)
            throw new ArgumentOutOfRangeException(nameof(maximum));

        return Math.Max(Math.Min(requested, maximum ?? int.MaxValue), minimum);
    }

    public static IReadOnlyList<int> CalculateTargets(int dynamicTargetCount, IReadOnlyList<int> minimums)
    {
        if (dynamicTargetCount < 0)
            throw new ArgumentOutOfRangeException(nameof(dynamicTargetCount));
        if (minimums.Any(minimum => minimum < 0))
            throw new ArgumentOutOfRangeException(nameof(minimums));

        var dynamicSlotCount = minimums.Count(minimum => minimum == 0);
        var overbooking = dynamicSlotCount == 0 ? 0 : dynamicTargetCount / dynamicSlotCount;
        var rest = dynamicSlotCount == 0 ? 0 : dynamicTargetCount % dynamicSlotCount;
        var dynamicIndex = 0;
        var targets = new int[minimums.Count];

        for (var index = 0; index < minimums.Count; index++)
        {
            if (minimums[index] > 0)
            {
                targets[index] = minimums[index];
                continue;
            }

            targets[index] = dynamicIndex < rest ? overbooking + 1 : overbooking;
            dynamicIndex++;
        }

        return targets;
    }
}
