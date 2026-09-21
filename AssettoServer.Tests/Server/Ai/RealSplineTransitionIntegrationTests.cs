using System.IO.Compression;
using System.Numerics;
using System.Text;
using AssettoServer.Server.Ai;
using AssettoServer.Server.Ai.Configuration;
using AssettoServer.Server.Ai.Routing;
using AssettoServer.Server.Ai.Splines;
using AssettoServer.Server.Configuration;

namespace AssettoServer.Tests.Server.Ai;

[TestFixture]
public class RealSplineTransitionIntegrationTests
{
    [Test]
    public void ParserAndNavigatorUseReachableEquivalentFromDenseParallelSplines()
    {
        using var fixture = AipFixture.CreateSynthetic();
        using var spline = fixture.Load("test_track");
        var navigator = new AiPursuitNavigator(
            new AiRoutePlanner(spline),
            new AiPursuitTargetLocator(spline));

        var result = navigator.Update(
            policePointId: 40,
            targetPosition: spline.Points[247].Position,
            targetVelocity: Vector3.UnitX * 20,
            maximumTargetDistanceSquared: 49,
            nowMilliseconds: 0,
            previous: null,
            new AiPursuitNavigationOptions(
                new AiRouteSearchLimits(20_000, 50_000),
                RouteGraceMilliseconds: 2000));

        Assert.Multiple(() =>
        {
            Assert.That(result.Status, Is.EqualTo(AiPursuitNavigationStatus.Active));
            Assert.That(result.State!.TargetPointId, Is.InRange(41, 49));
            Assert.That(result.State.Plan.DistanceMeters, Is.InRange(1, 9));
            Assert.That(result.State.Plan.JunctionDecisions, Is.Empty);
        });
    }

    [Test]
    [Explicit("Requires POLICE_CHASE_FAST_LANE_AIP pointing to the real Shutoko package")]
    public void RealShutokoPackageReproducesAndFixes171036Transition()
    {
        var sourcePath = Environment.GetEnvironmentVariable("POLICE_CHASE_FAST_LANE_AIP");
        Assert.That(sourcePath, Is.Not.Null.And.Not.Empty);
        Assert.That(File.Exists(sourcePath), Is.True, sourcePath);

        using var fixture = AipFixture.FromExisting(sourcePath!);
        using var spline = fixture.Load("shuto_revival_project_beta_ptb");
        var targetPoint = spline.Points[227470];
        var navigator = new AiPursuitNavigator(
            new AiRoutePlanner(spline),
            new AiPursuitTargetLocator(spline));

        var result = navigator.Update(
            policePointId: 171036,
            targetPosition: targetPoint.Position,
            targetVelocity: spline.Operations.GetForwardVector(targetPoint.Id) * 20,
            maximumTargetDistanceSquared: 49,
            nowMilliseconds: 0,
            previous: null,
            new AiPursuitNavigationOptions(
                new AiRouteSearchLimits(20_000, 50_000),
                RouteGraceMilliseconds: 2000));

        Assert.Multiple(() =>
        {
            Assert.That(spline.GetLanes(227470).ToArray(), Does.Contain(171048));
            Assert.That(result.Status, Is.EqualTo(AiPursuitNavigationStatus.Active));
            Assert.That(result.State!.TargetPointId, Is.InRange(171044, 171052));
            Assert.That(result.State.Plan.DistanceMeters, Is.InRange(12, 26));
            Assert.That(result.State.Plan.JunctionDecisions, Is.Empty);
        });
    }

    [TestCase(171761, 283881, 283943)]
    [TestCase(171763, 283883, 283945)]
    [TestCase(171765, 283885, 283947)]
    [Explicit("Requires POLICE_CHASE_FAST_LANE_AIP pointing to the real Shutoko package")]
    public void DirectPhysicalTargetUsesReciprocalAdjacentLaneWithoutJunction(
        int policePointId,
        int adjacentPointId,
        int targetPointId)
    {
        var sourcePath = GetRealFastLanePath();
        using var fixture = AipFixture.FromExisting(sourcePath);
        using var spline = fixture.Load("shuto_revival_project_beta_ptb");
        var planner = new AiRoutePlanner(spline);
        var selector = new AiPursuitLaneSelector(spline, planner);

        var result = selector.SelectForAlignment(
            policePointId,
            new HashSet<int> { targetPointId },
            new AiRouteSearchLimits(20_000, 50_000),
            60,
            1000);

        var sourceLeftId = spline.Points[policePointId].LeftId;
        var sourceRightId = spline.Points[policePointId].RightId;
        var adjacentLeftId = spline.Points[adjacentPointId].LeftId;
        var adjacentRightId = spline.Points[adjacentPointId].RightId;
        Assert.Multiple(() =>
        {
            Assert.That(new[] { sourceLeftId, sourceRightId },
                Does.Contain(adjacentPointId));
            Assert.That(new[] { adjacentLeftId, adjacentRightId },
                Does.Contain(policePointId));
            Assert.That(spline.Operations.IsSameDirection(policePointId, adjacentPointId),
                Is.True);
            Assert.That(result.Selection!.ToPointId, Is.EqualTo(adjacentPointId));
            Assert.That(result.Selection.Motivation,
                Is.EqualTo(AiPursuitLaneMotivation.TargetLaneAlignment));
            Assert.That(result.Selection.JunctionId, Is.Null);
            Assert.That(result.Selection.DistanceToDecisionMeters, Is.InRange(90, 100));
            Assert.That(result.Selection.DestinationPlan.Nodes[0].PointId,
                Is.EqualTo(adjacentPointId));
        });
    }

    [Test]
    [Explicit("Requires POLICE_CHASE_FAST_LANE_AIP pointing to the real Shutoko package")]
    public void DirectPhysicalTargetCompletesAtImmediateAdjacentLane()
    {
        const int policePointId = 171761;
        const int adjacentPointId = 283881;
        const int targetPointId = 283943;
        var sourcePath = GetRealFastLanePath();
        using var fixture = AipFixture.FromExisting(sourcePath);
        using var spline = fixture.Load("shuto_revival_project_beta_ptb");
        var planner = new AiRoutePlanner(spline);
        var selector = new AiPursuitLaneSelector(spline, planner);
        var selection = selector.SelectForAlignment(
            policePointId,
            new HashSet<int> { targetPointId },
            new AiRouteSearchLimits(20_000, 50_000),
            60,
            1000).Selection!;
        var controller = new AiLaneChangeController(60, 3000);

        var prepared = controller.Prepare(
            selection,
            CreateCursor(spline, policePointId, 0),
            CreateCursor(spline, adjacentPointId, 0),
            0,
            1);
        controller.UpdateWaiting(AiLaneChangeSafetyStatus.Safe, 0);
        var moved = controller.TryMove(60, 100, out var movement);

        Assert.Multiple(() =>
        {
            Assert.That(prepared, Is.True);
            Assert.That(moved, Is.True);
            Assert.That(movement.Completed, Is.True);
            Assert.That(selection.ToPointId, Is.EqualTo(adjacentPointId));
            Assert.That(IsOnForwardChain(
                spline, adjacentPointId, movement.DestinationPointId), Is.True,
                "Completion must advance continuously on the selected adjacent lane");
            Assert.That(movement.DestinationPointId, Is.Not.EqualTo(policePointId));
            Assert.That(controller.Event!.Kind,
                Is.EqualTo(AiPursuitLaneChangeEventKind.Completed));
            Assert.That(selection.DestinationPlan.Nodes[0].PointId,
                Is.EqualTo(adjacentPointId),
                "The route starts on the adjacent forward chain; lateral motion belongs to the controller");
        });
    }

    [Test]
    [Explicit("Requires POLICE_CHASE_FAST_LANE_AIP pointing to the real Shutoko package")]
    public void RealShutokoPackageSelectsImmediateLaneForForwardJunctionRoute()
    {
        var sourcePath = Environment.GetEnvironmentVariable("POLICE_CHASE_FAST_LANE_AIP");
        Assert.That(sourcePath, Is.Not.Null.And.Not.Empty);
        Assert.That(File.Exists(sourcePath), Is.True, sourcePath);

        using var fixture = AipFixture.FromExisting(sourcePath!);
        using var spline = fixture.Load("shuto_revival_project_beta_ptb");
        var planner = new AiRoutePlanner(spline);
        var selector = new AiPursuitLaneSelector(spline, planner);
        var limits = new AiRouteSearchLimits(20_000, 50_000);
        var scenario = FindLaneChangeScenario(spline, planner, limits);

        Assert.That(scenario, Is.Not.Null,
            "Expected at least one real junction reachable only through an immediate adjacent lane");

        var result = selector.Select(
            scenario!.SourcePointId,
            new HashSet<int> { scenario.TargetPointId },
            limits,
            maneuverDistanceMeters: 60);

        Assert.Multiple(() =>
        {
            Assert.That(result.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Change));
            Assert.That(result.Selection!.ToPointId, Is.EqualTo(scenario.DestinationPointId));
            Assert.That(result.Selection.DistanceToDecisionMeters, Is.GreaterThanOrEqualTo(60));
            Assert.That(result.Selection.DestinationPlan.JunctionDecisions,
                Contains.Key(scenario.JunctionId));
            Assert.That(planner.TryPlan(
                scenario.SourcePointId,
                new HashSet<int> { scenario.TargetPointId },
                limits).Plan, Is.Null,
                "The route graph must remain forward-only without lateral edges");

            var source = spline.Points[scenario.SourcePointId].Position;
            var destination = spline.Points[scenario.DestinationPointId].Position;
            var midpoint = AiLaneChangeTrajectory.Blend(
                new AiSplinePose(source, spline.Operations.GetForwardVector(scenario.SourcePointId)),
                new AiSplinePose(destination, spline.Operations.GetForwardVector(scenario.DestinationPointId)),
                0.5f,
                60).Position;
            Assert.That(Vector3.Distance(midpoint, (source + destination) / 2),
                Is.LessThan(0.01f));
        });
    }

    [Test]
    [Explicit("Requires POLICE_CHASE_FAST_LANE_AIP pointing to the real Shutoko package")]
    public void RealShutokoPackageReproducesLateLaneChangeEvaluationGap()
    {
        var sourcePath = Environment.GetEnvironmentVariable("POLICE_CHASE_FAST_LANE_AIP");
        Assert.That(sourcePath, Is.Not.Null.And.Not.Empty);
        Assert.That(File.Exists(sourcePath), Is.True, sourcePath);

        using var fixture = AipFixture.FromExisting(sourcePath!);
        using var spline = fixture.Load("shuto_revival_project_beta_ptb");
        var planner = new AiRoutePlanner(spline);
        var selector = new AiPursuitLaneSelector(spline, planner);
        var limits = new AiRouteSearchLimits(20_000, 50_000);
        var scenario = FindMaskedLaneChangeScenario(spline, planner, limits);
        Assert.That(scenario, Is.Not.Null,
            "Expected a real junction where P0.6 fallback targets mask lane preparation");

        var target = spline.Points[scenario!.TargetPointId];
        var navigator = new AiPursuitNavigator(
            planner,
            new AiPursuitTargetLocator(spline));
        var navigation = navigator.Update(
            scenario.SourcePointId,
            target.Position,
            spline.Operations.GetForwardVector(target.Id) * 20,
            49,
            0,
            null,
            new AiPursuitNavigationOptions(limits, 2000));
        var currentDecision = selector.Select(
            scenario.SourcePointId,
            navigation.TargetPointIds.ToHashSet(),
            limits,
            60,
            1000);
        var physicalTargetDecision = selector.Select(
            scenario.SourcePointId,
            new HashSet<int> { scenario.TargetPointId },
            limits,
            60,
            1000);
        var controller = new AiLaneChangeController(60, 3000);
        var pipeline = new AiPursuitLaneChangePipeline(
            selector,
            controller,
            (pointId, progress) => CreateCursor(spline, pointId, progress),
            pointId => GetSegmentLength(spline, pointId));
        var preparation = pipeline.Evaluate(
            scenario.SourcePointId,
            0,
            GetSegmentLength(spline, scenario.SourcePointId),
            navigation,
            null,
            new AiPursuitLaneChangeOptions(true, 60, 3000, 1000),
            limits,
            0);

        TestContext.Progress.WriteLine(
            $"source={scenario.SourcePointId} destination={scenario.DestinationPointId} " +
            $"target={scenario.TargetPointId} junction={scenario.JunctionId} " +
            $"navigatorTarget={navigation.State?.TargetPointId} " +
            $"candidateCount={navigation.TargetPointIds.Count} " +
            $"spatial=[{string.Join(',', navigation.TargetLocationDiagnostics.SpatialPointIds)}] " +
            $"equivalents=[{string.Join(',', navigation.TargetLocationDiagnostics.LaneEquivalentPointIds)}]");
        Assert.Multiple(() =>
        {
            Assert.That(navigation.Status, Is.EqualTo(AiPursuitNavigationStatus.Active));
            Assert.That(scenario.SourcePointId, Is.EqualTo(312936));
            Assert.That(scenario.TargetPointId, Is.EqualTo(311797));
            Assert.That(scenario.JunctionId, Is.EqualTo(2));
            Assert.That(navigation.State!.TargetPointId, Is.EqualTo(313071));
            Assert.That(navigation.PreferredPhysicalTargetPointId, Is.EqualTo(311797));
            Assert.That(currentDecision.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Stay),
                "P0.6 lane equivalents keep a fallback route active on the current lane");
            Assert.That(physicalTargetDecision.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Change),
                "The same position has a junction route through the immediate adjacent lane");
            Assert.That(physicalTargetDecision.Selection!.DestinationPlan.JunctionDecisions,
                Contains.Key(scenario.JunctionId));
            Assert.That(preparation.RequestPrepared, Is.True);
            Assert.That(preparation.Evaluation.Selection!.JunctionId,
                Is.EqualTo(scenario.JunctionId));
            Assert.That(controller.Phase, Is.EqualTo(AiLaneChangePhase.WaitingForGap));
        });

        controller.UpdateWaiting(AiLaneChangeSafetyStatus.Safe, 0);
        var moved = controller.TryMove(60, 100, out var movement);

        Assert.Multiple(() =>
        {
            Assert.That(moved, Is.True);
            Assert.That(movement.Completed, Is.True);
            Assert.That(movement.DestinationPointId,
                Is.Not.EqualTo(scenario.SourcePointId));
            Assert.That(controller.Phase, Is.EqualTo(AiLaneChangePhase.Cooldown));
            Assert.That(controller.Event!.Kind,
                Is.EqualTo(AiPursuitLaneChangeEventKind.Completed));
        });

        var adoptedRoute = planner.TryPlan(
            movement.DestinationPointId,
            new HashSet<int> { scenario.TargetPointId },
            limits);
        var postCompletion = selector.Select(
            movement.DestinationPointId,
            new HashSet<int> { scenario.TargetPointId },
            limits,
            60,
            1000);

        Assert.Multiple(() =>
        {
            Assert.That(adoptedRoute.Plan, Is.Not.Null,
                "Completed destination chain must still reach the physical anchor");
            Assert.That(adoptedRoute.Plan!.Nodes[^1].PointId,
                Is.EqualTo(scenario.TargetPointId));
            Assert.That(postCompletion.Kind, Is.EqualTo(AiPursuitLaneSelectionKind.Stay),
                "The adopted destination chain must not request an immediate return");
        });
    }

    [Test]
    [Explicit("Requires POLICE_CHASE_FAST_LANE_AIP pointing to the real Shutoko package")]
    public void DiagnoseRealServerFailureRegion171751()
    {
        var sourcePath = Environment.GetEnvironmentVariable("POLICE_CHASE_FAST_LANE_AIP");
        Assert.That(sourcePath, Is.Not.Null.And.Not.Empty);
        using var fixture = AipFixture.FromExisting(sourcePath!);
        using var spline = fixture.Load("shuto_revival_project_beta_ptb");
        var planner = new AiRoutePlanner(spline);
        var limits = new AiRouteSearchLimits(20_000, 50_000);
        var targetIds = spline.GetLanes(171780).ToArray();

        foreach (var pointId in new[] { 171751, 171772, 171780 })
        {
            ref readonly var point = ref spline.Points[pointId];
            var route = planner.TryPlan(pointId, targetIds.ToHashSet(), limits);
            TestContext.Progress.WriteLine(
                $"point={pointId} previous={point.PreviousId} next={point.NextId} " +
                $"left={point.LeftId} right={point.RightId} junctionStart={point.JunctionStartId} " +
                $"junctionEnd={point.JunctionEndId} lanes=[{string.Join(',', spline.GetLanes(pointId).ToArray())}] " +
                $"route={(route.Plan == null ? route.Failure : route.Plan.DistanceMeters)} " +
                $"visited={route.VisitedNodes} junctionEdges={route.JunctionEdgesExamined}");
        }

        var cursor = 171751;
        var travelled = 0.0f;
        for (var i = 0; i < 20_000 && cursor >= 0 && travelled < 20_000; i++)
        {
            ref readonly var point = ref spline.Points[cursor];
            if (point.JunctionStartId >= 0)
            {
                ref readonly var junction = ref spline.Junctions[point.JunctionStartId];
                TestContext.Progress.WriteLine(
                    $"forwardJunction={junction.Id} start={junction.StartPointId} " +
                    $"end={junction.EndPointId} distance={travelled:0.0}");
            }
            if (point.NextId < 0)
                break;
            travelled += Vector3.Distance(point.Position, spline.Points[point.NextId].Position);
            cursor = point.NextId;
        }

        var retainedTargetRoute = planner.TryPlan(171751, targetIds.ToHashSet(), limits);
        Assert.Multiple(() =>
        {
            Assert.That(retainedTargetRoute.Plan, Is.Not.Null);
            Assert.That(retainedTargetRoute.Plan!.DistanceMeters, Is.LessThan(100));
            Assert.That(retainedTargetRoute.Plan.JunctionDecisions, Is.Empty);
        });
    }

    private static LaneChangeScenario? FindMaskedLaneChangeScenario(
        AiSpline spline,
        AiRoutePlanner planner,
        AiRouteSearchLimits limits)
    {
        var selector = new AiPursuitLaneSelector(spline, planner);
        var navigator = new AiPursuitNavigator(
            planner,
            new AiPursuitTargetLocator(spline));

        foreach (ref readonly var junction in spline.Junctions)
        {
            var targetPointId = Advance(spline, junction.EndPointId, 150);
            var destinationPointId = junction.StartPointId;
            var distanceBeforeDecision = 0.0f;
            while (destinationPointId >= 0 && distanceBeforeDecision <= 250)
            {
                ref readonly var destination = ref spline.Points[destinationPointId];
                if (distanceBeforeDecision >= 60)
                {
                    foreach (var sourcePointId in new[] { destination.LeftId, destination.RightId })
                    {
                        if (sourcePointId < 0
                            || !spline.Operations.IsSameDirection(sourcePointId, destinationPointId))
                        {
                            continue;
                        }

                        var exactTargets = new HashSet<int> { targetPointId };
                        if (planner.TryPlan(destinationPointId, exactTargets, limits).Plan == null
                            || planner.TryPlan(sourcePointId, exactTargets, limits).Plan != null)
                        {
                            continue;
                        }

                        var target = spline.Points[targetPointId];
                        var navigation = navigator.Update(
                            sourcePointId,
                            target.Position,
                            spline.Operations.GetForwardVector(targetPointId) * 20,
                            49,
                            0,
                            null,
                            new AiPursuitNavigationOptions(limits, 2000));
                        if (navigation.Status != AiPursuitNavigationStatus.Active)
                            continue;

                        var currentDecision = selector.Select(
                            sourcePointId,
                            navigation.TargetPointIds.ToHashSet(),
                            limits,
                            60);
                        var physicalDecision = selector.Select(
                            sourcePointId,
                            exactTargets,
                            limits,
                            60);
                        if (currentDecision.Kind == AiPursuitLaneSelectionKind.Stay
                            && physicalDecision.Kind == AiPursuitLaneSelectionKind.Change)
                        {
                            return new LaneChangeScenario(
                                sourcePointId,
                                destinationPointId,
                                targetPointId,
                                junction.Id);
                        }
                    }
                }

                if (destination.PreviousId < 0)
                    break;
                distanceBeforeDecision += Vector3.Distance(
                    destination.Position,
                    spline.Points[destination.PreviousId].Position);
                destinationPointId = destination.PreviousId;
            }
        }

        return null;
    }

    private static LaneChangeScenario? FindLaneChangeScenario(
        AiSpline spline,
        AiRoutePlanner planner,
        AiRouteSearchLimits limits)
    {
        foreach (ref readonly var junction in spline.Junctions)
        {
            var targetPointId = Advance(spline, junction.EndPointId, 150);
            var destinationPointId = junction.StartPointId;
            var distanceBeforeDecision = 0.0f;

            while (destinationPointId >= 0 && distanceBeforeDecision <= 250)
            {
                ref readonly var destination = ref spline.Points[destinationPointId];
                if (distanceBeforeDecision >= 60)
                {
                    foreach (var sourcePointId in new[] { destination.LeftId, destination.RightId })
                    {
                        if (sourcePointId < 0
                            || !spline.Operations.IsSameDirection(sourcePointId, destinationPointId))
                        {
                            continue;
                        }

                        var targetIds = new HashSet<int> { targetPointId };
                        if (planner.TryPlan(destinationPointId, targetIds, limits).Plan != null
                            && planner.TryPlan(sourcePointId, targetIds, limits).Plan == null)
                        {
                            return new LaneChangeScenario(
                                sourcePointId,
                                destinationPointId,
                                targetPointId,
                                junction.Id);
                        }
                    }
                }

                if (destination.PreviousId < 0)
                    break;
                distanceBeforeDecision += Vector3.Distance(
                    destination.Position,
                    spline.Points[destination.PreviousId].Position);
                destinationPointId = destination.PreviousId;
            }
        }

        return null;
    }

    private static int Advance(AiSpline spline, int pointId, float distanceMeters)
    {
        var travelled = 0.0f;
        while (pointId >= 0 && travelled < distanceMeters)
        {
            ref readonly var point = ref spline.Points[pointId];
            if (point.NextId < 0)
                break;
            travelled += Vector3.Distance(point.Position, spline.Points[point.NextId].Position);
            pointId = point.NextId;
        }

        return pointId;
    }

    private static AiSplineCursor CreateCursor(
        AiSpline spline,
        int pointId,
        float progress) =>
        new(
            pointId,
            progress,
            current => spline.Points[current].NextId >= 0
                ? spline.Points[current].NextId
                : null,
            current => GetSegmentLength(spline, current),
            (current, _) => new AiSplinePose(
                spline.Points[current].Position,
                spline.Operations.GetForwardVector(current)));

    private static float GetSegmentLength(AiSpline spline, int pointId)
    {
        var nextPointId = spline.Points[pointId].NextId;
        return nextPointId < 0
            ? 0
            : Vector3.Distance(
                spline.Points[pointId].Position,
                spline.Points[nextPointId].Position);
    }

    private static bool IsOnForwardChain(
        AiSpline spline,
        int startPointId,
        int candidatePointId)
    {
        var pointId = startPointId;
        for (var i = 0; i < 1000 && pointId >= 0; i++)
        {
            if (pointId == candidatePointId)
                return true;
            pointId = spline.Points[pointId].NextId;
        }
        return false;
    }

    private static string GetRealFastLanePath()
    {
        var sourcePath = Environment.GetEnvironmentVariable("POLICE_CHASE_FAST_LANE_AIP");
        Assert.That(sourcePath, Is.Not.Null.And.Not.Empty);
        Assert.That(File.Exists(sourcePath), Is.True, sourcePath);
        return sourcePath!;
    }

    private sealed record LaneChangeScenario(
        int SourcePointId,
        int DestinationPointId,
        int TargetPointId,
        int JunctionId);

    private sealed class AipFixture : IDisposable
    {
        private readonly string _directory;
        private readonly string _packagePath;
        private readonly string _cachePath;

        private AipFixture(string directory)
        {
            _directory = directory;
            _packagePath = Path.Join(directory, "fast_lane.aip");
            _cachePath = Path.Join(directory, "spline.aic1");
        }

        public static AipFixture CreateSynthetic()
        {
            var fixture = Create();
            using var archive = ZipFile.Open(fixture._packagePath, ZipArchiveMode.Create);
            WriteText(archive, "config.yml", "Track: test_track\nSplines: []\n");
            WriteSpline(archive, "fast_lane_a.ai", z: 0);
            WriteSpline(archive, "fast_lane_b.ai", z: 3);
            WriteSpline(archive, "fast_lane_c.ai", z: 6);
            return fixture;
        }

        public static AipFixture FromExisting(string sourcePath)
        {
            var fixture = Create();
            File.Copy(sourcePath, fixture._packagePath);
            return fixture;
        }

        public AiSpline Load(string track)
        {
            var parser = new FastLaneParser(track, new AiParams
            {
                LaneWidthMeters = 3
            });
            var mutable = parser.FromFiles(_directory);
            new AiSplineWriter().ToFile(mutable, _cachePath);
            return new AiSpline(_cachePath);
        }

        public void Dispose()
        {
            Directory.Delete(_directory, recursive: true);
        }

        private static AipFixture Create()
        {
            var directory = Path.Join(
                Path.GetTempPath(),
                $"assetto-server-pursuit-{Guid.NewGuid():N}");
            Directory.CreateDirectory(directory);
            return new AipFixture(directory);
        }

        private static void WriteText(
            ZipArchive archive,
            string name,
            string content)
        {
            var entry = archive.CreateEntry(name);
            using var writer = new StreamWriter(
                entry.Open(),
                new UTF8Encoding(encoderShouldEmitUTF8Identifier: false));
            writer.Write(content);
        }

        private static void WriteSpline(
            ZipArchive archive,
            string name,
            float z)
        {
            var entry = archive.CreateEntry(name);
            using var writer = new BinaryWriter(entry.Open());
            writer.Write(-1);
            writer.Write(101);
            for (var x = 0; x <= 100; x++)
            {
                writer.Write((float)x);
                writer.Write(0.0f);
                writer.Write(z);
                writer.Write(1000.0f);
                writer.Write(0.0f);
            }
        }
    }
}
