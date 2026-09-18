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
