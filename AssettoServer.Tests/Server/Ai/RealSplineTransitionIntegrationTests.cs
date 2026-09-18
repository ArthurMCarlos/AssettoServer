using System.IO.Compression;
using System.Numerics;
using System.Text;
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
