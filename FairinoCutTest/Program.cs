using System.Text.Json;
using fairino;

if (args.Length == 0)
{
    PrintUsage();
    return;
}

var robotIp = args[0];
var trajectoryPath = args.Length > 1 ? args[1] : "trajectory_infinity.json";

if (!File.Exists(trajectoryPath))
{
    Console.WriteLine($"Trajectory file not found: {trajectoryPath}");
    return;
}

var jsonOptions = new JsonSerializerOptions
{
    PropertyNameCaseInsensitive = true
};

var config = JsonSerializer.Deserialize<TrajectoryConfig>(File.ReadAllText(trajectoryPath), jsonOptions);
if (config is null || config.Points.Count < 2)
{
    Console.WriteLine("Trajectory config invalid. Need at least 2 points.");
    return;
}

var robot = new Robot();
var connectCode = robot.RPC(robotIp);
if (connectCode != 0)
{
    Console.WriteLine($"RPC connect failed to {robotIp}. err={connectCode}");
    return;
}

Console.WriteLine($"Connected to {robotIp}. Start trajectory from '{trajectoryPath}'");
Console.WriteLine("Press Ctrl+C to stop.");

var stopRequested = false;
Console.CancelKeyPress += (_, eventArgs) =>
{
    eventArgs.Cancel = true;
    stopRequested = true;
};

var exaxis = new ExaxisPos(0, 0, 0, 0);
var zeroJoint = new JointPos(0, 0, 0, 0, 0, 0);

try
{
    var loopIndex = 0;
    while (!stopRequested && (config.LoopCount == 0 || loopIndex < config.LoopCount))
    {
        loopIndex++;
        Console.WriteLine($"Loop #{loopIndex}");

        for (var i = 0; i < config.Points.Count; i++)
        {
            if (stopRequested)
            {
                break;
            }

            var point = config.Points[i];
            var pose = new DescPose(point.X, point.Y, point.Z, point.Rx, point.Ry, point.Rz);
            var moveCode = robot.MoveL(
                zeroJoint,
                pose,
                config.Tool,
                config.User,
                config.Velocity,
                config.Acceleration,
                config.Ovl,
                config.BlendRadius,
                exaxis,
                0,
                0,
                pose);

            if (moveCode != 0)
            {
                Console.WriteLine($"MoveL failed at point #{i + 1} ({point.X},{point.Y},{point.Z}), err={moveCode}");
                stopRequested = true;
                break;
            }

            if (config.WaitMotionDone)
            {
                var done = 0;
                do
                {
                    Thread.Sleep(config.MotionDonePollMs);
                    var doneCode = robot.GetRobotMotionDone(ref done);
                    if (doneCode != 0)
                    {
                        Console.WriteLine($"GetRobotMotionDone failed. err={doneCode}");
                        stopRequested = true;
                        break;
                    }
                } while (!stopRequested && done == 0);
            }
        }
    }
}
finally
{
    robot.CloseRPC();
    Console.WriteLine("RPC closed.");
}

static void PrintUsage()
{
    Console.WriteLine("Usage: dotnet run -- <robot_ip> [trajectory_file]");
    Console.WriteLine("Example with simulator IP: dotnet run -- 127.0.0.1 trajectory_infinity.json");
}

internal sealed class TrajectoryConfig
{
    public int Tool { get; init; } = 0;
    public int User { get; init; } = 0;
    public float Velocity { get; init; } = 20;
    public float Acceleration { get; init; } = 20;
    public float Ovl { get; init; } = 100;
    public float BlendRadius { get; init; } = 5;
    public bool WaitMotionDone { get; init; } = true;
    public int MotionDonePollMs { get; init; } = 30;
    public int LoopCount { get; init; } = 0;
    public List<CartPoint> Points { get; init; } = [];
}

internal sealed class CartPoint
{
    public double X { get; init; }
    public double Y { get; init; }
    public double Z { get; init; }
    public double Rx { get; init; }
    public double Ry { get; init; }
    public double Rz { get; init; }
}
