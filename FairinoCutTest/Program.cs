using System.Web.Script.Serialization;
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

var serializer = new JavaScriptSerializer();
var config = serializer.Deserialize<TrajectoryConfig>(File.ReadAllText(trajectoryPath));
if (config is null || config.Points == null || config.Points.Count < 2)
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
var stopCommandSent = false;
Console.CancelKeyPress += (_, eventArgs) =>
{
    eventArgs.Cancel = true;
    stopRequested = true;
    RequestSafeStop(robot, "Ctrl+C", ref stopCommandSent);
};

var exaxis = new ExaxisPos(0, 0, 0, 0);

try
{
    JointPos currentJoint = new JointPos(new double[6]);
    var currCode = robot.GetActualJointPosDegree(0, ref currentJoint);
    if (currCode != 0)
    {
        Console.WriteLine($"GetActualJointPosDegree failed. err={currCode}");
        return;
    }

    if (config.PrecheckPathWithIk)
    {
        var precheckCode = PrecheckTrajectoryIkAndCollision(robot, config, currentJoint);
        if (precheckCode != 0)
        {
            Console.WriteLine("Trajectory precheck failed. Fix points/config and retry.");
            return;
        }
    }

    var loopIndex = 0;
    while (!stopRequested && (config.LoopCount == 0 || loopIndex < config.LoopCount))
    {
        loopIndex++;
        Console.WriteLine($"Loop #{loopIndex}");

        currCode = robot.GetActualJointPosDegree(0, ref currentJoint);
        if (currCode != 0)
        {
            Console.WriteLine($"GetActualJointPosDegree failed. err={currCode}");
            break;
        }

        for (var i = 0; i < config.Points.Count; i++)
        {
            if (HandleWebPanelState(robot, ref stopRequested, ref stopCommandSent))
            {
                break;
            }

            if (stopRequested)
            {
                RequestSafeStop(robot, "stop request before next point", ref stopCommandSent);
                break;
            }

            if (CheckCollisionState(robot, $"before point #{i + 1}", out var collisionErr))
            {
                Console.WriteLine($"Collision/safety stop detected {collisionErr}. Abort trajectory.");
                stopRequested = true;
                RequestSafeStop(robot, "collision/safety state before move", ref stopCommandSent);
                break;
            }

            var point = config.Points[i];
            var pose = new DescPose(point.X, point.Y, point.Z, point.Rx, point.Ry, point.Rz);

            JointPos targetJoint = new JointPos(new double[6]);
            var ikCode = robot.GetInverseKinRef(0, pose, currentJoint, ref targetJoint);
            if (ikCode != 0)
            {
                Console.WriteLine($"IK failed at point #{i + 1}. err={ikCode}. point=({point.X},{point.Y},{point.Z},{point.Rx},{point.Ry},{point.Rz}) refJ=({FormatJoint(currentJoint)})");
                stopRequested = true;
                break;
            }

            var isFirstPoint = i == 0;
            int moveCode;
            if (isFirstPoint)
            {
                moveCode = robot.MoveJ(targetJoint, pose, config.Tool, config.User, config.Velocity, config.Acceleration, config.Ovl, exaxis, -1, 0, pose);
            }
            else
            {
                moveCode = robot.MoveL(targetJoint, pose, config.Tool, config.User, config.Velocity, config.Acceleration, config.Ovl, config.BlendRadius, exaxis, 0, 0, pose);
            }

            if (moveCode != 0)
            {
                Console.WriteLine($"{(isFirstPoint ? "MoveJ" : "MoveL")} failed at point #{i + 1} ({point.X},{point.Y},{point.Z}), err={moveCode}");
                stopRequested = true;
                RequestSafeStop(robot, "move command failed", ref stopCommandSent);
                break;
            }

            currentJoint = targetJoint;

            if (config.WaitMotionDone)
            {
                byte done = 0;
                do
                {
                    if (HandleWebPanelState(robot, ref stopRequested, ref stopCommandSent))
                    {
                        break;
                    }

                    Thread.Sleep(config.MotionDonePollMs);
                    var doneCode = robot.GetRobotMotionDone(ref done);
                    if (doneCode != 0)
                    {
                        Console.WriteLine($"GetRobotMotionDone failed. err={doneCode}");
                        stopRequested = true;
                        RequestSafeStop(robot, "motion done polling failed", ref stopCommandSent);
                        break;
                    }

                    if (CheckCollisionState(robot, $"after point #{i + 1}", out collisionErr))
                    {
                        Console.WriteLine($"Collision/safety stop detected {collisionErr}. Abort trajectory.");
                        stopRequested = true;
                        RequestSafeStop(robot, "collision/safety state after move", ref stopCommandSent);
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

static int PrecheckTrajectoryIkAndCollision(Robot robot, TrajectoryConfig config, JointPos startJoint)
{
    Console.WriteLine("Run trajectory IK precheck...");

    var refJoint = startJoint;
    for (var i = 0; i < config.Points.Count; i++)
    {
        if (CheckCollisionState(robot, $"during precheck point #{i + 1}", out var collisionErr))
        {
            Console.WriteLine($"Precheck stopped by collision/safety state {collisionErr}");
            return -1;
        }

        var point = config.Points[i];
        var pose = new DescPose(point.X, point.Y, point.Z, point.Rx, point.Ry, point.Rz);

        var hasSolution = false;
        var hasSolutionCode = robot.GetInverseKinHasSolution(0, pose, refJoint, ref hasSolution);
        if (hasSolutionCode != 0)
        {
            Console.WriteLine($"GetInverseKinHasSolution failed at point #{i + 1}. err={hasSolutionCode}");
            return hasSolutionCode;
        }

        if (!hasSolution)
        {
            Console.WriteLine($"Precheck: IK has no solution at point #{i + 1}: ({point.X},{point.Y},{point.Z},{point.Rx},{point.Ry},{point.Rz}) with refJ=({FormatJoint(refJoint)})");
            return 112;
        }

        JointPos targetJoint = new JointPos(new double[6]);
        var ikCode = robot.GetInverseKinRef(0, pose, refJoint, ref targetJoint);
        if (ikCode != 0)
        {
            Console.WriteLine($"Precheck: GetInverseKinRef failed at point #{i + 1}. err={ikCode}");
            return ikCode;
        }

        refJoint = targetJoint;
    }

    Console.WriteLine("Trajectory IK precheck passed.");
    return 0;
}

static bool CheckCollisionState(Robot robot, string stage, out string stateSummary)
{
    var pkg = new ROBOT_STATE_PKG();
    var stateCode = robot.GetRobotRealTimeState(ref pkg);
    if (stateCode != 0)
    {
        stateSummary = $"GetRobotRealTimeState err={stateCode} at {stage}";
        return false;
    }

    if (pkg.collisionState == 1 || pkg.EmergencyStop == 1 || pkg.main_code != 0)
    {
        stateSummary = $"at {stage}: collisionState={pkg.collisionState}, estop={pkg.EmergencyStop}, main_code={pkg.main_code}, sub_code={pkg.sub_code}, robot_mode={pkg.robot_mode}, robot_state={pkg.robot_state}";
        return true;
    }

    stateSummary = string.Empty;
    return false;
}

static void RequestSafeStop(Robot robot, string reason, ref bool stopCommandSent)
{
    if (stopCommandSent)
    {
        return;
    }

    stopCommandSent = true;
    Console.WriteLine($"Request controlled stop. reason={reason}");

    var stopMotionCode = robot.StopMotion();
    Console.WriteLine($"StopMotion -> err={stopMotionCode}");

    var programStopCode = robot.ProgramStop();
    Console.WriteLine($"ProgramStop -> err={programStopCode}");
}

static bool HandleWebPanelState(Robot robot, ref bool stopRequested, ref bool stopCommandSent)
{
    var pkg = new ROBOT_STATE_PKG();
    var stateCode = robot.GetRobotRealTimeState(ref pkg);
    if (stateCode != 0)
    {
        return false;
    }

    // program_state: 1-stop, 2-run, 3-pause
    if (pkg.program_state == 1)
    {
        stopRequested = true;
        RequestSafeStop(robot, "web panel stop", ref stopCommandSent);
        return true;
    }

    if (pkg.program_state == 3)
    {
        Console.WriteLine("Web panel pause detected. Waiting for resume/stop...");
        while (true)
        {
            Thread.Sleep(100);
            var pollPkg = new ROBOT_STATE_PKG();
            var pollCode = robot.GetRobotRealTimeState(ref pollPkg);
            if (pollCode != 0)
            {
                Console.WriteLine($"Pause polling state read failed. err={pollCode}");
                stopRequested = true;
                RequestSafeStop(robot, "pause polling failed", ref stopCommandSent);
                return true;
            }

            if (pollPkg.program_state == 1)
            {
                stopRequested = true;
                RequestSafeStop(robot, "web panel stop while paused", ref stopCommandSent);
                return true;
            }

            if (pollPkg.program_state == 2)
            {
                Console.WriteLine("Web panel resume detected. Continue trajectory.");
                return false;
            }
        }
    }

    return false;
}

static string FormatJoint(JointPos joint)
{
    return string.Join(",", joint.jPos.Select(x => x.ToString("F3")));
}

static void PrintUsage()
{
    Console.WriteLine("Usage: dotnet run -- <robot_ip> [trajectory_file]");
    Console.WriteLine("Example with simulator IP: dotnet run -- 127.0.0.1 trajectory_infinity.json");
}

internal sealed class TrajectoryConfig
{
    public int Tool { get; set; } = 0;
    public int User { get; set; } = 0;
    public float Velocity { get; set; } = 20;
    public float Acceleration { get; set; } = 20;
    public float Ovl { get; set; } = 100;
    public float BlendRadius { get; set; } = 5;
    public bool WaitMotionDone { get; set; } = true;
    public int MotionDonePollMs { get; set; } = 30;
    public int LoopCount { get; set; } = 0;
    public bool PrecheckPathWithIk { get; set; } = true;
    public List<CartPoint> Points { get; set; } = new List<CartPoint>();
}

internal sealed class CartPoint
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
    public double Rx { get; set; }
    public double Ry { get; set; }
    public double Rz { get; set; }
}
