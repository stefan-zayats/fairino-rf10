using System.Globalization;
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

    DescPose currentTcp = new DescPose(0, 0, 0, 0, 0, 0);
    var tcpCode = robot.GetActualTCPPose(0, ref currentTcp);
    if (tcpCode == 0)
    {
        Console.WriteLine($"Current TCP: {FormatPose(currentTcp)}");
        Console.WriteLine($"Current joints: {FormatJoint(currentJoint)}");
    }
    else
    {
        Console.WriteLine($"GetActualTCPPose failed. err={tcpCode}. Continue with joints only.");
    }

    if (config.UseCurrentTcpOrientation && tcpCode == 0)
    {
        ApplyCurrentTcpOrientation(config, currentTcp);
        Console.WriteLine($"Use current TCP orientation for all points: rx={FormatNumber(currentTcp.rpy.rx)}, ry={FormatNumber(currentTcp.rpy.ry)}, rz={FormatNumber(currentTcp.rpy.rz)}");
    }

    var motionPlan = BuildMotionPlan(robot, config, currentJoint, currentTcp, tcpCode == 0);
    if (config.PrecheckPathWithIk && motionPlan.ErrorCode != 0)
    {
        Console.WriteLine("Trajectory precheck failed. Fix points/config and retry.");
        return;
    }

    var plannedPoints = motionPlan.Points;
    if (plannedPoints.Count < 2)
    {
        Console.WriteLine("Motion plan invalid. Need at least 2 planned points.");
        return;
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

        for (var i = 0; i < plannedPoints.Count; i++)
        {
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

            var plannedPoint = plannedPoints[i];
            var point = plannedPoint.Point;
            var pose = ToPose(point);

            JointPos targetJoint = new JointPos(new double[6]);
            var isFirstPoint = i == 0 && config.StartPointWithMoveJEachLoop;
            var ikCode = ResolveRuntimeIk(robot, pose, currentJoint, isFirstPoint, plannedPoint.Label, ref targetJoint);
            if (ikCode != 0)
            {
                Console.WriteLine($"IK failed at planned point #{i + 1} ({plannedPoint.Label}). err={ikCode}. point={FormatPoint(point)} refJ=({FormatJoint(currentJoint)})");
                stopRequested = true;
                break;
            }

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
                Console.WriteLine($"{(isFirstPoint ? "MoveJ" : "MoveL")} failed at planned point #{i + 1} ({plannedPoint.Label}) {FormatPoint(point)}, err={moveCode}");
                PrintRobotState(robot, "after failed move command");
                stopRequested = true;
                RequestSafeStop(robot, "move command failed", ref stopCommandSent);
                break;
            }

            currentJoint = targetJoint;
            Console.WriteLine($"Queued {(isFirstPoint ? "MoveJ" : "MoveL")} to planned point #{i + 1} ({plannedPoint.Label}) joints=({FormatJoint(targetJoint)})");

            if (config.WaitMotionDone)
            {
                byte done = 0;
                do
                {
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

static MotionPlanResult BuildMotionPlan(Robot robot, TrajectoryConfig config, JointPos startJoint, DescPose currentTcp, bool hasCurrentTcp)
{
    var result = new MotionPlanResult();
    result.Points.AddRange(config.Points.Select((point, index) => new PlannedPoint(ClonePoint(point), $"source #{index + 1}")));

    if (!config.PrecheckPathWithIk)
    {
        Console.WriteLine("Trajectory precheck disabled by config.");
        return result;
    }

    Console.WriteLine("Run trajectory IK/path precheck...");
    PrintRobotState(robot, "precheck start");

    var planned = new List<PlannedPoint>();
    var refJoint = startJoint;
    var previousPose = hasCurrentTcp ? currentTcp : ToPose(config.Points[0]);

    for (var i = 0; i < config.Points.Count; i++)
    {
        if (CheckCollisionState(robot, $"during precheck point #{i + 1}", out var collisionErr))
        {
            Console.WriteLine($"Precheck stopped by collision/safety state {collisionErr}");
            return MotionPlanResult.Failed(-1, planned);
        }

        var point = config.Points[i];
        var targetPose = ToPose(point);
        var targetLabel = $"source #{i + 1}";
        var firstPointUsesMoveJ = i == 0 && config.StartPointWithMoveJEachLoop;

        var allowFreeIkFallback = firstPointUsesMoveJ;
        if (!TrySolveIk(robot, targetPose, refJoint, targetLabel, allowFreeIkFallback, out var targetJoint, out var ikError))
        {
            PrintIkFixHints(point, firstPointUsesMoveJ);
            return MotionPlanResult.Failed(ikError, planned);
        }

        if (firstPointUsesMoveJ)
        {
            planned.Add(new PlannedPoint(ClonePoint(point), targetLabel));
            refJoint = targetJoint;
            previousPose = targetPose;
            continue;
        }

        var segmentCheck = CheckLinearSegment(robot, previousPose, targetPose, refJoint, targetLabel, config, out var segmentEndJoint);
        if (!segmentCheck.Ok && config.AutoViaZ.HasValue)
        {
            Console.WriteLine($"Precheck: direct LIN segment to {targetLabel} is not safe: {segmentCheck.Message}");
            Console.WriteLine($"Precheck: try alternate lifted path with autoViaZ={FormatNumber(config.AutoViaZ.Value)} mm");

            if (TryBuildViaPath(robot, previousPose, targetPose, refJoint, targetLabel, config, out var viaPoints, out segmentEndJoint))
            {
                planned.AddRange(viaPoints);
                refJoint = segmentEndJoint;
                previousPose = targetPose;
                continue;
            }
        }

        if (!segmentCheck.Ok)
        {
            Console.WriteLine($"Precheck: LIN segment to {targetLabel} rejected: {segmentCheck.Message}");
            Console.WriteLine("Precheck: robot may still reach the endpoint, but straight MoveL path is unsafe/unreachable. Reduce X/Y, change RX/RY/RZ, or set autoViaZ to try a lifted path.");
            return MotionPlanResult.Failed(segmentCheck.ErrorCode, planned);
        }

        planned.Add(new PlannedPoint(ClonePoint(point), targetLabel));
        refJoint = segmentEndJoint;
        previousPose = targetPose;
    }

    result.Points.Clear();
    result.Points.AddRange(planned);
    Console.WriteLine($"Trajectory IK/path precheck passed. plannedPoints={planned.Count}");
    return result;
}

static bool TryBuildViaPath(Robot robot, DescPose fromPose, DescPose targetPose, JointPos startJoint, string targetLabel, TrajectoryConfig config, out List<PlannedPoint> viaPoints, out JointPos endJoint)
{
    viaPoints = new List<PlannedPoint>();
    endJoint = startJoint;

    var viaStart = PoseToPoint(fromPose);
    viaStart.Z = config.AutoViaZ!.Value;
    var viaEnd = PoseToPoint(targetPose);
    viaEnd.Z = config.AutoViaZ.Value;

    var refJoint = startJoint;
    var previousPose = fromPose;
    var candidates = new[]
    {
        new PlannedPoint(viaStart, $"auto-via lift before {targetLabel}"),
        new PlannedPoint(viaEnd, $"auto-via travel before {targetLabel}"),
        new PlannedPoint(PoseToPoint(targetPose), targetLabel),
    };

    foreach (var candidate in candidates)
    {
        var candidatePose = ToPose(candidate.Point);
        if (DistanceMm(previousPose, candidatePose) < 0.001 && OrientationDistanceDeg(previousPose, candidatePose) < 0.001)
        {
            continue;
        }

        var check = CheckLinearSegment(robot, previousPose, candidatePose, refJoint, candidate.Label, config, out var candidateJoint);
        if (!check.Ok)
        {
            Console.WriteLine($"Precheck: alternate path failed at {candidate.Label}: {check.Message}");
            return false;
        }

        viaPoints.Add(new PlannedPoint(ClonePoint(candidate.Point), candidate.Label));
        refJoint = candidateJoint;
        previousPose = candidatePose;
    }

    endJoint = refJoint;
    Console.WriteLine($"Precheck: alternate lifted path accepted for {targetLabel}; inserted {viaPoints.Count} planned points.");
    return true;
}

static SegmentCheckResult CheckLinearSegment(Robot robot, DescPose fromPose, DescPose toPose, JointPos startJoint, string label, TrajectoryConfig config, out JointPos endJoint)
{
    endJoint = startJoint;

    var distance = DistanceMm(fromPose, toPose);
    var steps = Math.Max(1, (int)Math.Ceiling(distance / Math.Max(1, config.PathCheckStepMm)));
    var refJoint = startJoint;

    for (var step = 1; step <= steps; step++)
    {
        var t = (double)step / steps;
        var samplePose = LerpPose(fromPose, toPose, t);
        var sampleLabel = $"{label} sample {step}/{steps}";

        if (!TrySolveIk(robot, samplePose, refJoint, sampleLabel, allowFreeIkFallback: false, out var sampleJoint, out var ikError))
        {
            return SegmentCheckResult.Fail(ikError, $"IK has no solution at {sampleLabel}, pose={FormatPose(samplePose)}, refJ=({FormatJoint(refJoint)})");
        }

        var maxDelta = MaxJointDelta(refJoint, sampleJoint);
        if (maxDelta > config.MaxJointStepDeg)
        {
            return SegmentCheckResult.Fail(14, $"joint jump {FormatNumber(maxDelta)} deg > maxJointStepDeg={FormatNumber(config.MaxJointStepDeg)} at {sampleLabel}; fromJ=({FormatJoint(refJoint)}) toJ=({FormatJoint(sampleJoint)})");
        }

        refJoint = sampleJoint;
    }

    endJoint = refJoint;
    return SegmentCheckResult.Success();
}

static bool TrySolveIk(Robot robot, DescPose pose, JointPos refJoint, string label, bool allowFreeIkFallback, out JointPos targetJoint, out int errorCode)
{
    targetJoint = new JointPos(new double[6]);
    errorCode = 0;

    var hasSolution = false;
    var hasSolutionCode = robot.GetInverseKinHasSolution(0, pose, refJoint, ref hasSolution);
    if (hasSolutionCode != 0)
    {
        Console.WriteLine($"GetInverseKinHasSolution failed at {label}. err={hasSolutionCode}");
        errorCode = hasSolutionCode;
        return false;
    }

    if (hasSolution)
    {
        var ikCode = robot.GetInverseKinRef(0, pose, refJoint, ref targetJoint);
        if (ikCode == 0)
        {
            return true;
        }

        Console.WriteLine($"Precheck: GetInverseKinRef failed at {label}. err={ikCode}, pose={FormatPose(pose)}, refJ=({FormatJoint(refJoint)})");
        errorCode = ikCode;
    }
    else
    {
        Console.WriteLine($"Precheck: IK has no solution at {label} with current reference: pose={FormatPose(pose)} refJ=({FormatJoint(refJoint)})");
        errorCode = 112;
    }

    if (!allowFreeIkFallback)
    {
        return false;
    }

    var freeIkJoint = new JointPos(new double[6]);
    var freeIkCode = robot.GetInverseKin(0, pose, -1, ref freeIkJoint);
    if (freeIkCode != 0)
    {
        Console.WriteLine($"Precheck: free IK fallback also failed at {label}. err={freeIkCode}, pose={FormatPose(pose)}");
        errorCode = freeIkCode;
        return false;
    }

    targetJoint = freeIkJoint;
    Console.WriteLine($"Precheck: free IK fallback accepted at {label}; use MoveJ to switch configuration. targetJ=({FormatJoint(targetJoint)})");
    errorCode = 0;
    return true;
}


static int ResolveRuntimeIk(Robot robot, DescPose pose, JointPos currentJoint, bool isFirstPointMoveJ, string label, ref JointPos targetJoint)
{
    var ikCode = robot.GetInverseKinRef(0, pose, currentJoint, ref targetJoint);
    if (ikCode == 0 || !isFirstPointMoveJ)
    {
        return ikCode;
    }

    Console.WriteLine($"Runtime IK by current reference failed at first MoveJ point ({label}). err={ikCode}. Try free IK fallback.");
    var freeIkJoint = new JointPos(new double[6]);
    var freeIkCode = robot.GetInverseKin(0, pose, -1, ref freeIkJoint);
    if (freeIkCode == 0)
    {
        targetJoint = freeIkJoint;
    }

    return freeIkCode;
}

static void ApplyCurrentTcpOrientation(TrajectoryConfig config, DescPose currentTcp)
{
    foreach (var point in config.Points)
    {
        point.Rx = currentTcp.rpy.rx;
        point.Ry = currentTcp.rpy.ry;
        point.Rz = currentTcp.rpy.rz;
    }
}

static void PrintIkFixHints(CartPoint point, bool firstPointUsesMoveJ)
{
    Console.WriteLine($"Hint: endpoint is unreachable for the current joint reference: {FormatPoint(point)}");
    if (firstPointUsesMoveJ)
    {
        Console.WriteLine("Hint: even free IK could not find a first-point solution. The point/orientation is outside the robot workspace or conflicts with limits.");
    }
    else
    {
        Console.WriteLine("Hint: for MoveL this usually means the straight segment cannot keep one continuous joint configuration. Reduce X/Y, change RX/RY/RZ, or set autoViaZ.");
    }
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

    if (pkg.collisionState == 1 || pkg.EmergencyStop == 1 || pkg.main_code != 0 || pkg.sub_code != 0 || pkg.safety_stop0_state == 1 || pkg.safety_stop1_state == 1)
    {
        stateSummary = $"at {stage}: collisionState={pkg.collisionState}, estop={pkg.EmergencyStop}, safety0={pkg.safety_stop0_state}, safety1={pkg.safety_stop1_state}, main_code={pkg.main_code}, sub_code={pkg.sub_code}, robot_mode={pkg.robot_mode}, robot_state={pkg.robot_state}, motion_done={pkg.motion_done}, queue_len={pkg.mc_queue_len}";
        return true;
    }

    stateSummary = string.Empty;
    return false;
}

static void PrintRobotState(Robot robot, string stage)
{
    var pkg = new ROBOT_STATE_PKG();
    var stateCode = robot.GetRobotRealTimeState(ref pkg);
    if (stateCode != 0)
    {
        Console.WriteLine($"Robot state at {stage}: GetRobotRealTimeState err={stateCode}");
        return;
    }

    Console.WriteLine($"Robot state at {stage}: collisionState={pkg.collisionState}, estop={pkg.EmergencyStop}, safety0={pkg.safety_stop0_state}, safety1={pkg.safety_stop1_state}, main_code={pkg.main_code}, sub_code={pkg.sub_code}, robot_mode={pkg.robot_mode}, robot_state={pkg.robot_state}, motion_done={pkg.motion_done}, queue_len={pkg.mc_queue_len}");
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

static DescPose ToPose(CartPoint point)
{
    return new DescPose(point.X, point.Y, point.Z, point.Rx, point.Ry, point.Rz);
}

static CartPoint PoseToPoint(DescPose pose)
{
    return new CartPoint
    {
        X = pose.tran.x,
        Y = pose.tran.y,
        Z = pose.tran.z,
        Rx = pose.rpy.rx,
        Ry = pose.rpy.ry,
        Rz = pose.rpy.rz,
    };
}

static CartPoint ClonePoint(CartPoint point)
{
    return new CartPoint
    {
        X = point.X,
        Y = point.Y,
        Z = point.Z,
        Rx = point.Rx,
        Ry = point.Ry,
        Rz = point.Rz,
    };
}

static DescPose LerpPose(DescPose from, DescPose to, double t)
{
    return new DescPose(
        Lerp(from.tran.x, to.tran.x, t),
        Lerp(from.tran.y, to.tran.y, t),
        Lerp(from.tran.z, to.tran.z, t),
        Lerp(from.rpy.rx, to.rpy.rx, t),
        Lerp(from.rpy.ry, to.rpy.ry, t),
        Lerp(from.rpy.rz, to.rpy.rz, t));
}

static double Lerp(double from, double to, double t)
{
    return from + (to - from) * t;
}

static double DistanceMm(DescPose a, DescPose b)
{
    var dx = a.tran.x - b.tran.x;
    var dy = a.tran.y - b.tran.y;
    var dz = a.tran.z - b.tran.z;
    return Math.Sqrt(dx * dx + dy * dy + dz * dz);
}

static double OrientationDistanceDeg(DescPose a, DescPose b)
{
    return Math.Max(Math.Max(Math.Abs(a.rpy.rx - b.rpy.rx), Math.Abs(a.rpy.ry - b.rpy.ry)), Math.Abs(a.rpy.rz - b.rpy.rz));
}

static double MaxJointDelta(JointPos from, JointPos to)
{
    var max = 0.0;
    for (var i = 0; i < Math.Min(from.jPos.Length, to.jPos.Length); i++)
    {
        max = Math.Max(max, Math.Abs(to.jPos[i] - from.jPos[i]));
    }

    return max;
}

static string FormatJoint(JointPos joint)
{
    return string.Join("; ", joint.jPos.Select((x, i) => $"J{i + 1}={FormatNumber(x)}"));
}

static string FormatPose(DescPose pose)
{
    return $"x={FormatNumber(pose.tran.x)}, y={FormatNumber(pose.tran.y)}, z={FormatNumber(pose.tran.z)}, rx={FormatNumber(pose.rpy.rx)}, ry={FormatNumber(pose.rpy.ry)}, rz={FormatNumber(pose.rpy.rz)}";
}

static string FormatPoint(CartPoint point)
{
    return $"(x={FormatNumber(point.X)}, y={FormatNumber(point.Y)}, z={FormatNumber(point.Z)}, rx={FormatNumber(point.Rx)}, ry={FormatNumber(point.Ry)}, rz={FormatNumber(point.Rz)})";
}

static string FormatNumber(double value)
{
    return value.ToString("F3", CultureInfo.InvariantCulture);
}

static void PrintUsage()
{
    Console.WriteLine("Usage: dotnet run -- <robot_ip> [trajectory_file]");
    Console.WriteLine("Example with simulator IP: dotnet run -- 127.0.0.1 trajectory_infinity.json");
}

internal sealed class MotionPlanResult
{
    public int ErrorCode { get; set; }
    public List<PlannedPoint> Points { get; } = new List<PlannedPoint>();

    public static MotionPlanResult Failed(int errorCode, List<PlannedPoint> partialPlan)
    {
        var result = new MotionPlanResult { ErrorCode = errorCode };
        result.Points.AddRange(partialPlan);
        return result;
    }
}

internal sealed class SegmentCheckResult
{
    public bool Ok { get; private set; }
    public int ErrorCode { get; private set; }
    public string Message { get; private set; } = string.Empty;

    public static SegmentCheckResult Success()
    {
        return new SegmentCheckResult { Ok = true };
    }

    public static SegmentCheckResult Fail(int errorCode, string message)
    {
        return new SegmentCheckResult { Ok = false, ErrorCode = errorCode, Message = message };
    }
}

internal sealed class PlannedPoint
{
    public PlannedPoint(CartPoint point, string label)
    {
        Point = point;
        Label = label;
    }

    public CartPoint Point { get; }
    public string Label { get; }
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
    public bool StartPointWithMoveJEachLoop { get; set; } = true;
    public bool UseCurrentTcpOrientation { get; set; } = false;
    public double PathCheckStepMm { get; set; } = 25;
    public double MaxJointStepDeg { get; set; } = 45;
    public double? AutoViaZ { get; set; }
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
