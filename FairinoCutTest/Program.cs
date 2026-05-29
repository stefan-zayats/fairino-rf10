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

    var motionPlan = BuildMotionPlan(robot, config, currentJoint);
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

    if (!config.StartPointWithMoveJEachLoop)
    {
        Console.WriteLine("Move to trajectory start with MoveJ...");
        if (!QueueMove(robot, config, exaxis, plannedPoints[0], 0, currentJoint, moveJ: true, ref currentJoint, ref stopRequested, ref stopCommandSent, waitMotionDone: true))
        {
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

        var startIndex = !config.StartPointWithMoveJEachLoop && loopIndex == 1 ? 1 : 0;
        for (var i = startIndex; i < plannedPoints.Count; i++)
        {
            var moveJ = i == 0 && config.StartPointWithMoveJEachLoop;
            if (!QueueMove(robot, config, exaxis, plannedPoints[i], i, currentJoint, moveJ, ref currentJoint, ref stopRequested, ref stopCommandSent))
            {
                break;
            }
        }
    }

    if (!stopRequested && !config.WaitMotionDone)
    {
        WaitForQueuedMotionToFinish(robot, config, ref stopRequested, ref stopCommandSent);
    }
}
finally
{
    robot.CloseRPC();
    Console.WriteLine("RPC closed.");
}

static bool QueueMove(
    Robot robot,
    TrajectoryConfig config,
    ExaxisPos exaxis,
    PlannedPoint plannedPoint,
    int pointIndex,
    JointPos currentJoint,
    bool moveJ,
    ref JointPos nextJoint,
    ref bool stopRequested,
    ref bool stopCommandSent,
    bool? waitMotionDone = null)
{
    if (stopRequested)
    {
        RequestSafeStop(robot, "stop request before next point", ref stopCommandSent);
        return false;
    }

    if (CheckCollisionState(robot, $"before point #{pointIndex + 1}", out var collisionErr))
    {
        Console.WriteLine($"Collision/safety stop detected {collisionErr}. Abort trajectory.");
        stopRequested = true;
        RequestSafeStop(robot, "collision/safety state before move", ref stopCommandSent);
        return false;
    }

    if (!(waitMotionDone ?? config.WaitMotionDone) && !WaitForQueueCapacity(robot, config, $"before point #{pointIndex + 1}", ref stopRequested, ref stopCommandSent))
    {
        return false;
    }

    var pose = ToPose(plannedPoint.Point);
    var targetJoint = new JointPos(new double[6]);
    if (plannedPoint.HasJoint)
    {
        targetJoint = CloneJoint(plannedPoint.Joint);
    }
    else
    {
        var ikCode = ResolveRuntimeIk(robot, pose, currentJoint, moveJ, plannedPoint.Label, ref targetJoint);
        if (ikCode != 0)
        {
            Console.WriteLine($"IK failed at point #{pointIndex + 1} ({plannedPoint.Label}). err={ikCode}. point={FormatPoint(plannedPoint.Point)} refJ=({FormatJoint(currentJoint)})");
            stopRequested = true;
            return false;
        }
    }

    if (!CheckShoulderLevel(targetJoint, config, plannedPoint.Label, out var shoulderMessage))
    {
        Console.WriteLine($"Shoulder level limit rejected point #{pointIndex + 1} ({plannedPoint.Label}): {shoulderMessage}");
        stopRequested = true;
        RequestSafeStop(robot, "shoulder level limit", ref stopCommandSent);
        return false;
    }

    var moveCode = moveJ
        ? robot.MoveJ(targetJoint, pose, config.Tool, config.User, config.Velocity, config.Acceleration, config.Ovl, exaxis, -1, 0, pose)
        : robot.MoveL(targetJoint, pose, config.Tool, config.User, config.Velocity, config.Acceleration, config.Ovl, config.BlendRadius, exaxis, 0, 0, pose);

    var moveName = moveJ ? "MoveJ" : "MoveL";
    if (moveCode != 0)
    {
        Console.WriteLine($"{moveName} failed at point #{pointIndex + 1} ({plannedPoint.Label}) {FormatPoint(plannedPoint.Point)}, err={moveCode}");
        PrintRobotState(robot, "after failed move command");
        stopRequested = true;
        RequestSafeStop(robot, "move command failed", ref stopCommandSent);
        return false;
    }

    nextJoint = targetJoint;
    Console.WriteLine($"Queued {moveName} point #{pointIndex + 1} ({plannedPoint.Label})");

    if (!(waitMotionDone ?? config.WaitMotionDone))
    {
        return true;
    }

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
            return false;
        }

        if (CheckCollisionState(robot, $"after point #{pointIndex + 1}", out collisionErr))
        {
            Console.WriteLine($"Collision/safety stop detected {collisionErr}. Abort trajectory.");
            stopRequested = true;
            RequestSafeStop(robot, "collision/safety state after move", ref stopCommandSent);
            return false;
        }
    } while (!stopRequested && done == 0);

    return !stopRequested;
}

static MotionPlanResult BuildMotionPlan(Robot robot, TrajectoryConfig config, JointPos startJoint)
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

    if (CheckCollisionState(robot, "during precheck start", out var collisionErr))
    {
        Console.WriteLine($"Precheck stopped by collision/safety state {collisionErr}");
        return MotionPlanResult.Failed(-1, new List<PlannedPoint>());
    }

    var firstPoint = config.Points[0];
    var firstPose = ToPose(firstPoint);
    var firstCandidates = GetIkCandidates(robot, firstPose, startJoint, "source #1");
    if (firstCandidates.Count == 0)
    {
        PrintIkFixHints(firstPoint, true);
        return MotionPlanResult.Failed(112, new List<PlannedPoint>());
    }

    SegmentCheckResult? lastFailure = null;
    List<PlannedPoint>? bestPlan = null;
    IkCandidate? bestCandidate = null;
    var bestScore = double.MaxValue;

    foreach (var candidate in firstCandidates)
    {
        if (!CheckShoulderLevel(candidate.Joint, config, $"source #1, {candidate.Label}", out var shoulderMessage))
        {
            lastFailure = SegmentCheckResult.Fail(113, shoulderMessage);
            Console.WriteLine($"Precheck: first-point IK {candidate.Label} rejected by shoulder level limit: {shoulderMessage}");
            continue;
        }

        var planned = new List<PlannedPoint>
        {
            new PlannedPoint(ClonePoint(firstPoint), $"source #1, {candidate.Label}", CloneJoint(candidate.Joint))
        };

        if (TryAppendLinearPlan(robot, config, startIndex: 1, firstPose, candidate.Joint, planned, out lastFailure, out var linearTravelDeg))
        {
            var score = JointTravelDeg(startJoint, candidate.Joint) + linearTravelDeg;
            Console.WriteLine($"Precheck: first-point IK {candidate.Label} accepted; jointTravelScore={FormatNumber(score)} deg; firstJ=({FormatJoint(candidate.Joint)})");
            if (score < bestScore)
            {
                bestScore = score;
                bestPlan = planned;
                bestCandidate = candidate;
            }

            continue;
        }

        Console.WriteLine($"Precheck: first-point IK {candidate.Label} rejected: {lastFailure?.Message}");
    }

    if (bestPlan != null && bestCandidate != null)
    {
        result.Points.Clear();
        result.Points.AddRange(bestPlan);
        Console.WriteLine($"Precheck: selected first-point IK {bestCandidate.Label}; jointTravelScore={FormatNumber(bestScore)} deg; firstJ=({FormatJoint(bestCandidate.Joint)})");
        Console.WriteLine($"Trajectory IK/path precheck passed. plannedPoints={bestPlan.Count}");
        return result;
    }

    Console.WriteLine("Precheck: no first-point joint configuration can run the whole MoveL trajectory safely. Move J4/J5 closer to a working wrist configuration, reduce X/Y, or change RX/RY/RZ.");
    return MotionPlanResult.Failed(lastFailure?.ErrorCode ?? 14, new List<PlannedPoint>());
}

static bool TryAppendLinearPlan(
    Robot robot,
    TrajectoryConfig config,
    int startIndex,
    DescPose startPose,
    JointPos startJoint,
    List<PlannedPoint> planned,
    out SegmentCheckResult? failure,
    out double linearTravelDeg)
{
    failure = null;
    linearTravelDeg = 0;
    var previousPose = startPose;
    var refJoint = startJoint;

    for (var i = startIndex; i < config.Points.Count; i++)
    {
        if (CheckCollisionState(robot, $"during precheck point #{i + 1}", out var collisionErr))
        {
            Console.WriteLine($"Precheck stopped by collision/safety state {collisionErr}");
            failure = SegmentCheckResult.Fail(-1, collisionErr);
            return false;
        }

        var point = config.Points[i];
        var targetPose = ToPose(point);
        var targetLabel = $"source #{i + 1}";

        var segmentCheck = CheckLinearSegment(robot, previousPose, targetPose, refJoint, targetLabel, config, out var segmentEndJoint);
        if (!segmentCheck.Ok && config.AutoViaZ.HasValue)
        {
            Console.WriteLine($"Precheck: direct LIN segment to {targetLabel} is not safe: {segmentCheck.Message}");
            Console.WriteLine($"Precheck: try alternate lifted path with autoViaZ={FormatNumber(config.AutoViaZ.Value)} mm");

            if (TryBuildViaPath(robot, previousPose, targetPose, refJoint, targetLabel, config, out var viaPoints, out segmentEndJoint))
            {
                foreach (var viaPoint in viaPoints)
                {
                    if (viaPoint.HasJoint)
                    {
                        linearTravelDeg += JointTravelDeg(refJoint, viaPoint.Joint);
                        refJoint = viaPoint.Joint;
                    }
                }

                planned.AddRange(viaPoints);
                refJoint = segmentEndJoint;
                previousPose = targetPose;
                continue;
            }
        }

        if (!segmentCheck.Ok)
        {
            Console.WriteLine($"Precheck: LIN segment to {targetLabel} rejected: {segmentCheck.Message}");
            failure = segmentCheck;
            return false;
        }

        planned.Add(new PlannedPoint(ClonePoint(point), targetLabel, CloneJoint(segmentEndJoint)));
        linearTravelDeg += JointTravelDeg(refJoint, segmentEndJoint);
        refJoint = segmentEndJoint;
        previousPose = targetPose;
    }

    return true;
}

static List<IkCandidate> GetIkCandidates(Robot robot, DescPose pose, JointPos refJoint, string label)
{
    var candidates = new List<IkCandidate>();

    var refCandidate = new JointPos(new double[6]);
    var refCode = robot.GetInverseKinRef(0, pose, refJoint, ref refCandidate);
    if (refCode == 0)
    {
        AddIkCandidate(candidates, refCandidate, "current-ref IK");
    }
    else
    {
        Console.WriteLine($"Precheck: GetInverseKinRef failed at {label}. err={refCode}, pose={FormatPose(pose)}, refJ=({FormatJoint(refJoint)})");
    }

    for (var config = -1; config <= 7; config++)
    {
        var joint = new JointPos(new double[6]);
        var code = robot.GetInverseKin(0, pose, config, ref joint);
        if (code == 0)
        {
            AddIkCandidate(candidates, joint, $"config {config}");
        }
    }

    Console.WriteLine($"Precheck: first point has {candidates.Count} unique IK candidate(s). IK branch will be selected by checking the whole LIN path and shoulder-level constraint.");
    return candidates;
}

static void AddIkCandidate(List<IkCandidate> candidates, JointPos joint, string label)
{
    if (candidates.Any(candidate => MaxJointDelta(candidate.Joint, joint) < 0.001))
    {
        return;
    }

    candidates.Add(new IkCandidate(CloneJoint(joint), label));
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

        viaPoints.Add(new PlannedPoint(ClonePoint(candidate.Point), candidate.Label, CloneJoint(candidateJoint)));
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

        var sampleCheck = TrySolveLinearSampleIk(robot, samplePose, refJoint, sampleLabel, config, out var sampleJoint, out var ikError, out var ikMessage);
        if (!sampleCheck)
        {
            return SegmentCheckResult.Fail(ikError, ikMessage);
        }

        refJoint = sampleJoint;
    }

    endJoint = refJoint;
    return SegmentCheckResult.Success();
}

static bool TrySolveLinearSampleIk(
    Robot robot,
    DescPose pose,
    JointPos refJoint,
    string label,
    TrajectoryConfig config,
    out JointPos targetJoint,
    out int errorCode,
    out string message)
{
    targetJoint = new JointPos(new double[6]);
    errorCode = 112;
    message = $"IK has no solution at {label}, pose={FormatPose(pose)}, refJ=({FormatJoint(refJoint)})";

    var candidates = GetLinearSampleIkCandidates(robot, pose, refJoint, label, out var primaryErrorCode);
    if (candidates.Count == 0)
    {
        errorCode = primaryErrorCode;
        return false;
    }

    LinearSampleCandidate? bestCandidate = null;
    var bestScore = double.MaxValue;
    string? firstShoulderFailure = null;
    JointStepCheck? firstStepFailure = null;
    JointPos? firstStepFailureJoint = null;
    var hasShoulderAllowedCandidate = false;

    foreach (var candidate in candidates)
    {
        if (!CheckShoulderLevel(candidate.Joint, config, label, out var shoulderMessage))
        {
            if (firstShoulderFailure == null)
            {
                firstShoulderFailure = shoulderMessage;
            }

            continue;
        }

        hasShoulderAllowedCandidate = true;
        var jointStepCheck = CheckJointStep(refJoint, candidate.Joint, config);
        if (!jointStepCheck.Ok)
        {
            if (firstStepFailure == null)
            {
                firstStepFailure = jointStepCheck;
                firstStepFailureJoint = candidate.Joint;
            }

            continue;
        }

        var score = JointTravelDeg(refJoint, candidate.Joint);
        if (score < bestScore)
        {
            bestScore = score;
            bestCandidate = candidate;
        }
    }

    if (bestCandidate != null)
    {
        targetJoint = CloneJoint(bestCandidate.Joint);
        if (!bestCandidate.IsReferenceSolution)
        {
            Console.WriteLine($"Precheck: switched IK branch at {label} to keep constraints; selected {bestCandidate.Label}, targetJ=({FormatJoint(targetJoint)})");
        }

        errorCode = 0;
        message = string.Empty;
        return true;
    }

    if (firstStepFailure is { } stepFailure && firstStepFailureJoint is { } stepFailureJoint)
    {
        errorCode = 14;
        message = $"all IK branches exceed joint-step limits at {label}; first joint jump on {stepFailure.AxisName}: {FormatNumber(stepFailure.DeltaDeg)} deg > limit={FormatNumber(stepFailure.LimitDeg)}; delta=({FormatJointDelta(refJoint, stepFailureJoint)}); fromJ=({FormatJoint(refJoint)}) toJ=({FormatJoint(stepFailureJoint)})";
        return false;
    }

    if (!hasShoulderAllowedCandidate && firstShoulderFailure != null)
    {
        errorCode = 113;
        message = $"all IK branches violate shoulder level at {label}; first violation: {firstShoulderFailure}";
        return false;
    }

    return false;
}

static List<LinearSampleCandidate> GetLinearSampleIkCandidates(Robot robot, DescPose pose, JointPos refJoint, string label, out int primaryErrorCode)
{
    var candidates = new List<LinearSampleCandidate>();
    primaryErrorCode = 112;

    var hasSolution = false;
    var hasSolutionCode = robot.GetInverseKinHasSolution(0, pose, refJoint, ref hasSolution);
    if (hasSolutionCode != 0)
    {
        Console.WriteLine($"GetInverseKinHasSolution failed at {label}. err={hasSolutionCode}");
        primaryErrorCode = hasSolutionCode;
    }
    else if (hasSolution)
    {
        var refCandidate = new JointPos(new double[6]);
        var ikCode = robot.GetInverseKinRef(0, pose, refJoint, ref refCandidate);
        if (ikCode == 0)
        {
            AddLinearSampleCandidate(candidates, refCandidate, "current-ref IK", isReferenceSolution: true);
        }
        else
        {
            Console.WriteLine($"Precheck: GetInverseKinRef failed at {label}. err={ikCode}, pose={FormatPose(pose)}, refJ=({FormatJoint(refJoint)})");
            primaryErrorCode = ikCode;
        }
    }
    else
    {
        Console.WriteLine($"Precheck: IK has no solution at {label} with current reference: pose={FormatPose(pose)} refJ=({FormatJoint(refJoint)})");
    }

    for (var configIndex = -1; configIndex <= 7; configIndex++)
    {
        var joint = new JointPos(new double[6]);
        var code = robot.GetInverseKin(0, pose, configIndex, ref joint);
        if (code == 0)
        {
            AddLinearSampleCandidate(candidates, joint, $"config {configIndex}", isReferenceSolution: false);
        }
    }

    return candidates;
}

static void AddLinearSampleCandidate(List<LinearSampleCandidate> candidates, JointPos joint, string label, bool isReferenceSolution)
{
    if (candidates.Any(candidate => MaxJointDelta(candidate.Joint, joint) < 0.001))
    {
        return;
    }

    candidates.Add(new LinearSampleCandidate(CloneJoint(joint), label, isReferenceSolution));
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

static bool WaitForQueueCapacity(Robot robot, TrajectoryConfig config, string stage, ref bool stopRequested, ref bool stopCommandSent)
{
    if (config.MaxQueuedMotionSegments <= 0)
    {
        return true;
    }

    var printedWait = false;
    while (!stopRequested)
    {
        var pkg = new ROBOT_STATE_PKG();
        var stateCode = robot.GetRobotRealTimeState(ref pkg);
        if (stateCode != 0)
        {
            Console.WriteLine($"Queue throttle skipped at {stage}: GetRobotRealTimeState err={stateCode}");
            return true;
        }

        if (pkg.collisionState == 1 || pkg.EmergencyStop == 1 || pkg.main_code != 0 || pkg.sub_code != 0 || pkg.safety_stop0_state == 1 || pkg.safety_stop1_state == 1)
        {
            Console.WriteLine($"Collision/safety stop detected while waiting queue capacity at {stage}: collisionState={pkg.collisionState}, estop={pkg.EmergencyStop}, safety0={pkg.safety_stop0_state}, safety1={pkg.safety_stop1_state}, main_code={pkg.main_code}, sub_code={pkg.sub_code}, robot_mode={pkg.robot_mode}, robot_state={pkg.robot_state}, motion_done={pkg.motion_done}, queue_len={pkg.mc_queue_len}");
            stopRequested = true;
            RequestSafeStop(robot, "collision/safety state while waiting queue capacity", ref stopCommandSent);
            return false;
        }

        if (pkg.mc_queue_len < config.MaxQueuedMotionSegments)
        {
            return true;
        }

        if (!printedWait)
        {
            Console.WriteLine($"Queue throttle: queue_len={pkg.mc_queue_len} >= maxQueuedMotionSegments={config.MaxQueuedMotionSegments}. Wait before queuing more points. Press Ctrl+C to stop.");
            printedWait = true;
        }

        Thread.Sleep(Math.Max(1, config.QueuePollMs));
    }

    RequestSafeStop(robot, "stop request while waiting queue capacity", ref stopCommandSent);
    return false;
}

static bool WaitForQueuedMotionToFinish(Robot robot, TrajectoryConfig config, ref bool stopRequested, ref bool stopCommandSent)
{
    Console.WriteLine("All requested loops are queued. Wait for robot queue to finish. Press Ctrl+C to stop.");

    while (!stopRequested)
    {
        var pkg = new ROBOT_STATE_PKG();
        var stateCode = robot.GetRobotRealTimeState(ref pkg);
        if (stateCode != 0)
        {
            Console.WriteLine($"Wait for queued motion failed: GetRobotRealTimeState err={stateCode}");
            stopRequested = true;
            RequestSafeStop(robot, "queued motion state polling failed", ref stopCommandSent);
            return false;
        }

        if (pkg.collisionState == 1 || pkg.EmergencyStop == 1 || pkg.main_code != 0 || pkg.sub_code != 0 || pkg.safety_stop0_state == 1 || pkg.safety_stop1_state == 1)
        {
            Console.WriteLine($"Collision/safety stop detected while waiting queued motion: collisionState={pkg.collisionState}, estop={pkg.EmergencyStop}, safety0={pkg.safety_stop0_state}, safety1={pkg.safety_stop1_state}, main_code={pkg.main_code}, sub_code={pkg.sub_code}, robot_mode={pkg.robot_mode}, robot_state={pkg.robot_state}, motion_done={pkg.motion_done}, queue_len={pkg.mc_queue_len}");
            stopRequested = true;
            RequestSafeStop(robot, "collision/safety state while waiting queued motion", ref stopCommandSent);
            return false;
        }

        if (pkg.mc_queue_len == 0 && pkg.motion_done == 1)
        {
            Console.WriteLine("Queued motion finished.");
            return true;
        }

        Thread.Sleep(Math.Max(1, config.QueuePollMs));
    }

    RequestSafeStop(robot, "stop request while waiting queued motion", ref stopCommandSent);
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

static JointPos CloneJoint(JointPos joint)
{
    return new JointPos(joint.jPos.ToArray());
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

static bool CheckShoulderLevel(JointPos joint, TrajectoryConfig config, string label, out string message)
{
    message = string.Empty;
    if (config.AllowShoulderBelowLevel || joint.jPos.Length < 2)
    {
        return true;
    }

    var joint2 = joint.jPos[1];
    var isBelowLevel = config.ShoulderBelowLevelWhenJ2Less
        ? joint2 < config.ShoulderLevelJ2Deg
        : joint2 > config.ShoulderLevelJ2Deg;

    if (!isBelowLevel)
    {
        return true;
    }

    var comparator = config.ShoulderBelowLevelWhenJ2Less ? "<" : ">";
    message = $"{label}: J2={FormatNumber(joint2)} deg {comparator} shoulderLevelJ2Deg={FormatNumber(config.ShoulderLevelJ2Deg)} deg; allowShoulderBelowLevel=false";
    return false;
}

static JointStepCheck CheckJointStep(JointPos from, JointPos to, TrajectoryConfig config)
{
    var count = Math.Min(from.jPos.Length, to.jPos.Length);
    for (var i = 0; i < count; i++)
    {
        var delta = Math.Abs(to.jPos[i] - from.jPos[i]);
        var limit = GetMaxJointStepDeg(config, i);
        if (delta > limit)
        {
            return JointStepCheck.Fail($"J{i + 1}", delta, limit);
        }
    }

    return JointStepCheck.Success();
}

static double GetMaxJointStepDeg(TrajectoryConfig config, int jointIndex)
{
    if (config.MaxJointStepDegByAxis != null && jointIndex < config.MaxJointStepDegByAxis.Length && config.MaxJointStepDegByAxis[jointIndex] > 0)
    {
        return config.MaxJointStepDegByAxis[jointIndex];
    }

    return config.MaxJointStepDeg;
}

static double JointTravelDeg(JointPos from, JointPos to)
{
    var total = 0.0;
    for (var i = 0; i < Math.Min(from.jPos.Length, to.jPos.Length); i++)
    {
        total += Math.Abs(to.jPos[i] - from.jPos[i]);
    }

    return total;
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

static string FormatJointDelta(JointPos from, JointPos to)
{
    return string.Join("; ", to.jPos.Take(Math.Min(from.jPos.Length, to.jPos.Length)).Select((x, i) => $"J{i + 1}={FormatNumber(Math.Abs(x - from.jPos[i]))}"));
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

internal sealed class JointStepCheck
{
    public bool Ok { get; private set; }
    public string AxisName { get; private set; } = string.Empty;
    public double DeltaDeg { get; private set; }
    public double LimitDeg { get; private set; }

    public static JointStepCheck Success()
    {
        return new JointStepCheck { Ok = true };
    }

    public static JointStepCheck Fail(string axisName, double deltaDeg, double limitDeg)
    {
        return new JointStepCheck { Ok = false, AxisName = axisName, DeltaDeg = deltaDeg, LimitDeg = limitDeg };
    }
}

internal sealed class PlannedPoint
{
    public PlannedPoint(CartPoint point, string label, JointPos? joint = null)
    {
        Point = point;
        Label = label;
        Joint = new JointPos(new double[6]);
        HasJoint = false;
    }

    public PlannedPoint(CartPoint point, string label, JointPos joint)
    {
        Point = point;
        Label = label;
        Joint = joint;
        HasJoint = true;
    }

    public CartPoint Point { get; }
    public string Label { get; }
    public JointPos Joint { get; }
    public bool HasJoint { get; }
}

internal sealed class IkCandidate
{
    public IkCandidate(JointPos joint, string label)
    {
        Joint = joint;
        Label = label;
    }

    public JointPos Joint { get; }
    public string Label { get; }
}

internal sealed class LinearSampleCandidate
{
    public LinearSampleCandidate(JointPos joint, string label, bool isReferenceSolution)
    {
        Joint = joint;
        Label = label;
        IsReferenceSolution = isReferenceSolution;
    }

    public JointPos Joint { get; }
    public string Label { get; }
    public bool IsReferenceSolution { get; }
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
    public int QueuePollMs { get; set; } = 30;
    public int MaxQueuedMotionSegments { get; set; } = 20;
    public int LoopCount { get; set; } = 0;
    public bool PrecheckPathWithIk { get; set; } = true;
    public bool StartPointWithMoveJEachLoop { get; set; } = true;
    public bool UseCurrentTcpOrientation { get; set; } = false;
    public double PathCheckStepMm { get; set; } = 25;
    public bool AllowShoulderBelowLevel { get; set; } = true;
    public double ShoulderLevelJ2Deg { get; set; } = 0;
    public bool ShoulderBelowLevelWhenJ2Less { get; set; } = true;
    public double MaxJointStepDeg { get; set; } = 45;
    public double[]? MaxJointStepDegByAxis { get; set; }
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
