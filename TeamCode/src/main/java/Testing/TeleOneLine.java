package Testing;import com.acmerobotics.dashboard.FtcDashboard;import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;import com.qualcomm.robotcore.eventloop.opmode.TeleOp;import com.qualcomm.robotcore.hardware.DcMotor;import com.qualcomm.robotcore.hardware.DcMotorEx;import com.qualcomm.robotcore.hardware.HardwareMap;import com.qualcomm.robotcore.util.ElapsedTime;import CompCode.TELEmap;@Disabled@TeleOp(name="oneline") public class TeleOneLine extends LinearOpMode { public DcMotorEx liftMotor;TELEmap robot   = new TELEmap();ElapsedTime toggleTimer = new ElapsedTime();double toggleTime = .25;double SpeedAdjust = 1;double SSVar = 5;int dropVar = 50;int liftVar = 350;public static double p = .007, i = 0, d = 0;public static double f = .06;public static int LiftTarget = 0;public static int START_POS = 230;public static int LOW = 1208;public static int MID = 2078;public static int HIGH = 2900;public static int CONE5START = 655;public static int CONE5 = 520;public static double INTPOWER = 0;public static int INT = 1;public static double INTLOW = .2;public static int INTOFF = 0;public static int  INTOUT= -1;private final double ticks_in_degree = 700/180.0;double waitTime1 = .3;ElapsedTime waitTimer1 = new ElapsedTime();public PIDController controller;private DcMotorEx larm;private DcMotorEx rarm;private DcMotorEx rin;private DcMotorEx lin;enum LiftState {LIFT_DOWN, INTAKE, LIFT_UP, LIFT_START, IDLE}enum IntakeState {INTAKE_IN, INTAKE_OUT, INTAKE_OFF}LiftState liftState = LiftState.IDLE;IntakeState intState = IntakeState.INTAKE_OFF;@Override public void runOpMode() throws InterruptedException {robot.init(hardwareMap);Lift lift = new Lift(hardwareMap);Intake intake = new Intake(hardwareMap);waitForStart();if (isStopRequested()) return;LiftTarget = START_POS;INTPOWER = INTOFF;liftState = LiftState.LIFT_START;intState = IntakeState.INTAKE_OFF;while (opModeIsActive() && !isStopRequested()) {switch (liftState) {case LIFT_START: if (gamepad1.right_trigger==1) {INTPOWER = INT;waitTimer1.reset();liftState = LiftState.INTAKE;intState = IntakeState.INTAKE_IN;}break; case INTAKE: if (waitTimer1.seconds() >= waitTime1) {dropVar = robot.larm.getCurrentPosition() - 180;LiftTarget = dropVar;liftState = LiftState.LIFT_DOWN;}break; case LIFT_DOWN: if (Math.abs(larm.getCurrentPosition() - dropVar) < 20) {liftVar = robot.larm.getCurrentPosition() + 380;LiftTarget = liftVar;liftState = LiftState.LIFT_UP;}break; case LIFT_UP:if (Math.abs(larm.getCurrentPosition() - liftVar) < 20) {liftState = LiftState.LIFT_START;INTPOWER = INTOFF;intState = IntakeState.INTAKE_OFF;}break;}switch (intState) {case INTAKE_OFF: if (gamepad1.left_trigger ==1 ) {INTPOWER = INTOUT;intState = IntakeState.INTAKE_OUT;}break; case INTAKE_IN: break; case INTAKE_OUT: if (gamepad1.left_trigger!=1 ) {INTPOWER = 0;intState = IntakeState.INTAKE_OFF;}break;}if (gamepad2.dpad_up) {LiftTarget = HIGH;} else if (gamepad2.dpad_down) {LiftTarget = LOW;} else if (gamepad2.dpad_right) {LiftTarget = MID;} else if (gamepad2.dpad_left) {LiftTarget = MID;} else if (gamepad2.triangle) {if (SSVar == 5 ) {LiftTarget = 655;} else if (SSVar == 4) {LiftTarget = 550;} else if (SSVar == 3){LiftTarget = 450;} else if (SSVar==2){LiftTarget = 350;} else if (SSVar==1){LiftTarget = 230;}}if (gamepad2.cross) {LiftTarget = START_POS;} else if(gamepad1.square) {LiftTarget = 0;} else if (gamepad2.square) {LiftTarget = 10;}if (gamepad2.right_bumper){toggleTimer.reset();if (SSVar <5) {SSVar++;}while (opModeIsActive() && (toggleTimer.seconds() < toggleTime)) {}} else if (gamepad2.left_bumper){toggleTimer.reset();if (SSVar >1) {SSVar--;}while (opModeIsActive() && (toggleTimer.seconds() < toggleTime)) {}}robot.leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);robot.rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);robot.leftBack.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);robot.rightBack.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);if (gamepad1.left_bumper) {SpeedAdjust = 4;} else if (gamepad1.right_bumper) {SpeedAdjust = 1;}lift.update();intake.update();telemetry.addData("SS", SSVar);telemetry.update();}}class Lift { public Lift(HardwareMap hardwareMap) {controller = new PIDController(p, i, d);telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());larm = hardwareMap.get(DcMotorEx.class,"la");rarm = hardwareMap.get(DcMotorEx.class,"ra");larm.setDirection(DcMotorEx.Direction.FORWARD);rarm.setDirection(DcMotorEx.Direction.REVERSE);larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}public void update() {controller.setPID(p, i, d);int larmPos = larm.getCurrentPosition();int rarmPos = rarm.getCurrentPosition();double Lpid = controller.calculate(larmPos, LiftTarget);double Rpid = controller.calculate(rarmPos, LiftTarget);double Lff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f;double Rff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f;double Lpower = Lpid + Lff;double Rpower = Rpid + Rff;larm.setPower(Lpower);rarm.setPower(Rpower);telemetry.addData("SSVar", SSVar);telemetry.update();}}class Intake { public Intake(HardwareMap hardwareMap){rin = hardwareMap.get(DcMotorEx.class,"rin");lin = hardwareMap.get(DcMotorEx.class,"lin");lin.setDirection(DcMotor.Direction.REVERSE);rin.setDirection(DcMotor.Direction.FORWARD);}public void update() {lin.setPower(INTPOWER);rin.setPower(INTPOWER);}}}