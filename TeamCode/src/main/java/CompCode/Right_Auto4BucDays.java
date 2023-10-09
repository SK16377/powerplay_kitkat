package CompCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import AprilTagsss.AprilTagDetectionPipeline;
import Testing.PoseStorage;

//import Tessts.PoseStorage;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(name = "RRIGHT_AUTO BUC DAYS" , group = "COMP")
public class Right_Auto4BucDays extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;


    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    public static ColorSensor L_color;
    public static ColorSensor C_color;
    public static ColorSensor R_color;


    int ID_TAG_OF_INTEREST1 = 0; // Tag ID 18 from the 36h11 family
    int ID_TAG_OF_INTEREST2 = 1;
    int ID_TAG_OF_INTEREST3 = 2;


    public PIDController controller;

    //TODO: edit variables on dashboard lol

    // p = increase if not reaching target position

    public static double p = .006, i = 0, d = 0; // d = dampener (dampens arm movement and is scary). ignore i
    public static double f = .05;  // prevents arm from falling from gravity

    public static int LiftTarget = 0; // target position


    public static int LOW = 0; //1208 = LOW
    public static int MID = 195; //2078 = MID
    public static int HIGH = 500; //2900 = HIGH


    double OPEN = .01;
    double CLOSE = 0.1376;

    double startPos = 0.69;
    double flipPos = 0.01;

    public static int gray = 1;
    public static int notGray = 0;

    public static int left = 1;
    public static int center = 1;

    public static int right = 1;




    //public static int Rtarget = 0;
    // private final double ticks_in_degree = 700/180.0;

    //VoltageSensor voltageSensor;

    private DcMotorEx larm;
    private DcMotorEx rarm;



    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    AprilTagDetection tagOfInterest = null;

    /*GENERAL IDEA
     * go forward while lifting the lift
     * turn
     * bring out arm
     * drop cone  (VERY IMPORTANT THAT THESE ARE SEPARATE)
     * bring arm back in and  drop lift to cone 5 while doing spline
     * go towards cone
     * grab
     * lift and go to deliver trajectory
     * flip arm
     *
     * */

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        START,
        TURN_1,
        WAIT_1,
        FLIP1,
        SPLINE1,
        CONES1,
        SLOWCONES1,
        STRAFE_ADJ1,
        LIFT1,
        GRAB1,
        DELIVER1,
        DROP1,
        CONES2,
        SLOWCONES2,
        STRAFE_ADJ2,
        LIFT2,
        GRAB2,
        DELIVER2,
        DROP2,
        CONES3,
        SLOWCONES3,
        STRAFE_ADJ3,
        LIFT3,
        GRAB3,
        DELIVER3,
        DROP3,
        CONES4,
        SLOWCONES4,
        STRAFE_ADJ4,
        LIFT4,
        GRAB4,
        DELIVER4,
        DROP4,
        PARK,
        IDLE// Our bot will enter the IDLE state when done
    }

    enum elbowUpState {
        START,
        GRAB,
        LIFT_TWIST,
        OUTTAKE,
        IDLE
    }
    enum elbowDownState {
        START,
        CLOSE,
        LIFT_TWIST,
        OUTTAKE,
        IDLE
    }
    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;
    elbowUpState elbowUp = elbowUpState.IDLE;
    elbowDownState elbowDown = elbowDownState.IDLE;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);



        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });


        Lift lift = new Lift(hardwareMap);
        colorS colorS = new colorS(hardwareMap);



        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FLIPmap robot = new FLIPmap();

        // Set inital pose
        Pose2d startPose = new Pose2d(35, -64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory START = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(37.5,-18.00, Math.toRadians(348.00)))
                .build();

//        Trajectory LINE1 = drive.trajectoryBuilder(START.end())
//                .lineToLinearHeading(new Pose2d(36.61, -8.71, Math.toRadians(0.00)))
//                .build();
//
//        Trajectory CONES1 = drive.trajectoryBuilder(LINE1.end())
//                .lineToLinearHeading(new Pose2d(60.00, -12.00, Math.toRadians(0.00)))
//                .build();

        TrajectorySequence LINE_AND_CONES1 = drive.trajectorySequenceBuilder(START.end())
                .lineToLinearHeading(new Pose2d(36.61, -7.50, Math.toRadians(0.00)))
                .lineToLinearHeading(new Pose2d(65.50, -10.00, Math.toRadians(0.00)))
                .build();
//        Trajectory SLOWCONES1 = drive.trajectoryBuilder(CONES1.end())
//                .lineToLinearHeading(new Pose2d(63.50,-17.00,Math.toRadians(0.0)))
//                .build();
        Trajectory Strafe_SEESRIGHTONLY = drive.trajectoryBuilder(LINE_AND_CONES1.end())
                .strafeRight(5)
                .build();
        Trajectory Strafe_SEESRIGHTANDCENTER = drive.trajectoryBuilder(LINE_AND_CONES1.end())
                .strafeRight(5)
                .build();
        Trajectory Strafe_SEESLEFTANDCENTER = drive.trajectoryBuilder(LINE_AND_CONES1.end())
                .strafeLeft(5.0)
                .build();
        Trajectory Strafe_SEESLEFTONLY = drive.trajectoryBuilder(LINE_AND_CONES1.end())
                .strafeLeft(5.0)
                .build();


        Pose2d newPose1 = new Pose2d(63.50, -10,LINE_AND_CONES1.end().getHeading());

        //drive.setPoseEstimate(newPose1);
        //TODO RELOCALAIZE
        Trajectory DELIVER1 = drive.trajectoryBuilder(newPose1,true)
                .splineTo(new Vector2d(34.00, -16.50), Math.toRadians(-152.48))
                .build();

        Trajectory CONES2 = drive.trajectoryBuilder(DELIVER1.end())
                .splineTo(new Vector2d(64.00, -10), Math.toRadians(0.00))
                .build();

        Pose2d newPose2 = new Pose2d(63.50, -10,CONES2.end().getHeading());

        Trajectory DELIVER2 = drive.trajectoryBuilder(newPose2,true)
                .splineTo(new Vector2d(34.00, -15.50), Math.toRadians(-152.48))
                .build();

        Trajectory CONES3 = drive.trajectoryBuilder(DELIVER2.end())
                .splineTo(new Vector2d(63.50, -11.5), Math.toRadians(0.00))
                .build();

        Pose2d newPose3 = new Pose2d(63.50, -10,CONES3.end().getHeading());


        Trajectory DELIVER3 = drive.trajectoryBuilder(newPose3,true)
                .splineTo(new Vector2d(34.00, -15.2), Math.toRadians(-152.48))
                .build();

        Trajectory CONES4 = drive.trajectoryBuilder(DELIVER2.end())
                .splineTo(new Vector2d(63.50, -10.00), Math.toRadians(0.00))
                .build();

        Pose2d newPose4 = new Pose2d(63.50, -10,CONES4.end().getHeading());

        Trajectory DELIVER4 = drive.trajectoryBuilder(newPose4,true)
                .splineTo(new Vector2d(34.00, -12.500), Math.toRadians(-152.48))
                .build();


        TrajectorySequence PARK_C = drive.trajectorySequenceBuilder(DELIVER4.end())
                .splineTo(new Vector2d(60, -10.50), Math.toRadians(0.00))
               // .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence PARK_B = drive.trajectorySequenceBuilder(DELIVER4.end())
                .forward(2.5)
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence PARK_A = drive.trajectorySequenceBuilder(DELIVER4.end())
                .lineTo(new Vector2d(36, -0))
                .lineToLinearHeading(new Pose2d(8.00,-8.00,Math.toRadians(0)))
                .turn(Math.toRadians(-90))
                .build();

//        Trajectory PARK_A = drive.trajectoryBuilder(DELIVER4.end())
//                .lineTo(new Vector2d(56, -3))
//                .build();
//        Trajectory PARK_A2 = drive.trajectoryBuilder(PARK_A.end())
//                .lineToLinearHeading(new Pose2d(14,-7.00,Math.toRadians(0)))
//                .build();





        // Define a 1.5 second wait time
        double upTime = .5;
        double upTime1 = 1.2;
        ElapsedTime upTimer = new ElapsedTime();

        double downTime = .5;
        double downTime1 = 1.0;
        ElapsedTime downTimer = new ElapsedTime();

        double waitTime1 = .5;
        ElapsedTime waitTimer = new ElapsedTime();

        double waitTime2 = .65;

        double waitTime3 = 0.5;

        telemetry.setMsTransmissionInterval(50);

        robot.init(hardwareMap);





        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                double tagFound = 0;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST1)
                    {
                        /** tag id 0 is detected**/
                        telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
                        tagOfInterest = tag;
                        tagFound = 1;
                        break;
                    }
                    else if(tag.id == ID_TAG_OF_INTEREST2)
                    {
                        /** tag id 1 is detected**/
                        telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
                        tagOfInterest = tag;
                        tagFound = 2;
                        break;
                    }
                    else if(tag.id == ID_TAG_OF_INTEREST3)
                    {
                        /** tag id 2 is detected**/
                        telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
                        tagOfInterest = tag;
                        tagFound = 3;
                        break;
                    }
                }

                if(tagFound == 1)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else if(tagFound == 2)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else if(tagFound == 3)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);

            //initialize the servos
            robot.relbow.setPosition(0.05);
            robot.lelbow.setPosition(0.05);
            robot.twist.setPosition(startPos);
            robot.grab.setPosition(CLOSE);
        }

        waitForStart();

        if (isStopRequested()) return;

//program is running now, these are starting commands
        drive.followTrajectoryAsync(START);
        LiftTarget = MID;
        robot.grab.setPosition(CLOSE);
        robot.twist.setPosition(startPos);
        upTimer.reset();
        elbowUp = elbowUpState.START;
        currentState = State.TURN_1;
        elbowDown = elbowDownState.IDLE;

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            if (isStopRequested()) return;
            // We essentially define the flow of the state machine through this switch statement

            switch (currentState) {
                case TURN_1:
                    if (!drive.isBusy()) {
                        //  drive.followTrajectoryAsync(START1);
                        robot.grab.setPosition(OPEN);
                        currentState = State.WAIT_1;
                    }
                    break;
                case WAIT_1:
                    if (elbowUp == elbowUpState.IDLE) {
                        currentState = State.FLIP1;
                        upTime1 = .67;
                        waitTimer.reset();
                    }
                    break;
                case FLIP1:
                    if ((waitTimer.seconds() >= waitTime1) && !drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(LINE_AND_CONES1);
                        //drive.followTrajectoryAsync(LINE1);
                        LiftTarget = 170;//cone 5
                        //robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        downTimer.reset();
                        elbowDown = elbowDownState.START;
                        currentState = State.CONES1;
                    }
                    break;
//                case SPLINE1:
//                    if (!drive.isBusy()) {
//                        drive.followTrajectoryAsync(CONES1);
//                        currentState = State.CONES1;
//                    }
//                    break;
                case CONES1:
                    if (!drive.isBusy()) {
                        //drive.followTrajectoryAsync(SLOWCONES1);
                        if (left == gray && center == gray && right == notGray){
                            //aka left + center see tile and right sees tape
                            drive.followTrajectoryAsync(Strafe_SEESRIGHTONLY);
                            drive.setPoseEstimate(newPose1);
                            currentState = State.STRAFE_ADJ1;

                        } else if (left == gray && center == notGray && right == notGray) {
                            //aka left sees tile and center + right see tape
                            drive.followTrajectoryAsync(Strafe_SEESRIGHTANDCENTER);
                            drive.setPoseEstimate(newPose1);
                            currentState = State.STRAFE_ADJ1;

                        } else if (left == notGray && center == notGray && right == notGray) {
                            //aka left + center + right see tape
                            drive.setPoseEstimate(newPose1);
                            currentState = State.STRAFE_ADJ1;

                        } else if (left == notGray && center == notGray && right == gray) {
                            //aka left + center see tape and right sees tile
                            drive.followTrajectoryAsync(Strafe_SEESLEFTANDCENTER);
                            drive.setPoseEstimate(newPose1);
                            currentState = State.STRAFE_ADJ1;

                        } else if (left == notGray && center == gray && right == gray) {
                            //aka left sees tape and center + right see tile
                            drive.followTrajectoryAsync(Strafe_SEESLEFTONLY);
                            drive.setPoseEstimate(newPose1);
                            currentState = State.STRAFE_ADJ1;
                        } else {
                            drive.setPoseEstimate(newPose1);
                            currentState = State.STRAFE_ADJ1;
                        }
                        //currentState = State.STRAFE_ADJ;
                    }
                    break;
                case STRAFE_ADJ1:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        upTimer.reset();
                        //LiftTarget = HIGH;
                        waitTimer.reset();
                        currentState = State.LIFT1;
                    }
                    break;
                case LIFT1:
                    if (waitTimer.seconds()>=waitTime1) {

                        drive.followTrajectoryAsync(DELIVER1);
                        currentState = State.GRAB1;
                        elbowUp = elbowUpState.START;
                        waitTimer.reset();
                    }
                    break;
                case GRAB1:
                    if (waitTimer.seconds()>=waitTime2) {
                        LiftTarget = MID;

                        currentState = State.DELIVER1;
                    }
                    break;
                case DELIVER1:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(OPEN);
                        waitTimer.reset();
                        currentState = State.DROP1;
                    }
                    break;
                case DROP1:
                    if (waitTimer.seconds()>=waitTime1) {
                        drive.followTrajectoryAsync(CONES2);
                        LiftTarget = 125;//cone 4
                        //robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        downTime1 = 0.5;
                        downTimer.reset();
                        elbowDown = elbowDownState.START;
                        currentState = State.CONES2;
                    }
                    break;
                case CONES2:
                    if (!drive.isBusy()) {
                        //drive.followTrajectoryAsync(SLOWCONES1);
                        if (left == gray && center == gray && right == notGray){
                            //aka left + center see tile and right sees tape
                            drive.followTrajectoryAsync(Strafe_SEESRIGHTONLY);
                            drive.setPoseEstimate(newPose2);
                            currentState = State.STRAFE_ADJ2;

                        } else if (left == gray && center == notGray && right == notGray) {
                            //aka left sees tile and center + right see tape
                            drive.followTrajectoryAsync(Strafe_SEESRIGHTANDCENTER);
                            drive.setPoseEstimate(newPose2);
                            currentState = State.STRAFE_ADJ2;

                        } else if (left == notGray && center == notGray && right == notGray) {
                            //aka left + center + right see tape
                            drive.setPoseEstimate(newPose2);
                            currentState = State.STRAFE_ADJ2;

                        } else if (left == notGray && center == notGray && right == gray) {
                            //aka left + center see tape and right sees tile
                            drive.followTrajectoryAsync(Strafe_SEESLEFTANDCENTER);
                            drive.setPoseEstimate(newPose2);
                            currentState = State.STRAFE_ADJ2;

                        } else if (left == notGray && center == gray && right == gray) {
                            //aka left sees tape and center + right see tile
                            drive.followTrajectoryAsync(Strafe_SEESLEFTONLY);
                            drive.setPoseEstimate(newPose2);
                            currentState = State.STRAFE_ADJ2;
                        } else {
                            drive.setPoseEstimate(newPose2);
                            currentState = State.STRAFE_ADJ2;
                        }
                        //currentState = State.STRAFE_ADJ;
                    }
                    break;
                case STRAFE_ADJ2:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        upTimer.reset();
                        waitTimer.reset();
                        currentState = State.LIFT2;
                    }
                    break;
                case LIFT2:
                    if (waitTimer.seconds()>=waitTime1) {
                        waitTimer.reset();

                        drive.followTrajectoryAsync(DELIVER2);
                        currentState = State.GRAB2;
                        elbowUp = elbowUpState.START;

                    }
                    break;
                case GRAB2:
                    if (waitTimer.seconds()>=waitTime3) {
                        LiftTarget = MID;

                        currentState = State.DELIVER2;

                    }
                    break;
                case DELIVER2:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(OPEN);
                        waitTimer.reset();
                        currentState = State.DROP2;
                    }
                    break;
                case DROP2:
                    if (waitTimer.seconds() >= waitTime1) {
                        drive.followTrajectoryAsync(CONES3);
                        LiftTarget = 80;//cone 3
                        //robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        downTimer.reset();
                        elbowDown = elbowDownState.START;
                        currentState = State.CONES3;
                    }
                    break;
                case CONES3:
                    if (!drive.isBusy()) {
                        //drive.followTrajectoryAsync(SLOWCONES1);
                        if (left == gray && center == gray && right == notGray){
                            //aka left + center see tile and right sees tape
                            drive.followTrajectoryAsync(Strafe_SEESRIGHTONLY);
                            drive.setPoseEstimate(newPose3);
                            currentState = State.STRAFE_ADJ3;

                        } else if (left == gray && center == notGray && right == notGray) {
                            //aka left sees tile and center + right see tape
                            drive.followTrajectoryAsync(Strafe_SEESRIGHTANDCENTER);
                            drive.setPoseEstimate(newPose3);
                            currentState = State.STRAFE_ADJ3;

                        } else if (left == notGray && center == notGray && right == notGray) {
                            //aka left + center + right see tape
                            drive.setPoseEstimate(newPose3);
                            currentState = State.STRAFE_ADJ3;

                        } else if (left == notGray && center == notGray && right == gray) {
                            //aka left + center see tape and right sees tile
                            drive.followTrajectoryAsync(Strafe_SEESLEFTANDCENTER);
                            drive.setPoseEstimate(newPose3);
                            currentState = State.STRAFE_ADJ3;

                        } else if (left == notGray && center == gray && right == gray) {
                            //aka left sees tape and center + right see tile
                            drive.followTrajectoryAsync(Strafe_SEESLEFTONLY);
                            drive.setPoseEstimate(newPose3);
                            currentState = State.STRAFE_ADJ3;
                        } else {
                            drive.setPoseEstimate(newPose3);
                            currentState = State.STRAFE_ADJ3;
                        }
                        //currentState = State.STRAFE_ADJ;
                    }
                    break;
                case STRAFE_ADJ3:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        upTimer.reset();
                        //LiftTarget = HIGH;
                        waitTimer.reset();
                        currentState = State.LIFT3;
                    }
                    break;
                case LIFT3:
                    if (waitTimer.seconds()>=waitTime1) {

                        drive.followTrajectoryAsync(DELIVER3);
                        currentState = State.GRAB3;
                        elbowUp = elbowUpState.START;
                        waitTimer.reset();
                    }
                    break;
                case GRAB3:
                    if (waitTimer.seconds()>=waitTime3) {
                        LiftTarget = MID-10;
                        currentState = State.DELIVER3;

                    }
                    break;
                case DELIVER3:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(OPEN);
                        waitTimer.reset();
                        currentState = State.DROP3;
                        upTime1 = 1.0;
                    }
                    break;
                case DROP3:
                    if (waitTimer.seconds() >= waitTime1) {
                       // drive.followTrajectoryAsync(CONES4);
                        LiftTarget = 0;//cone 3
                        //robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        downTimer.reset();
                        elbowDown = elbowDownState.START;
                        currentState = State.DROP4;
                    }
                    break;
//                case CONES4:
//                    if (!drive.isBusy()) {
//                        //drive.followTrajectoryAsync(SLOWCONES1);
//                        if (left == gray && center == gray && right == notGray){
//                            //aka left + center see tile and right sees tape
//                            drive.followTrajectoryAsync(Strafe_SEESRIGHTONLY);
//                            drive.setPoseEstimate(newPose4);
//                            currentState = State.STRAFE_ADJ4;
//
//                        } else if (left == gray && center == notGray && right == notGray) {
//                            //aka left sees tile and center + right see tape
//                            drive.followTrajectoryAsync(Strafe_SEESRIGHTANDCENTER);
//                            drive.setPoseEstimate(newPose4);
//                            currentState = State.STRAFE_ADJ4;
//
//                        } else if (left == notGray && center == notGray && right == notGray) {
//                            //aka left + center + right see tape
//                            drive.setPoseEstimate(newPose4);
//                            currentState = State.STRAFE_ADJ4;
//
//                        } else if (left == notGray && center == notGray && right == gray) {
//                            //aka left + center see tape and right sees tile
//                            drive.followTrajectoryAsync(Strafe_SEESLEFTANDCENTER);
//                            drive.setPoseEstimate(newPose4);
//                            currentState = State.STRAFE_ADJ4;
//
//                        } else if (left == notGray && center == gray && right == gray) {
//                            //aka left sees tape and center + right see tile
//                            drive.followTrajectoryAsync(Strafe_SEESLEFTONLY);
//                            drive.setPoseEstimate(newPose4);
//                            currentState = State.STRAFE_ADJ4;
//                        } else {
//                            drive.setPoseEstimate(newPose4);
//                            currentState = State.STRAFE_ADJ4;
//                        }
//                        //currentState = State.STRAFE_ADJ;
//                    }
//                    break;
//                case STRAFE_ADJ4:
//                    if (!drive.isBusy()) {
//                        robot.grab.setPosition(CLOSE);
//                        robot.twist.setPosition(startPos);
//                        upTimer.reset();
//                        //LiftTarget = HIGH;
//                        waitTimer.reset();
//                        currentState = State.LIFT4;
//                    }
//                    break;
//                case LIFT4:
//                    if (waitTimer.seconds()>=waitTime1) {
//                        drive.followTrajectoryAsync(DELIVER4);
//                        currentState = State.GRAB4;
//                        elbowUp = elbowUpState.START;
//                        waitTimer.reset();
//
//                    }
//                    break;
//                case GRAB4:
//                    if (waitTimer.seconds()>=waitTime2) {
//
//                        LiftTarget = MID;
//
//                        currentState = State.DELIVER4;
//
//                    }
//                    break;
//                case DELIVER4:
//                    if (!drive.isBusy()) {
//                        robot.grab.setPosition(OPEN);
//                        waitTimer.reset();
//                        currentState = State.DROP4;
//                        LiftTarget = 0;
//                    }
//                    break;
                case DROP4:
                    if (waitTimer.seconds()>=waitTime1) {
                        if (!drive.isBusy()) {
                            //LiftTarget = 0;
                            currentState = State.PARK;
                            if(tagOfInterest == null)
                            {
                                /*
                                 * Insert your autonomous code here, presumably running some default configuration
                                 * since the tag was never sighted during INIT
                                 */
                                robot.twist.setPosition(startPos);
                                drive.followTrajectorySequenceAsync(PARK_B);
                            }
                            else //TODO:auto code
                            {
                                /*
                                 * Insert your autonomous code here, probably using the tag pose to decide your configuration.
                                 */
                                //drive.followTrajectoryAsync(PARK_B);

                                if(tagOfInterest.id == 0)
                                {
                                    // do something
                                    robot.twist.setPosition(startPos);
                                    drive.followTrajectorySequenceAsync(PARK_A);

                                }
                                else if(tagOfInterest.id == 1)
                                {
                                    // do something else
                                    robot.twist.setPosition(startPos);
                                    drive.followTrajectorySequenceAsync(PARK_B);
                                }
                                else if(tagOfInterest.id == 2)
                                {
                                    // do something else
                                    robot.twist.setPosition(startPos);
                                    drive.followTrajectorySequenceAsync(PARK_C);
                                }
                                robot.lelbow.setPosition(0);
                                robot.relbow.setPosition(0);

                            }
                            // Start the wait timer once we switch to the next state
                            // This is so we can track how long we've been in the WAIT_1 state
                        }
                    }
                    break;

                case PARK:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }
            /* TO USE IN PROGRAM
             * set state to START in other switch case
             * AT THE SAME TIME
             *  set claw to close
             *  set twist to startPos
             *  reset up timer
             * */
            switch (elbowUp) {
                case START:
                    if (upTimer.seconds() >= upTime1) {
                        robot.relbow.setPosition(.55);
                        robot.lelbow.setPosition(.55);
                        upTimer.reset();
                        elbowUp = elbowUpState.GRAB;
                    }
                    break;
                case GRAB:
                    if(upTimer.seconds() >= upTime) {
                        robot.twist.setPosition(flipPos);
                        elbowUp = elbowUpState.LIFT_TWIST;
                    }
                    break;
                case LIFT_TWIST:
                    if (Math.abs(robot.twist.getPosition() - flipPos) <.005 ) {
                        elbowUp = elbowUpState.IDLE;
                    }
                    break;
                case IDLE:

                    break;
            }

            /* TO USE IN PROGRAM
             * set state to START in other switch case
             * AT THE SAME TIME
             *  set claw to close
             *  set twist to startPos
             *  reset down timer
             * */
            switch (elbowDown) {
                case START:
                    if(downTimer.seconds() >= downTime1) {
                        robot.relbow.setPosition(0.03);
                        robot.lelbow.setPosition(0.03);
                        downTimer.reset();
                        elbowDown = elbowDownState.CLOSE;                    }
                    break;
                case CLOSE:
                    if (downTimer.seconds() >= downTime) {
                        robot.grab.setPosition(OPEN);
                        elbowDown = elbowDownState.LIFT_TWIST;
                    }
                    break;
                case LIFT_TWIST:
                    if (Math.abs(robot.grab.getPosition() - OPEN) <.005) {
                        elbowDown = elbowDownState.IDLE;
                    }
                    break;
                case IDLE:

                    break;
            }


            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();

            colorS.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class colorS {
        public colorS(HardwareMap hardwareMap){

            L_color = hardwareMap.get(ColorSensor.class, "Left");
            C_color = hardwareMap.get(ColorSensor.class, "Center");
            R_color = hardwareMap.get(ColorSensor.class,"Right");
        }
        public void update() {

            int red_blue_L = (L_color.red())/(L_color.blue());
            int red_blue_C = (C_color.red())/(C_color.blue());
            int red_blue_R = (R_color.red())/(R_color.blue());

            int blue_red_L = (L_color.blue())/(L_color.red());
            int blue_red_C = (C_color.blue())/(C_color.red());
            int blue_red_R = (R_color.blue())/(R_color.red());


            if(red_blue_L<=1 && blue_red_L<=1) {
                telemetry.addData("color from left", "gray");
                left = gray;
            } else if (blue_red_L>=2){
                telemetry.addData("color from left", "blue");
                left = notGray;
            } else if (red_blue_L>=2 && blue_red_L<=1) {
                telemetry.addData("color from left", "red");
                left = notGray;
            }

            if(red_blue_C<=1 && blue_red_C<=1) {
                telemetry.addData("color from center", "gray");
                center = gray;
            } else if (blue_red_C>=2){
                telemetry.addData("color from center", "blue");
                center = notGray;
            } else if (red_blue_C>=2 && blue_red_C<=1) {
                telemetry.addData("color from center", "red");
                center = notGray;
            }

            if(red_blue_R<=1 && blue_red_R<=1) {
                telemetry.addData("color from right", "gray");
                right = gray;
            } else if (blue_red_R>=2){
                telemetry.addData("color from right", "blue");
                right = notGray;
            } else if (red_blue_R>=2 && blue_red_R<=1) {
                telemetry.addData("color from right", "red");
                right = notGray;
            }
        }
    }

    class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware

            controller = new PIDController(p, i, d);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            larm = hardwareMap.get(DcMotorEx.class,"Llift");
            rarm = hardwareMap.get(DcMotorEx.class,"Rlift");


            larm.setDirection(DcMotorEx.Direction.REVERSE);
            rarm.setDirection(DcMotorEx.Direction.FORWARD);

            larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift

            controller.setPID(p, i, d);

            int larmPos = larm.getCurrentPosition();
            int rarmPos = rarm.getCurrentPosition();

            double Lpid = controller.calculate(larmPos, LiftTarget);
            double Rpid = controller.calculate(rarmPos, LiftTarget);

            double Lpower = Lpid + f;
            double Rpower = Rpid + f;

            larm.setPower(Lpower);
            rarm.setPower(Rpower);

            telemetry.addData("Lpos", larmPos);
            telemetry.addData("Rpos", rarmPos);
            telemetry.addData("Ltarget", LiftTarget);
            telemetry.addData("Rtarget", LiftTarget);
            telemetry.update();
        }
    }
}
// hi there :]