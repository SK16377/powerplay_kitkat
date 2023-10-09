package Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import AprilTagsss.AprilTagDetectionPipeline;
import CompCode.FLIPmap;

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
@Autonomous(name = "LEFT_AUTO_NEW", group = "COMP")
@Disabled

public class Left_AutoSAVED extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;


    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;


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
    public static int MID = 180; //2078 = MID
    public static int HIGH = 500; //2900 = HIGH


    double OPEN = .01;
    double CLOSE = 0.1375;

    double startPos = 0.69;
    double flipPos = 0.01;





    //public static int Rtarget = 0;
   // private final double ticks_in_degree = 700/180.0;

    //VoltageSensor voltageSensor;

    private DcMotorEx larm;
    private DcMotorEx rarm;

    public ColorSensor L_color;
    public ColorSensor C_color;
    public ColorSensor R_color;


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
        GRAB1,
        DELIVER1,
        DROP1,
        CONES2,
        SLOWCONES2,
        GRAB2,
        DELIVER2,
        DROP2,
        CONES3,
        SLOWCONES3,
        GRAB3,
        DELIVER3,
        DROP3,
        CONES4,
        SLOWCONES4,
        GRAB4,
        DELIVER4,
        DROP4,
        PARK,
        IDLE            // Our bot will enter the IDLE state when done
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



        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FLIPmap robot = new FLIPmap();

        // Set inital pose
        Pose2d startPose = new Pose2d(-35, -67, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory START = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35.00,-28.75, Math.toRadians(198.00)))
                .build();

        Trajectory LINE1 = drive.trajectoryBuilder(START.end()) //plus turn .plus(new Pose2d(0,0, Math.toRadians(turnAngle1)
                .lineToLinearHeading(new Pose2d(-36.61, -13.25, Math.toRadians(180.00)))
                .build();

        Trajectory CONES1 = drive.trajectoryBuilder(LINE1.end())
                .lineToLinearHeading(new Pose2d(-56.00, -17.00, Math.toRadians(180.00)))
                .build();

        Trajectory SLOWCONES1 = drive.trajectoryBuilder(CONES1.end())
                .lineToLinearHeading(new Pose2d(-62.50,-17.00,Math.toRadians(180.0)))
                .build();

        Trajectory DELIVER1 = drive.trajectoryBuilder(SLOWCONES1.end(),true)
                .splineTo(new Vector2d(-35.75, -21.00), Math.toRadians(-32.48))
                .build();

        Trajectory CONES2 = drive.trajectoryBuilder(DELIVER1.end())
                .splineTo(new Vector2d(-56.50, -16.50), Math.toRadians(180.00))
                .build();

        Trajectory SLOWCONES2 = drive.trajectoryBuilder(CONES2.end())
                .lineToLinearHeading(new Pose2d(-64.50,-16.50,Math.toRadians(180.0)))
                .build();

        Trajectory DELIVER2 = drive.trajectoryBuilder(SLOWCONES2.end(),true)
                .splineTo(new Vector2d(-36.00, -22.00), Math.toRadians(-34.48))
                .build();

        Trajectory CONES3 = drive.trajectoryBuilder(DELIVER2.end())
                .splineTo(new Vector2d(-58.00, -16.00), Math.toRadians(180.00))
                .build();

        Trajectory SLOWCONES3 = drive.trajectoryBuilder(CONES3.end())
                .lineToLinearHeading(new Pose2d(-65.50,-16.00,Math.toRadians(180.0)))
                .build();

        Trajectory DELIVER3 = drive.trajectoryBuilder(SLOWCONES3.end(),true)
                .splineTo(new Vector2d(-35.53, -22.00), Math.toRadians(-34.48))
                .build();

        Trajectory CONES4 = drive.trajectoryBuilder(DELIVER3.end())
                .splineTo(new Vector2d(-58.54, -15.25), Math.toRadians(180.00))
                .build();

        Trajectory SLOWCONES4 = drive.trajectoryBuilder(CONES3.end())
                .lineToLinearHeading(new Pose2d(-67.3,-15.25,Math.toRadians(180.0)))
                .build();

        Trajectory DELIVER4 = drive.trajectoryBuilder(SLOWCONES4.end(),true)
                .splineTo(new Vector2d(-35.23, -22.00), Math.toRadians(-34.48))
                .build();


        Trajectory PARK_A = drive.trajectoryBuilder(DELIVER4.end())
                .splineTo(new Vector2d(-60.50, -14), Math.toRadians(180.00))
                .build();


        Trajectory PARK_B = drive.trajectoryBuilder(DELIVER4.end())
                .forward(5.5)
                .build();

        Trajectory PARK_C1 = drive.trajectoryBuilder(DELIVER4.end())
                .splineToConstantHeading(new Vector2d(-26.07, -11.01), Math.toRadians(25.74))
                .build();

        Trajectory PARK_C2 = drive.trajectoryBuilder(PARK_C1.end())
                .splineToLinearHeading(new Pose2d(-14.63, -13.00, Math.toRadians(0.00)), Math.toRadians(13.55))
                .build();



        // Define a 1.5 second wait time
        double upTime = .5;
        double upTime1 = 1.0;
        ElapsedTime upTimer = new ElapsedTime();

        double downTime = .5;
        double downTime1 = 1.0;
        ElapsedTime downTimer = new ElapsedTime();

        double waitTime1 = .5;
        ElapsedTime waitTimer = new ElapsedTime();

        double waitTime2 = .8;

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
            robot.relbow.setPosition(0.03);
            robot.lelbow.setPosition(0.03);
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
//                case START:
//                    //once the first trajectory is finished,
//                    // turn towards the junction
//                    if (!drive.isBusy()) {
//                        currentState = State.TURN_1;
//                        drive.turnAsync(turnAngle1);
//                    }
//                    break;
                case TURN_1:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(OPEN);
                        currentState = State.WAIT_1;
                    }
                    break;
                case WAIT_1:
                    if (elbowUp == elbowUpState.IDLE) {
                        currentState = State.FLIP1;
                        upTime1 = 0.75;
                        waitTimer.reset();
                    }
                    break;
                case FLIP1:
                    if (waitTimer.seconds() >= waitTime1) {
                        drive.followTrajectoryAsync(LINE1);
                        LiftTarget = 180;//cone 5
                        //robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        downTimer.reset();
                        elbowDown = elbowDownState.START;
                        currentState = State.SPLINE1;
                    }
                    break;
                case SPLINE1:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(CONES1);
                        currentState = State.CONES1;
                    }
                    break;
                case CONES1:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(SLOWCONES1);
                        currentState = State.SLOWCONES1;
                    }
                    break;
                case SLOWCONES1:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        upTimer.reset();
                        LiftTarget = MID;
                        waitTimer.reset();
                        elbowUp = elbowUpState.START;
                        currentState = State.GRAB1;
                    }
                    break;
                case GRAB1:
                    if (waitTimer.seconds()>=waitTime1) {
                        drive.followTrajectoryAsync(DELIVER1);
                        currentState = State.DELIVER1;

                    }
                    break;
                case DELIVER1:
                    if (!drive.isBusy()) {
                       robot.grab.setPosition(OPEN);
                       waitTimer.reset();
                       currentState = State.DROP1;
                       // upTime1 = 0.7;
                    }
                    break;
                case DROP1:
                    if (waitTimer.seconds()>=waitTime1) {
                        drive.followTrajectoryAsync(CONES2);
                        LiftTarget = 135;//cone 4
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
                        drive.followTrajectoryAsync(SLOWCONES2);
                        currentState = State.SLOWCONES2;
                    }
                    break;
                case SLOWCONES2:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        upTimer.reset();
                        waitTimer.reset();
                        elbowUp = elbowUpState.START;
                        currentState = State.GRAB2;
                    }
                    break;
                case GRAB2:
                    if (waitTimer.seconds()>=waitTime1) {
                        LiftTarget = MID;
                        drive.followTrajectoryAsync(DELIVER2);
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
                        LiftTarget = 90;//cone 3
                        //robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        downTimer.reset();
                        elbowDown = elbowDownState.START;
                        currentState = State.CONES3;
                    }
                    break;
                case CONES3:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(SLOWCONES3);
                        currentState = State.SLOWCONES3;
                    }
                    break;
                case SLOWCONES3:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        upTimer.reset();
                        waitTimer.reset();
                        elbowUp = elbowUpState.START;
                        currentState = State.GRAB3;
                    }
                    break;
                case GRAB3:
                    if (waitTimer.seconds()>=waitTime1) {
                        LiftTarget = MID;
                        drive.followTrajectoryAsync(DELIVER3);
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
                        drive.followTrajectoryAsync(CONES4);
                        LiftTarget = 45;//cone 3
                        //robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        downTimer.reset();
                        elbowDown = elbowDownState.START;
                        currentState = State.CONES4;
                    }
                    break;
                case CONES4:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(SLOWCONES4);
                        currentState = State.SLOWCONES4;
                    }
                    break;
                case SLOWCONES4:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(CLOSE+.001);
                        robot.twist.setPosition(startPos);
                        upTimer.reset();
                        waitTimer.reset();
                        elbowUp = elbowUpState.START;
                        currentState = State.GRAB4;
                    }
                    break;
                case GRAB4:
                    if (waitTimer.seconds()>=waitTime1 ) {
                        LiftTarget = MID;
                        drive.followTrajectoryAsync(DELIVER4);
                        currentState = State.DELIVER4;

                    }
                    break;
                case DELIVER4:
                    if (!drive.isBusy()) {
                        robot.grab.setPosition(OPEN);
                        waitTimer.reset();
                        currentState = State.DROP4;
                    }
                    break;
                case DROP4:
                    if (waitTimer.seconds()>=waitTime1) {
                        if (!drive.isBusy()) {
                            currentState = State.PARK;
                            if(tagOfInterest == null)
                            {
                                /*
                                 * Insert your autonomous code here, presumably running some default configuration
                                 * since the tag was never sighted during INIT
                                 */
                                 drive.followTrajectoryAsync(PARK_B);
                            }
                            else //TODO:auto code
                            {
                                /*
                                 * Insert your autonomous code here, probably using the tag pose to decide your configuration.
                                 */
                                //drive.followTrajectoryAsync(PARK_B);
                                LiftTarget = 0;
                                if(tagOfInterest.id == 0)
                                {
                                    // do something
                                    robot.twist.setPosition(startPos);
                                    drive.followTrajectoryAsync(PARK_A);

                                }
                                else if(tagOfInterest.id == 1)
                                {
                                    // do something else
                                    robot.twist.setPosition(startPos);
                                    drive.followTrajectoryAsync(PARK_B);
                                    //drive.turnAsync(Math.toRadians(20));

                                }
                                else if(tagOfInterest.id == 2)
                                {
                                    // do something else
                                     robot.twist.setPosition(startPos);
                                     drive.followTrajectoryAsync(PARK_C1);
                                     drive.followTrajectoryAsync(PARK_C2);
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
    class Color {
        public Color(HardwareMap hardwareMap){

            L_color = hardwareMap.get(ColorSensor.class, "Left");
            C_color = hardwareMap.get(ColorSensor.class, "Center");
            R_color = hardwareMap.get(ColorSensor.class,"Right");
        }
        public void update () {

            int red_blue_L = (L_color.red()) / (L_color.blue());
            int red_blue_C = (C_color.red())/(C_color.blue());
            int red_blue_R = (R_color.red())/(R_color.blue());

            int blue_red_L = (L_color.blue())/(L_color.red());
            int blue_red_C = (C_color.blue())/(C_color.red());
            int blue_red_R = (R_color.blue())/(R_color.red());
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