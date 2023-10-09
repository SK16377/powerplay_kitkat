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
@Autonomous(name = "LEFT_AUTO", group = "COMP")
@Disabled
public class Left_Auto_FSM extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;


    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    double OPEN = .7;
    double CLOSE = .85;
    double RFBINIT = 0.06;
    double LFBINIT = 0.94;
    double RFBUP = 0.08;
    double LFBUP =0.92;

    int ID_TAG_OF_INTEREST1 = 0; // Tag ID 18 from the 36h11 family
    int ID_TAG_OF_INTEREST2 = 1;
    int ID_TAG_OF_INTEREST3 = 2;


    public PIDController controller;

    //TODO: edit variables on dashboard lol

    // p = increase if not reaching target position

    public static double p = .007, i = 0, d = 0; // d = dampener (dampens arm movement and is scary). ignore i
    public static double f = .06;  // prevents arm from falling from gravity

    public static int LiftTarget = 0; // target position

    public static int START_POS = 230;
    public static int LOW = 1208; //1208 = LOW
    public static int MID = 2078; //2078 = MID
    public static int HIGH = 2900; //2900 = HIGH
    public static int CONE5START = 655;
    public static int CONE5 = 475;


    public static double INTPOWER = 0;

    public static int INT = 1;
    public static double INTLOW = .2;
    public static int INTOFF = 0;
    public static int  INTOUT= -1;

    //public static int Rtarget = 0;
    private final double ticks_in_degree = 700/180.0;

    //VoltageSensor voltageSensor;

    private DcMotorEx larm;
    private DcMotorEx rarm;

    private DcMotorEx rin;
    private DcMotorEx lin;


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    AprilTagDetection tagOfInterest = null;

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        START,
        TURN_1,
        HIGH1,
        WAIT_1,         // Then we're gonna wait a second
        BACKHIGH1,
        TURN_2,
        CONES,
        SLOWCONES,
        WAIT_2,
        WAIT_3,
        WAIT_4,
        BACKCONES,
        HIGH_HEADING,
        HIGH2,
        EXTRA_WAIT,
        WAIT_5,
        BACKHIGH2,
        TURN_3,
        PARK,



        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(15, 10, Math.toRadians(180));

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
        Intake intake = new Intake(hardwareMap);



        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory START = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-36,-12)) //set up
                .build();

        //turn
        // Define the angle to turn at
        double turnAngle1 = Math.toRadians(152); //145

        Trajectory HIGH1 = drive.trajectoryBuilder(START.end().plus(new Pose2d(0,0, turnAngle1)))
                .lineTo(new Vector2d(-24,-5))
                .build();
        //wait (arm stuff happens) place preload

        Trajectory BACKHIGH1 = drive.trajectoryBuilder(HIGH1.end())
                //.lineToLinearHeading(new Pose2d(36,12,Math.toRadians(0))) //pose to begin cycles
                .lineTo(new Vector2d(-36,-12))
                .build();

        //turn
        double turnAngle2 = Math.toRadians(119); //135

        Trajectory CONES = drive.trajectoryBuilder(BACKHIGH1.end().plus(new Pose2d(0,0, turnAngle2)))
                .lineTo(new Vector2d(-56,-11))
                .build();

        Trajectory SLOWCONES = drive.trajectoryBuilder(CONES.end())
                .lineTo(new Vector2d(-68,-10.5))
                .build();
        //wait (arm stuff) grab cone1

        Trajectory BACKCONES = drive.trajectoryBuilder(CONES.end())
                .lineTo(new Vector2d(-42,-12.5))
                .build();

        Trajectory HIGH_HEADING = drive.trajectoryBuilder(BACKCONES.end())
                .lineToLinearHeading(new Pose2d(-38.5,-12.7,Math.toRadians(58)))
                .build();

        Trajectory HIGH2 = drive.trajectoryBuilder(HIGH_HEADING.end())
                .lineTo(new Vector2d(-27,-4))
                .build();
        //wait (arm stuff happens) place preload

        Trajectory BACKHIGH2 = drive.trajectoryBuilder(HIGH2.end())
                //.lineToLinearHeading(new Pose2d(36,12,Math.toRadians(0))) //pose to begin cycles
                .lineTo(new Vector2d(-36,-12))
                .build();

        //high1
        //backhigh1

        //turn
        double turnAngle3 = Math.toRadians(122);

        //go forward to the stack


        Trajectory PARK_A = drive.trajectoryBuilder(BACKHIGH2.end().plus(new Pose2d(0,0,turnAngle3)))
                .forward(22)
                .build();

        Trajectory PARK_B = drive.trajectoryBuilder(BACKHIGH2.end().plus(new Pose2d(0,0,turnAngle3)))
                .back(1)
                .build();

        Trajectory PARK_C = drive.trajectoryBuilder(BACKHIGH2.end().plus(new Pose2d(0,0,turnAngle3)))
                .back(21.5)
                .build();

        // Third trajectory
        // We have to define a new end pose because we can't just call trajectory2.end()
        // Since there was a point turn before that
        // So we just take the pose from trajectory2.end(), add the previous turn angle to it


        // Define a 1.5 second wait time
        double waitTime1 = .5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        double waitTime2 = .3;  // INTAKE
        ElapsedTime waitTimer2 = new ElapsedTime();

        double waitTime3 = .5;
        ElapsedTime waitTimer3 = new ElapsedTime(); //STOP

        double waitTime4 = .4;
        ElapsedTime waitTimer4 = new ElapsedTime(); // LIFT

        double waitTime5 = .8;
        ElapsedTime waitTimer5 = new ElapsedTime();

        double waitTime6 = 1;
        ElapsedTime waitTimer6 = new ElapsedTime();

        telemetry.setMsTransmissionInterval(50);

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
                        /** tag id 0 is detected**/
                        telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
                        tagOfInterest = tag;
                        tagFound = 2;
                        break;
                    }
                    else if(tag.id == ID_TAG_OF_INTEREST3)
                    {
                        /** tag id 0 is detected**/
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
        }

        // start is running



        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.START;
        drive.followTrajectoryAsync(START);
        LiftTarget = LOW;


        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case START:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.TURN_1;
                        drive.turnAsync(turnAngle1);
                        LiftTarget = HIGH;
                    }
                    break;
                case TURN_1:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.HIGH1;
                        drive.followTrajectoryAsync(HIGH1);
                    }
                    break;
                case HIGH1:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;
                        INTPOWER = INTOUT;
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.BACKHIGH1;
                        INTPOWER = INTOFF;
                        drive.followTrajectoryAsync(BACKHIGH1);

                    }
                    break;

                case BACKHIGH1:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.TURN_2;
                        drive.turnAsync(turnAngle2);
                        LiftTarget = CONE5START; // cone 5 start hehe

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                    }
                    break;
                case TURN_2:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.CONES;
                        drive.followTrajectoryAsync(CONES);
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                    }
                    break;

                case CONES:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.SLOWCONES;
                        drive.followTrajectoryAsync(SLOWCONES);
                        waitTimer2.reset();
                        //INTPOWER = INT;
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                    }
                    break;

                case SLOWCONES:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_2;
                        //LiftTarget = CONE5;
                        INTPOWER = INT;
                        waitTimer2.reset();

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                    }
                    break;

                case WAIT_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (waitTimer2.seconds() >= waitTime2) {
                        currentState = State.WAIT_3;
                        //INTPOWER = INTOFF;
                        LiftTarget = CONE5;

                        waitTimer3.reset();
                    }
                    break;

                case WAIT_3:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (waitTimer3.seconds() >= waitTime3) {
                        currentState = State.WAIT_4;
                        //INTPOWER = INTOFF;
                        LiftTarget = LOW;
                        waitTimer4.reset();

                    }
                    break;

                case WAIT_4:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (waitTimer4.seconds() >= waitTime4) {
                        INTPOWER = .1;

                        currentState = State.BACKCONES;
                        drive.followTrajectoryAsync(BACKCONES);
                    }
                    break;

                case BACKCONES:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.HIGH_HEADING;
                        drive.followTrajectoryAsync(HIGH_HEADING);
                        LiftTarget = HIGH;
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                    }
                    break;
                case HIGH_HEADING:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.EXTRA_WAIT;
                        drive.followTrajectoryAsync(HIGH2);
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer6.reset();
                    }
                    break;
                case EXTRA_WAIT:
                    if (waitTimer6.seconds()>=waitTime6) {
                        currentState = State.HIGH2;

                    }
                    break;
                case HIGH2:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_5;
                        INTPOWER = INTOUT;
                        waitTimer5.reset();
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                    }
                    break;
                case WAIT_5:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (waitTimer5.seconds() >= waitTime5) {
                        currentState = State.BACKHIGH2;
                        INTPOWER = INTOFF;
                        drive.followTrajectoryAsync(BACKHIGH2);
                    }
                    break;
                case BACKHIGH2:
                    // Check if the drive class is busy following the trajectory7
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.TURN_3;
                        drive.turnAsync(turnAngle3);
                        LiftTarget = START_POS;
                        INTPOWER = INTOFF;
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                    }
                    break;
                case TURN_3:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
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
                            // e.g.
                            if(tagOfInterest.id == 0)
                            {
                                // do something
                                drive.followTrajectoryAsync(PARK_A);

                            }
                            else if(tagOfInterest.id == 1)
                            {
                                // do something else
                                drive.followTrajectoryAsync(PARK_B);
                            }
                            else if(tagOfInterest.id == 2)
                            {
                                // do something else
                                drive.followTrajectoryAsync(PARK_C);
                            }

                            LiftTarget = 0;
                        }
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
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

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();

            intake.update();

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

    class Intake {
        public Intake(HardwareMap hardwareMap){

            rin = hardwareMap.get(DcMotorEx.class,"rin");
            lin = hardwareMap.get(DcMotorEx.class,"lin");

            lin.setDirection(DcMotor.Direction.REVERSE);
            rin.setDirection(DcMotor.Direction.FORWARD);
        }

        public void update() {
            lin.setPower(INTPOWER);
            rin.setPower(INTPOWER);
        }
    }


    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware

            controller = new PIDController(p, i, d);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            larm = hardwareMap.get(DcMotorEx.class,"la");
            rarm = hardwareMap.get(DcMotorEx.class,"ra");


            larm.setDirection(DcMotorEx.Direction.FORWARD);
            rarm.setDirection(DcMotorEx.Direction.REVERSE);

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

            double Lff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; //* (12/voltageSensor.getVoltage()
            double Rff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; // * (12/voltageSensor.getVoltage()

            double Lpower = Lpid + Lff;
            double Rpower = Rpid + Rff;

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