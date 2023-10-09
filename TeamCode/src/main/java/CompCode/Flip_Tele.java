package CompCode;/*
 * Some declarations that are boilerplate are
 * skipped for the sake of brevity.
 * Since there are no real values to use, named constants will be used.
 */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Testing.teleptest;


@TeleOp(name="Flip Tele")
public class Flip_Tele extends LinearOpMode {


    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode

    //TELEmap robot   = new TELEmap();
    FLIPmap robot = new FLIPmap();


    // used with the dump servo, this will get covered in a bit
    ElapsedTime toggleTimer = new ElapsedTime();
    double toggleTime = .25;


    double SpeedAdjust = 1;

    double SSVar = 5;



    public static double p = .006, i = 0, d = 0.0; // d = dampener (dampens arm movement and is scary). ignore i
    public static double f = .05;  // prevents arm from falling from gravity

    public static int LiftTarget = 0; // target position

    //public static int START_POS = 230;
    public static int LOW = 0; //1208 = LOW
    public static int MID = 180; //2078 = MID
    public static int HIGH = 500; //2900 = HIGH


    double OPEN = .01;
    double CLOSE = 0.136;

    double startPos = 0.69;
    double flipPos = 0.01;


    boolean cone;


   // private final double ticks_in_degree = /360.0; //look this up later please :"" 537.6 / 360.0   700/ 180.0


    double waitTime1 = .5;
    double waitTime2 = .5;
    double waitTime3 = .5;
    ElapsedTime waitTimer1 = new ElapsedTime();
    ElapsedTime waitTimer2 = new ElapsedTime();
    ElapsedTime waitTimer3 = new ElapsedTime();


    public PIDController controller;


    private DcMotorEx larm;
    private DcMotorEx rarm;



    enum elbowUpState {
        START,
        LOWGRAB,
        GRAB,
        LOWCLOSE,
        LIFT_TWIST,
        OUTTAKE,
        LIFT, IDLE
    }
    enum elbowDownState {
        START,
        CLOSE,
        LOWCLOSE,
        LIFT_TWIST,
        OUTTAKE,
        IDLE
    }

    enum elbowFlipState {
        START,
        CLOSE,
        LOWCLOSE,
        LIFT_TWIST,
        OUTTAKE,
        IDLE
    }




    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        Lift lift = new Lift(hardwareMap);


        robot.relbow.setPosition(0.03);
        robot.lelbow.setPosition(0);
        robot.twist.setPosition(startPos);//good star .2204


        waitForStart();

        if (isStopRequested()) return;

       // LiftState liftState = LiftState.LIFT_START;
        elbowUpState elbowUp = elbowUpState.START;
        elbowDownState elbowDown = elbowDownState.START;
        elbowFlipState elbowFlip = elbowFlipState.START;
        LiftTarget = 0;

        while (opModeIsActive() && !isStopRequested()) {

            switch (elbowUp) {
                case START:
                    if (gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right) {
                        robot.grab.setPosition(CLOSE);
                        waitTimer1.reset();
                        elbowUp = elbowUpState.GRAB;
                    } else if (gamepad2.dpad_down) {
                        robot.grab.setPosition(CLOSE);
                        waitTimer1.reset();
                        elbowUp = elbowUpState.LOWGRAB;
                    }
                    break;
                case GRAB:
                    if(waitTimer1.seconds() >= waitTime1) {
                        robot.relbow.setPosition(.54);
                        robot.lelbow.setPosition(.54);
                        waitTimer1.reset();
                        elbowUp = elbowUpState.LIFT_TWIST;
                    }
                    break;
                case LOWGRAB:
                    if(waitTimer1.seconds() >= waitTime1) {
                        robot.relbow.setPosition(0.03);
                        robot.lelbow.setPosition(0.03);
                        waitTimer1.reset();
                        elbowUp = elbowUpState.LIFT;
                    }
                    break;
                case LIFT:
                    if (waitTimer1.seconds() >= waitTime2) {
                        robot.twist.setPosition(startPos);
                        elbowUp = elbowUpState.START;
                    }
                    break;
                case LIFT_TWIST:
                    if (waitTimer1.seconds() >= waitTime2) {
                        robot.twist.setPosition(flipPos);
                        elbowUp = elbowUpState.START;
                    }
                    break;
            }

            switch (elbowDown) {
                case START:
                    if (gamepad2.cross || gamepad2.triangle) {
                        robot.grab.setPosition(CLOSE);
                        robot.twist.setPosition(startPos);
                        waitTimer2.reset();
                        elbowDown = elbowDownState.CLOSE;
                    }
                    break;
                case CLOSE:
                    if(waitTimer2.seconds() >= waitTime1) {
                        robot.relbow.setPosition(0.01);
                        robot.lelbow.setPosition(0.01);
                        waitTimer2.reset();
                        elbowDown = elbowDownState.LIFT_TWIST;
                    }
                    break;
                case LIFT_TWIST:
                    if (waitTimer2.seconds() >= waitTime2) {
                        robot.grab.setPosition(OPEN);
                        elbowDown = elbowDownState.START;
                    }
                    break;
            }

            switch (elbowFlip) {
                case START:
                   if (gamepad1.square || gamepad2.square) {
                        robot.twist.setPosition(startPos);
                        waitTimer3.reset();
                        elbowFlip = elbowFlipState.CLOSE;
                    }
                    break;
                case CLOSE:
                    if(waitTimer3.seconds() >= waitTime1) {
                        robot.relbow.setPosition(0.81);
                        robot.lelbow.setPosition(0.81);
                        waitTimer3.reset();
                        elbowFlip = elbowFlipState.LIFT_TWIST;
                    }
                    break;
                case LIFT_TWIST:
                    if (waitTimer2.seconds() >= waitTime2) {
                        robot.grab.setPosition(OPEN);
                        elbowFlip = elbowFlipState.START;
                    }
                    break;
            }



//elbow flip wrist tests
           if (gamepad1.left_trigger==1) { //open
                robot.grab.setPosition(OPEN);
            } else if (gamepad1.right_trigger==1) {
                robot.grab.setPosition(CLOSE); //close
            }

            if (gamepad2.right_trigger == 1) {
                robot.twist.setPosition(startPos);
            } else if (gamepad2.left_trigger == 1) {
                robot.twist.setPosition(flipPos);//good star .2204
            }





//junction height
            if (gamepad2.dpad_up) {
                LiftTarget = HIGH;
            }
            else if (gamepad2.dpad_down) {
                LiftTarget = 475;
            }
            else if (gamepad2.dpad_right) {
                LiftTarget = MID;
            }
            else if (gamepad2.dpad_left) {
                LiftTarget = MID;
            }
            if (gamepad2.cross) {
                LiftTarget = 0;
            }


            else if (gamepad2.triangle) {
                //start stack
                if (SSVar == 5 ) {
                    LiftTarget = 180;
                }
                else if (SSVar == 4) {
                    //intake 4
                    LiftTarget = 135;
                }
                else if (SSVar == 3){
                    //intake 3
                    LiftTarget = 90;
                }
                else if (SSVar==2){
                    //intake 2
                    LiftTarget = 45;
                }
                else if (SSVar==1){
                    //intake 2
                    LiftTarget = 0;
                }
            }

            //square flip
            else if (gamepad2.circle) {
                LiftTarget = larm.getCurrentPosition() + 20;
            }

            else if (gamepad2.square) {
                LiftTarget = 0;
            }

            else if (gamepad1.square) {
                LiftTarget = 0;
            }


            if (gamepad2.right_bumper){
                //toggle R
                toggleTimer.reset();
                if (SSVar <5) {
                    SSVar++;
                }
                while (opModeIsActive()
                        && (toggleTimer.seconds() < toggleTime)
                ) {
                    // Display it for the driver.
//                    telemetry.addData("SS", SSVar);
//                    telemetry.update();
                }

            } else if (gamepad2.left_bumper){
                //toggle L
                toggleTimer.reset();
                if (SSVar >1) {
                    SSVar--;
                }
                while (opModeIsActive()
                        && (toggleTimer.seconds() < toggleTime)
                ) {
                    // Display it for the driver.
//                    telemetry.addData("SS", SSVar);
//                    telemetry.update();
                }
            }

            robot.leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
            robot.leftBack.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightBack.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);

            if (gamepad1.left_bumper) {
                SpeedAdjust = 4;
            } else if (gamepad1.right_bumper) {
                SpeedAdjust = 1;
            }



            lift.update();

            telemetry.addData("cone", cone);
            telemetry.addData("SS", SSVar);
            telemetry.update();
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

           // double Lff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; //* (12/voltageSensor.getVoltage()
           // double Rff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; // * (12/voltageSensor.getVoltage()

            double Lpower = Lpid + f;
            double Rpower = Rpid + f;

            larm.setPower(Lpower);
            rarm.setPower(Rpower);

//            telemetry.addData("Lpos", larmPos);
//            telemetry.addData("Rpos", rarmPos);
//            telemetry.addData("Ltarget", LiftTarget);
//            telemetry.addData("Rtarget", LiftTarget);
            telemetry.addData("cone", cone);
            telemetry.addData("SSVar", SSVar);
            telemetry.update();
        }
      }
    }
