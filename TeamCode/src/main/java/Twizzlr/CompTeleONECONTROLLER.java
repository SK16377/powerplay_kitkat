/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package Twizzlr;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="singlePlayr", group="twizzy")

public class CompTeleONECONTROLLER extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    BugMap robot = new BugMap();

    double SpeedAdjust = 1;


    @Override
    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightClaw.setPosition(robot.R_OPEN);
        robot.leftClaw.setPosition(robot.L_OPEN);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Drive equation
            drive();

            //Speed adjust variable for slow mode
            if (gamepad1.a) {
                robot.leftClaw.setPosition(robot.L_OPEN);
                robot.rightClaw.setPosition(robot.R_OPEN);
            } else if (gamepad1.b) {
                robot.leftClaw.setPosition(robot.L_CLOSE);
                robot.rightClaw.setPosition(robot.R_CLOSE);
            }


            if (gamepad1.x) {
                robot.duck.setPosition(1);
            } else if (gamepad1.y) {
                robot.duck.setPosition(0);
            } else {
                robot.duck.setPosition(.5);
            }


            if (gamepad1.dpad_up) {
                arm_outTOP();
            } else if (gamepad1.dpad_down) {
                arm_in();
            } else if (gamepad1.dpad_left) {
                arm_outMID();
            } else if (gamepad1.dpad_right) {
                arm_outLOW();
            }


            if (gamepad1.left_trigger > 0) {
                robot.intake.setPower(1);
            } else if (gamepad1.right_trigger > 0) {
                robot.intake.setPower(-1);
            } else {
                robot.intake.setPower(0);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
        }
    }

    public void wrist( double counts) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = (int) (counts);
            robot.wrist.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.wrist.setPower(Math.abs(1));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive()
                    //&& (runtime.seconds() < timeoutS)
                    &&
                    (robot.wrist.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d");
                telemetry.addData("Path2", "Running at %7d :%7d");
                telemetry.update();
                drive();
            }

            // Stop all motion;
            robot.wrist.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }

    public void arm( double counts) {
        int newTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newTarget = (int) counts;
            robot.larm.setTargetPosition(newTarget);
            robot.rarm.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.larm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.larm.setPower(Math.abs(1));
            robot.rarm.setPower(Math.abs(-1));

            while (opModeIsActive()
                    //&& (runtime.seconds() < timeoutS)
                    &&
                    (robot.larm.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d");
                telemetry.addData("Path2", "Running at %7d :%7d");
                telemetry.update();
                drive();
            }
            // Stop all motion;
            robot.larm.setPower(0);
            robot.rarm.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void bof( double ARMPos, double WRISTPos) {
        int NewArmTarget;
        int NewWristTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            NewArmTarget = (int) (ARMPos);
            NewWristTarget = (int) (WRISTPos);
            robot.wrist.setTargetPosition(NewWristTarget);
            robot.larm.setTargetPosition(NewArmTarget);
            robot.rarm.setTargetPosition(NewArmTarget);
            // Determine new target position, and pass to motor controller

            // Turn On RUN_TO_POSITION
            robot.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.larm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.wrist.setPower(Math.abs(1));
            robot.larm.setPower(Math.abs(1));
            robot.rarm.setPower(Math.abs(-1));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive()
                    //&& (runtime.seconds() < timeoutS)
                    &&
                    (robot.wrist.isBusy() || robot.larm.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d");
                telemetry.addData("Path2", "Running at %7d :%7d");
                telemetry.update();
                drive();
            }

            // Stop all motion;
            robot.wrist.setPower(0);
            robot.larm.setPower(0);
            robot.rarm.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move

        }
    }
    private void drive() {
        robot.leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
        robot.rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
        robot.leftBack.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
        robot.rightBack.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);

        if (gamepad1.left_bumper) {
            SpeedAdjust = 4;
        } else if (gamepad1.right_bumper) {
            SpeedAdjust = 1;
        }
    }

    public void arm_outTOP () {
        if (robot.larm.getCurrentPosition() <= 1700) {
            robot.rightClaw.setPosition(robot.R_CLOSE);
            robot.leftClaw.setPosition(robot.L_CLOSE);
            sleep(150);
            wrist(-50);
            sleep(150);
            wrist( -320);
            bof(1150,-320);
            sleep(500);
            arm( 1150);
            // bof(2100,400);
        }
        bof(2140,450);
        // wrist(400);
        // arm(2100);
    }


    public void arm_outMID () {
        if (robot.larm.getCurrentPosition() <= 1700) {
            robot.rightClaw.setPosition(robot.R_CLOSE);
            robot.leftClaw.setPosition(robot.L_CLOSE);
            sleep(400);
            wrist( -350);
            arm( 1150);
            //  bof(2500,400);
        }
        bof(2800,600);
        // wrist(400);
        // arm(2100);
    }


    public void arm_outCAP () {
        if (robot.larm.getCurrentPosition() <= 1000) {
            robot.rightClaw.setPosition(robot.R_CLOSE);
            robot.leftClaw.setPosition(robot.L_CLOSE);
            sleep(400);
            wrist( -350);
            arm( 1150);
            //  bof(1800,250);
        }

        bof(1800,250);
        // wrist(400);
        // arm(2100);
    }

    public void arm_outLOW () {
        if (robot.larm.getCurrentPosition() <= 1700) {
            robot.rightClaw.setPosition(robot.R_CLOSE);
            robot.leftClaw.setPosition(robot.L_CLOSE);
            sleep(400);
            wrist( -350);
            arm( 1150);
        }
        bof(3200,550);
        //  wrist(500);
        //  arm(3000);
    }

    public void arm_in () {
        robot.rightClaw.setPosition(robot.R_CLOSE);
        robot.leftClaw.setPosition(robot.L_CLOSE);
        sleep(400);
       /* if (robot.larm.getCurrentPosition() > 2800) {
            arm( 2000);
        }*/
        //wrist(-100);
        bof(-20,-300);
        // wrist(-300);
        // arm(0);
        sleep(200);
        wrist(0);
        robot.rightClaw.setPosition(robot.R_OPEN);
        robot.leftClaw.setPosition(robot.L_OPEN);
    }
}