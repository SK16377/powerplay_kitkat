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

package Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import Testing.BugMap;


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
@TeleOp(name="Ta nk", group="COMP")

public class NOTcomptele extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    BugMap robot   = new BugMap();

    double SpeedAdjust = 1;
    boolean buttonIsReleased = true;
    double OPEN = .7;
    double CLOSE = .85;
    double RFBINIT = 0.06;
    double LFBINIT = 0.94;
    double RFBUP = 0.08;
    double LFBUP =0.92;
   // int endgame = 0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.leftClaw.setPosition(L_OPEN);

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

            // max counts: 4400 (outdated)
            if (gamepad1.triangle) {
//                if (testVar == 0) {
//                    testVar = 1;
//                } else if (testVar ==1 ){
//                    lift(350);
//                    testVar = 0;
//                }
            } else if (gamepad1.a){
                grab();
            }else if (gamepad1.dpad_down){
                lift(1800);
            } else if (gamepad1.dpad_left){
                lift(2900);
            }

            // positions have to be equidistant from .5
            if (gamepad2.triangle) {

            }
            else if (gamepad2.circle) {

            } else if (gamepad2.square) {

            } else if (gamepad2.cross) {

            }


            if (gamepad1.left_trigger==1) {
                robot.lin.setPower(0.5);
                robot.rin.setPower(0.5);
            } else if (gamepad1.right_trigger==1){
                robot.lin.setPower(-1);
                robot.rin.setPower(-1);
            }else  {
                robot.lin.setPower(0);
                robot.rin.setPower(0);
            }




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Endgame:", endgame);
            telemetry.update();
        }
    }


//TODO: put in an auto
    public void lift(double counts) {
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
            robot.larm.setPower(Math.abs(-1)); //left arm positive
            robot.rarm.setPower(Math.abs(1)); //right arm negative

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

        }
    }

    public void FBar (double position) {
        if (position ==0 ) {
            //dd
        } else if (position ==1 ) {

        }
    }

    public void grab (){
        robot.rin.setPower(1);
        robot.lin.setPower(1);
        lift(50);
        //sleep(300);
        robot.rin.setPower(0);
        robot.lin.setPower(0);
        lift(350);

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


}