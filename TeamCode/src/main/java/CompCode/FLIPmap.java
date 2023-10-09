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

package CompCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.Arrays;
import java.util.List;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
@Config
public class FLIPmap
{
    /* Public OpMode members. */
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotorEx larm = null;
    public DcMotorEx rarm = null;
//    public DcMotor rin = null;
//    public DcMotor lin = null;

    public Servo grab  = null;
    public Servo twist = null;
    public Servo lelbow  = null;
    public Servo relbow = null;


    public WebcamName phoneCam;

    public ColorSensor L_color;
    public ColorSensor C_color;
    public ColorSensor R_color;


    private VoltageSensor batteryVoltageSensor;
    private List<DcMotorEx> motors;


            /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public FLIPmap(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "lf"); //0
        rightFront = hwMap.get(DcMotor.class, "rf"); //2
        leftBack  = hwMap.get(DcMotor.class, "lb"); //1
        rightBack = hwMap.get(DcMotor.class, "rb"); //3
        larm  = hwMap.get(DcMotorEx.class, "Llift"); //0 ex
        rarm = hwMap.get(DcMotorEx.class, "Rlift");


        grab = hwMap.get(Servo.class, "grab");
        twist = hwMap.get(Servo.class, "wrist");
        lelbow = hwMap.get(Servo.class, "LE");
        relbow = hwMap.get(Servo.class, "RE");
//        grab.setPosition(0);



        motors =  Arrays.asList(larm, rarm );
        //arm2  = hwMap.get(DcMotor.class, "a2");


        L_color = hwMap.get(ColorSensor.class, "Left");
        C_color = hwMap.get(ColorSensor.class, "Center");
        R_color = hwMap.get(ColorSensor.class,"Right");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        larm.setDirection(DcMotor.Direction.REVERSE);
        rarm.setDirection(DcMotor.Direction.FORWARD);

//

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        larm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        larm.setPower(0);
        rarm.setPower(0);

//        lin.setPower(0);
//        rin.setPower(0);



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.






    }


}
