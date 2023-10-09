package Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import CompCode.FLIPmap;

/*
 * This is an example of a more complex path to really test the tuning.
 */
//@Disabled
@Autonomous(group = "drive")
public class Color_sensor_testsLEFT extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FLIPmap robot = new FLIPmap();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);
        final float[] hsvValuesL = new float[3];
        final float[] hsvValuesC = new float[3];
        final float[] hsvValuesR = new float[3];
        double red_blue_L;
        double red_blue_C;
        double red_blue_R;

        double blue_red_L;
        double blue_red_C;
        double blue_red_R;

        double LColor = 0;
        double CColor = 0;
        double RColor = 0;

        double Gray = 1;
        double NotGray = 0;

        waitForStart();



        /*
        go towards color line
        get reading from color sensors
            if left.blue > right.blue
            go left until all three are roughly equal?
            relocalize!
            follow rest of path
         */

        if (isStopRequested()) return;
        Pose2d startPose = new Pose2d(35, -67, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory START = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(35,-28.5, Math.toRadians(348.00)))
                .build();

        Trajectory LINE1 = drive.trajectoryBuilder(START.end())
                .lineToLinearHeading(new Pose2d(36.61, -12.71, Math.toRadians(0.00)))
                .build();

        Trajectory CONES1 = drive.trajectoryBuilder(LINE1.end())
                .lineToLinearHeading(new Pose2d(56.34, -15.50, Math.toRadians(0.00)))
                .build();


        // if left is gray move right
        // if right is gray move left
//
//        if (LColor == Gray){
//            robot.
//        }


            while (opModeIsActive()) {

                red_blue_L = (robot.L_color.red())/(robot.L_color.blue());
                red_blue_C = (robot.C_color.red())/(robot.C_color.blue());
                red_blue_R = (robot.R_color.red())/(robot.R_color.blue());

                blue_red_L = (robot.L_color.blue())/(robot.L_color.red());
                blue_red_C = (robot.C_color.blue())/(robot.C_color.red());
                blue_red_R = (robot.R_color.blue())/(robot.R_color.red());

                if(red_blue_L<=1 && blue_red_L<=1) {
                    telemetry.addData("color from left", "gray");
                } else if (blue_red_L>2){
                    telemetry.addData("color from left", "blue");

                } else if (red_blue_L>=2 && blue_red_L<=1) {
                    telemetry.addData("color from left", "red");
                } else {
                    LColor = NotGray;
                }

                if(red_blue_C<=1 && blue_red_C<=1) {
                    telemetry.addData("color from center", "gray");
                } else if (blue_red_C>2){
                    telemetry.addData("color from center", "blue");
                } else if (red_blue_C>=2 && blue_red_C<=1) {
                    telemetry.addData("color from center", "red");
                }

                if(red_blue_R<=1 && blue_red_R<=1) {
                    telemetry.addData("color from right", "gray");
                } else if (blue_red_R>2){
                    telemetry.addData("color from right", "blue");
                } else if (red_blue_R>=2 && blue_red_R<=1) {
                    telemetry.addData("color from right", "red");
                }



                telemetry.addData("Left (red/blue)", (red_blue_L));
                telemetry.addData("center (red/blue)", (red_blue_C));
                telemetry.addData("right (red/blue)", (red_blue_R));

                telemetry.addData("Left (blue/red)", (blue_red_L));
                telemetry.addData("center (blue/red)", (blue_red_C));
                telemetry.addData("right (blue/red)", (blue_red_R));


                telemetry.update();
            }



        drive.followTrajectory(START);

        drive.followTrajectory(LINE1);

        drive.followTrajectory(CONES1);


//        sleep(2000);


    }
}
