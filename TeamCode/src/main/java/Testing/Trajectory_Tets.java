package Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.opencv.core.Mat;

/*
 * This is an example of a more complex path to really test the tuning.
 */
//@Disabled
@Autonomous(group = "drive")
public class Trajectory_Tets extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);
//
//        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);

        /*
        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(36.24, -64.28, Math.toRadians(90.00)))
        .lineToLinearHeading(new Pose2d(34.92, -33.79, Math.toRadians(135.00)))
        .lineToLinearHeading(new Pose2d(36.61, -12.71, Math.toRadians(180.00)))
        .build();
drive.setPoseEstimate(untitled0.start());*/


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

        drive.followTrajectory(START);

        drive.followTrajectory(LINE1);

        drive.followTrajectory(CONES1);

        sleep(2000);


    }
}
