package Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class RRAutoTestRIGHT extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-33, 62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-8,60))
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                .lineTo(new Vector2d(-7.5, 23))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0,0, Math.toRadians(90))))
                .lineTo(new Vector2d(-9,5))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-49, 8))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-60, 8))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(-34,11,Math.toRadians(135)))
                .build();
        Trajectory RIGHTZONE1 = drive.trajectoryBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(-1,11, Math.toRadians(90)))
                .build();
        Trajectory RIGHTZONE2 = drive.trajectoryBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(-35,14,Math.toRadians(90)))
                .build();
        Trajectory RIGHTZONE3 = drive.trajectoryBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(-60,11,Math.toRadians(90)))
                .build();



        drive.followTrajectory(traj);
        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
       // drive.followTrajectory(REDLEFTZONE3);
        //drive.followTrajectory(REDLEFTZONE2);
         drive.followTrajectory(RIGHTZONE1);

    }
}
