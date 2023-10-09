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
public class rrTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-33, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .strafeLeft(21)
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                .back(39)

                .build();


        drive.followTrajectory(traj);
        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(90));

    }
}
