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
public class RRAutoParkRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-33, 62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .strafeLeft(7)
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                .back(30)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-40,40,Math.toRadians(135)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-38,35,Math.toRadians(90)))
                .build();

//        Trajectory ZONE1 = drive.trajectoryBuilder(traj3.end())
//                .strafeRight(28)
//                .build();
//        Trajectory ZONE3 = drive.trajectoryBuilder(traj3.end())
//                .strafeLeft(28)
//                .build();


        drive.followTrajectory(traj);
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

       // drive.followTrajectory(ZONE1);
        //drive.followTrajectory(ZONE3);
       // drive.followTrajectory(REDLEFTZONE3);
        //drive.followTrajectory(REDLEFTZONE2);
         //drive.followTrajectory(REDLEFTZONE1);

    }
}
