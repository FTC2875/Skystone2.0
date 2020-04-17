package org.firstinspires.ftc.teamcode.test;
/**
Use this to test out trajectories easily
 **/

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.drivetrain.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();

        if (isStopRequested()) return;


        Path path = new PathBuilder(new Pose2d(0, 0, 0))
                .lineTo(new Vector2d(20, 0))
                .build();


        DriveConstraints constraints = new DriveConstraints(20, 40, 80, 1, 2, 4);
        Trajectory traj = TrajectoryGenerator.INSTANCE.generateTrajectory(path, constraints);

//        drive.turn(Math.toRadians(45));
//        Trajectory traj = drive.trajectoryBuilder(new Pose2d(0, 0))
//                .lineTo(v)
//                .lineTo(new Vector2d(0,-15))
//                .build();

        drive.followTrajectoryAsync(traj);
        while(drive.isBusy()) {drive.update();}

//        drive.turn(Math.toRadians(-45));

        sleep(100);


//        drive.followTrajectory(
//                drive.trajectoryBuilder(new Pose2d(25, 25, Math.toRadians(180)), true)
//                        .splineTo(new Pose2d(0, 0, Math.toRadians(180)))





//                        .build()
//        );
    }
}
