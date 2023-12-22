package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.pinkhawks.ftc.drive.SampleMecanumDrive;
import org.pinkhawks.ftc.subsystems.SubsystemManager;

@Autonomous(name="ValkyriesAutoLeft", group="Auto")
public class ValkyriesAutoLeft extends LinearOpMode{
    private final SubsystemManager subsystemManager = new SubsystemManager();

    private Trajectory traj1, traj2, traj3, traj4, traj5, traj6;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startpose = new Pose2d(-20, 60, Math.toRadians(270));
        drive.setPoseEstimate(startpose);

        traj1 = drive.trajectoryBuilder(startpose)
                .lineToLinearHeading(new Pose2d(-10, 18, Math.toRadians(0)))
                .build();

        traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(48, 20))
                .build();

        traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(48, 15, Math.toRadians(0)))
                .build();

        traj4 = drive.trajectoryBuilder(traj3.end(), true)
                .strafeTo(new Vector2d(50, 22))
                .build();

        traj5 = drive.trajectoryBuilder(traj4.end(), true)
                .back(60)
                .build();

        traj6 = drive.trajectoryBuilder(traj5.end())
                .strafeLeft(22)
                .build();


        waitForStart();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
    }
}
