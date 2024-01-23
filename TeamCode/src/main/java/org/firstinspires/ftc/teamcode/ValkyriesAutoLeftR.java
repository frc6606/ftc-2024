package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.pinkhawks.ftc.drive.PoseStorage;
import org.pinkhawks.ftc.drive.SampleMecanumDrive;
import org.pinkhawks.ftc.subsystems.SubsystemManager;

@Autonomous(name="AutoLeftRED", group="Auto")
public class ValkyriesAutoLeftR extends LinearOpMode{
    private final SubsystemManager subsystemManager = new SubsystemManager();

    private Trajectory traj0,traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intakeMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startpose = new Pose2d(-20, -60, Math.toRadians(90));
        drive.setPoseEstimate(startpose);

        traj0 = drive.trajectoryBuilder(startpose)
                .forward(10)
                .build();

        traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(-8,-25, Math.toRadians(185)))
                .build();

        traj2 = drive.trajectoryBuilder(traj1.end())
               .strafeRight(35)
                .build();

        traj3 = drive.trajectoryBuilder(traj2.end())
               .lineToLinearHeading(new Pose2d(50,10, Math.toRadians(182)))
                .build();

        traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(40,-20, Math.toRadians(0)))
                .build();

        traj5 = drive.trajectoryBuilder(traj4.end())
                .strafeLeft(30)
                .build();

        traj6 = drive.trajectoryBuilder(traj5.end())
                .forward(13)
                .build();



        waitForStart();

        drive.followTrajectory(traj0);
        drive.followTrajectory(traj1);
        pixelIntake(-0.3,2);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        pixelIntake(-0.5,2);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);


        PoseStorage.currentPose = drive.getPoseEstimate();

    }
    public void pixelIntake (double speed,double timeout){
        if (opModeIsActive()) {
            intakeMotor.setPower(speed);
            runtime.reset();

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout)) {

                telemetry.addData("Intake currently active ",intakeMotor.getPower());
                telemetry.update();
            }
            intakeMotor.setPower(0);
        }
    }
}