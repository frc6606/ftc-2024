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

@Autonomous(name="AutoCenterBLUE", group="Auto")
public class ValkyriesAutoCenterB extends LinearOpMode{
    private final SubsystemManager subsystemManager = new SubsystemManager();
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intakeMotor;
    private Trajectory traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startpose = new Pose2d(-20, 60, Math.toRadians(270));
        drive.setPoseEstimate(startpose);

        traj1 = drive.trajectoryBuilder(startpose)
                .forward(25)
                .build();

        traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo( new Vector2d(35,30))
                .build();

        traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(45,14, Math.toRadians(355)))
                .build();

        traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(25)
                .build();

        traj5 = drive.trajectoryBuilder(traj4.end())
                .forward(15)
                .build();


        waitForStart();

        drive.followTrajectory(traj1);
        pixelIntake(-0.3,2);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        pixelIntake(-0.5,2);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);

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
