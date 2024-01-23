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

@Autonomous(name="AutoRightBLUE", group="Auto")
public class ValkyriesAutoRightB extends LinearOpMode {

    private final SubsystemManager subsystemManager = new SubsystemManager();

    private Trajectory traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intakeMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startpose = new Pose2d(-20,60, Math.toRadians(270));
        drive.setPoseEstimate(startpose);

        traj1 = drive.trajectoryBuilder(startpose)
                .lineToLinearHeading(new Pose2d(-5,18, Math.toRadians(178)))
                .build();

        traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(48,20))
                .build();

        traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(48,15, Math.toRadians(2)))
                .build();

        traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(23)
                .build();

        traj5 = drive.trajectoryBuilder(traj4.end())
                .forward(15)
                .build();

        /*traj4 = drive.trajectoryBuilder(traj3.end(),true)
                .strafeTo(new Vector2d(50,14))
                .build();

        traj5 = drive.trajectoryBuilder(traj4.end(),true)
                .strafeTo(new Vector2d(50,18))
                .build();

        traj6 = drive.trajectoryBuilder(traj5.end(),true) // Pixels
                .back(60)
                .build();

        traj7 = drive.trajectoryBuilder(traj6.end())
                .strafeLeft(25)
                .build();

        traj8 = drive.trajectoryBuilder(traj7.end())
                .back(30)
                .build();

        traj9 = drive.trajectoryBuilder(traj8.end())
                .lineToSplineHeading(new Pose2d(-25,18, Math.toRadians(178)))
                .build();
*/


        waitForStart();

        drive.followTrajectory(traj1);
        pixelIntake(-0.3,2);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        pixelIntake(-0.5,2);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        /*drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
*/
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