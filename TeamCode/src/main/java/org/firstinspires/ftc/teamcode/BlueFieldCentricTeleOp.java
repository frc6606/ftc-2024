package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.pinkhawks.ftc.drive.PoseStorage;
import org.pinkhawks.ftc.drive.SampleMecanumDrive;
import org.pinkhawks.ftc.subsystems.SubsystemManager;

@TeleOp(name="BLUEFieldCentricTeleOp", group="TeleOp")
public class BlueFieldCentricTeleOp extends LinearOpMode {

    private final SubsystemManager subsystemManager = new SubsystemManager();
    private Arm arm = null;
    private Claw claw = null;
    private Drone drone = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = new Arm(telemetry, hardwareMap);
        claw = new Claw(hardwareMap);
        drone = new Drone(hardwareMap);
        subsystemManager.register(arm);
        subsystemManager.register(claw);
        subsystemManager.register(drone);

        subsystemManager.init(telemetry);
        drive.setPoseEstimate(PoseStorage.currentPose);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            if (gamepad1.dpad_up) {
                arm.goToBackdrop();
            } else if (gamepad1.dpad_down) {
                arm.goToGround();
            } else if (gamepad1.dpad_right) {
                arm.hang();
            }

            if (gamepad1.y) {
                claw.groundPos();
            } else if (gamepad1.x) {
                claw.backdropPos();
            } else
                claw.restPosition();

            if (gamepad1.right_bumper) {
                claw.takePixel();
            } else if (gamepad1.left_bumper) {
                claw.dropPixel();
            }

            if (gamepad1.b) {
                drone.launchDrone();
            } else {
                drone.restDronePos();
            }


            drive.update();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}