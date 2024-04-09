package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmTest;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Hanging;
import org.pinkhawks.ftc.drive.PoseStorage;
import org.pinkhawks.ftc.drive.SampleMecanumDrive;
import org.pinkhawks.ftc.subsystems.SubsystemManager;

@TeleOp(name="FieldCentricTeleOp", group="TeleOp")
public class FieldCentricTeleOp extends LinearOpMode {
    private final SubsystemManager subsystemManager = new SubsystemManager();
    private Hanging hang = null;
    private Arm arm = null;
    private Claw claw = null;
    private Drone drone = null;
    private ArmTest armTest;
    private double leftPos;
    private double rightPos;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hang = new Hanging(telemetry, hardwareMap);
        arm = new Arm(telemetry, hardwareMap, armTest);
        claw = new Claw(hardwareMap);
        drone = new Drone(hardwareMap);
        subsystemManager.register(hang);
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
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            if (gamepad2.dpad_up) {
                arm.goToBackdrop1Line();
            } else if (gamepad2.dpad_down) {
                arm.goToGround();
            } else if (gamepad2.dpad_right) {
                arm.goToBackdrop2Line();
            } else if (gamepad2.dpad_left) {
                arm.hang();
            }

            if(gamepad1.y){
                hang.hangRobot();
            }

            if (gamepad2.y) {
                claw.groundPos();
            } else if (gamepad2.x) {
                claw.backdropPos();
            } else
                claw.restPosition();

            if (gamepad2.right_bumper) {
                claw.dropRightPixel();
            }

            if (gamepad2.left_bumper) {
                claw.dropLeftPixel();
            }


            if (gamepad2.b) {
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