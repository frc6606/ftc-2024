package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmTest;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Hanging;
import org.pinkhawks.ftc.subsystems.SubsystemManager;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Amazona'sTeleOp", group="TeleOp")
public class TeleOp extends LinearOpMode {

    private final SubsystemManager subsystemManager = new SubsystemManager();
    private Hanging hang = null;
    private Arm arm = null;
    private Claw claw = null;
    private Drone drone = null;
    private Drivetrain drivetrain = null;
    private ArmTest armTest;


    @Override
    public void runOpMode() throws InterruptedException {

        hang = new Hanging(telemetry, hardwareMap);
        arm = new Arm(telemetry, hardwareMap, armTest);
        claw = new Claw(hardwareMap);
        drone = new Drone(hardwareMap);
        drivetrain = new Drivetrain(telemetry, hardwareMap);

        subsystemManager.register(hang);
        subsystemManager.register(arm);
        subsystemManager.register(claw);
        subsystemManager.register(drone);
        subsystemManager.register(drivetrain);

        subsystemManager.init(telemetry);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {


            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double vd = Math.hypot(x, y);
            double td = Math.atan2(y, x);

            drivetrain.drive(vd, td, turn * 0.6);

            if (gamepad2.dpad_up) {
                arm.goToBackdrop1Line();
            } else if (gamepad2.dpad_down) {
                arm.goToGround();
            } else if (gamepad2.dpad_right) {
                arm.goToBackdrop2Line();
            } else if (gamepad2.dpad_left) {
                arm.hang();
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

            telemetry.addData("armPos", arm.armPos());
            telemetry.update();
            subsystemManager.update();
        }
    }
}

