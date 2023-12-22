package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.pinkhawks.ftc.subsystems.SubsystemManager;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name="ValkyriesTeleOp", group="TeleOp")
public class ValkyriesTeleOp extends OpMode {

    private final SubsystemManager subsystemManager = new SubsystemManager();

    private Drivetrain drivetrain = null;

    @Override
    public void init() {
        drivetrain = new Drivetrain(telemetry, hardwareMap);
        subsystemManager.register(drivetrain);

        subsystemManager.init(telemetry);
    }

    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double vd = Math.hypot(x,y);
        double td = Math.atan2(y, x);

        drivetrain.drive(vd, td, turn * 0.6);

        subsystemManager.update();
    }

    @Override
    public void stop() {
        subsystemManager.clear();
    }
}
