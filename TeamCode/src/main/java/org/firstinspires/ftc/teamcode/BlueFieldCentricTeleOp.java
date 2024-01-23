package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.pinkhawks.ftc.drive.PoseStorage;
import org.pinkhawks.ftc.drive.SampleMecanumDrive;

@TeleOp(name="BLUEFieldCentricTeleOp", group="TeleOp")
public class BlueFieldCentricTeleOp extends LinearOpMode {

    private DcMotor intakeMotor, hangingLeftMotor, hangingRightMotor;
    private Servo droneServo;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(PoseStorage.currentPose);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        hangingLeftMotor = hardwareMap.get(DcMotor.class, "hangingLeftMotor");
        hangingRightMotor = hardwareMap.get(DcMotor.class, "hangingRightMotor");
        droneServo = hardwareMap.get(Servo.class,"droneServo");
hangingRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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


            if (gamepad2.right_bumper){
                intakeMotor.setPower(1);
            } else if (gamepad2.left_bumper) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            if (gamepad2.y){
                droneServo.setPosition(0);
            }else {
                droneServo.setPosition(0.6);
            }

            if (gamepad2.dpad_up){
                hangingLeftMotor.setPower(1);
                hangingRightMotor.setPower(1);
            } else if (gamepad2.dpad_down) {
                hangingLeftMotor.setPower(-1);
                hangingRightMotor.setPower(-1);
            } else {
                hangingLeftMotor.setPower(0);
                hangingRightMotor.setPower(0);
            }
            drive.update();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("position",hangingLeftMotor.getCurrentPosition());
            telemetry.addData("position",hangingRightMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}