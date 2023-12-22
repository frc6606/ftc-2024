package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class TargetPosition extends LinearOpMode {

    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightRear = null;

    @Override
    public void runOpMode() throws InterruptedException {

        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "rearRight");
        leftRear = hardwareMap.get(DcMotorEx.class, "rearLeft");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setTargetPosition(2263);
        leftFront.setTargetPosition(2263);
        rightRear.setTargetPosition(2263);
        leftRear.setTargetPosition(2263);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        rightFront.setPower(0.3);
        leftFront.setPower(0.3);
        rightRear.setPower(0.3);
        leftRear.setPower(0.3);

        while(opModeIsActive() && rightFront.isBusy() && leftFront.isBusy() && rightRear.isBusy() && leftRear.isBusy()){
            telemetry.addData("ticks", leftFront.getCurrentPosition() + rightFront.getCurrentPosition() / 2);
            telemetry.update();
        }

    }
}

