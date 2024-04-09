package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.actions.ArmPos1Line;
import org.pinkhawks.ftc.subsystems.Subsystem;

@Config
public class Arm implements Subsystem {
    private PIDController controller;
    public static double  p = 0.021, i = 0, d = 0.0007; //p = 0.005, i = 0, d = 0.00009
    public static double f = 0.1; // 0.2
    private final double ticks_in_degree = 103.6 / 180.0;
    public static int backdrop1LineTarget = 400;
    public static int getBackdrop2LineTarget = 850;
    public static int groundTarget = 100;
    public static int hangingTarget = 700;
    private DcMotorEx armMotor;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Arm(Telemetry telemetry, HardwareMap hardwareMap, ArmTest armTest) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void loop(int target){
        controller.setPID(p, i, d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power  = pid + ff;


        armMotor.setPower(power * 0.1);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }

    public void goToBackdrop1Line() {
        /*controller.setPID(p, i, d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, backdrop1LineTarget);
        double ff = Math.cos(Math.toRadians(backdrop1LineTarget / ticks_in_degree)) * f;

        double power  = pid + ff;

        armMotor.setPower(power * 0.3);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", backdrop1LineTarget);
        telemetry.update();
         */
        this.loop(backdrop1LineTarget);
    }

    public void goToBackdrop2Line() {
       /* controller.setPID(p, i, d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, getBackdrop2LineTarget);
        double ff = Math.cos(Math.toRadians(getBackdrop2LineTarget / ticks_in_degree)) * f;

        double power  = pid + ff;

        armMotor.setPower(power * 0.3);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", getBackdrop2LineTarget);
        telemetry.update();

        */
        this.loop(getBackdrop2LineTarget);
    }

    public void goToGround() {
        /*controller.setPID(p, i, d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, groundTarget);
        double ff = Math.cos(Math.toRadians(groundTarget / ticks_in_degree)) * f;

        double power  = pid + ff;

        armMotor.setPower(power *  0.3);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", groundTarget);
        telemetry.update();

         */

        this.loop(groundTarget);
    }

    public void hang() {
        /*controller.setPID(p, i, d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, hangingTarget);
        double ff = Math.cos(Math.toRadians(hangingTarget / ticks_in_degree)) * f;

        double power  = pid + ff;
        armMotor.setPower(power * 0.3);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", hangingTarget);
        telemetry.update();

         */
        this.loop(hangingTarget);
    }

    public DcMotorEx getMotor(){
        return armMotor;

    }

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotor = hardwareMap.get(DcMotorEx.class,"armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public int armPos(){
        return armMotor.getCurrentPosition();
    }

    @Override
    public void log() {

    }

    @Override
    public void update() {

    }

    @Override
    public boolean check() {
        return false;
    }


}
