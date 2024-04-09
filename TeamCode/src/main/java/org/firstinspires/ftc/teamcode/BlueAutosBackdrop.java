package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmTest;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.pinkhawks.ftc.drive.PoseStorage;
import org.pinkhawks.ftc.drive.SampleMecanumDrive;
import org.pinkhawks.ftc.subsystems.SubsystemManager;

import java.util.List;

@Autonomous(name="BlueAutosBackdrop", group="Auto")
public class BlueAutosBackdrop extends LinearOpMode {
    private final SubsystemManager subsystemManager = new SubsystemManager();
    private ElapsedTime runtime = new ElapsedTime();
    private static  final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "model_20240213_132229.tflite";
    private static final String[] LABELS = {
            "BLUETP",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private Arm arm = null;
    private Claw claw = null;
    private Drone drone = null;
    private ArmTest armTest;
    private Trajectory leftTraj1, leftTraj2, leftTraj3, leftTraj4, leftTraj5, leftTraj6;
    private Trajectory centerTraj1, centerTraj2, centerTraj3, centerTraj4;
    private Trajectory rightTraj0, rightTraj1, rightTraj2, rightTraj3, rightTraj4;

    private void initTfod() {

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Amazona's_Webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);

        visionPortal = builder.build();

    }

    private double telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        if (currentRecognitions.size() == 0) return Double.MAX_VALUE;

        Recognition recognition = currentRecognitions.get(0);

        double x = (recognition.getLeft() + recognition.getRight()) / 2;
        double y = (recognition.getTop() + recognition.getBottom()) / 2;

        telemetry.addData("", " ");
        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
        telemetry.addData("- Position", "%.0f / %.0f", x, y);
        telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

        return x;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startpose = new Pose2d(18, 60, Math.toRadians(270));
        drive.setPoseEstimate(startpose);
        arm = new Arm(telemetry, hardwareMap);
        claw = new Claw(hardwareMap, runtime);
        drone = new Drone(hardwareMap);
        subsystemManager.register(arm);
        subsystemManager.register(claw);
        subsystemManager.register(drone);

        subsystemManager.init(telemetry);

        initTfod();
        telemetryTfod();
        telemetry.update();

        // Left trajectories

        leftTraj1 = drive.trajectoryBuilder(startpose)
                .lineToLinearHeading(new Pose2d(22,45, Math.toRadians(280)))
                .build();

        leftTraj2 = drive.trajectoryBuilder(leftTraj1.end())
                .back(5)
                .build();

        leftTraj3 = drive.trajectoryBuilder(leftTraj2.end())
                .lineToLinearHeading(new Pose2d(44,20, Math.toRadians(5)))
                .build();

        leftTraj4 = drive.trajectoryBuilder(leftTraj3.end())
                .back(5)
                .build();

        leftTraj5 = drive.trajectoryBuilder(leftTraj4.end())
                .strafeRight(25)
                .build();

        leftTraj6 = drive.trajectoryBuilder(leftTraj5.end())
                .forward(10)
                .build();



        //Center trajectories
        centerTraj1 = drive.trajectoryBuilder(startpose)
                .forward(24)
                .build();

        centerTraj2 = drive.trajectoryBuilder(centerTraj1.end())
                .lineToLinearHeading(new Pose2d(40,14, Math.toRadians(0)))
                .build();

        centerTraj3 = drive.trajectoryBuilder(centerTraj2.end())
                .strafeRight(20)
                .build();

        centerTraj4 = drive.trajectoryBuilder(centerTraj3.end())
                .forward(12)
                .build();


        // Right trajectories
        rightTraj0 = drive.trajectoryBuilder(startpose)
                .forward(10)
                .build();

        rightTraj1 = drive.trajectoryBuilder(rightTraj0.end())
                .lineToLinearHeading(new Pose2d(34,25, Math.toRadians(180)))
                .build();

        rightTraj2 = drive.trajectoryBuilder(rightTraj1.end())
                .lineToLinearHeading(new Pose2d(40,15, Math.toRadians(0)))
                .build();

        rightTraj3 = drive.trajectoryBuilder(rightTraj2.end())
                .strafeRight(20)
                .build();

        rightTraj4 = drive.trajectoryBuilder(rightTraj3.end())
                .forward(12)
                .build();


        waitForStart();

        telemetryTfod();
        telemetry.update();

        sleep(20);


        if (telemetryTfod() > 0 && telemetryTfod() < 200) {
            claw.groundPos();
            drive.followTrajectory(leftTraj1);
            claw.dropPixelSM();
            sleep(500);
            claw.restPosition();
            drive.followTrajectory(leftTraj2);
            drive.followTrajectory(leftTraj3);
            drive.followTrajectory(leftTraj4);
            drive.followTrajectory(leftTraj5);
            drive.followTrajectory(leftTraj6);
        } else if (telemetryTfod() > 200 && telemetryTfod() < 500) {
            claw.groundPos();
            drive.followTrajectory(centerTraj1);
            claw.dropPixelSM();
            sleep(500);
            claw.restPosition();
            drive.followTrajectory(centerTraj2);
            drive.followTrajectory(centerTraj3);
            drive.followTrajectory(centerTraj4);
        } else if (telemetryTfod() > 500 && telemetryTfod() < 700){
            claw.groundPos();
            drive.followTrajectory(rightTraj1);
            claw.dropPixelSM();
            sleep(500);
            claw.restPosition();
            drive.followTrajectory(rightTraj2);
            drive.followTrajectory(rightTraj3);
            drive.followTrajectory(rightTraj4);
        }


        PoseStorage.currentPose = drive.getPoseEstimate();

    }


}
