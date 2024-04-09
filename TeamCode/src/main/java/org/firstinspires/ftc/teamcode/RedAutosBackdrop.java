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

@Autonomous(name="RedAutosBackdrop", group="Auto")
public class RedAutosBackdrop extends LinearOpMode {
    private final SubsystemManager subsystemManager = new SubsystemManager();
    private ElapsedTime runtime = new ElapsedTime();
    private static  final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "model_20240211_085857.tflite";
    private static final String[] LABELS = {
        "REDTP",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private Arm arm = null;
    private Claw claw = null;
    private ArmTest armTest;
    private Trajectory leftTraj0, leftTraj1, leftTraj2, leftTraj3, leftTraj4;
    private Trajectory centerTraj0, centerTraj1, centerTraj2, centerTraj3, centerTraj4;
    private Trajectory rightTraj1, rightTraj2, rightTraj3, rightTraj4, rightTraj5, rightTraj6;

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
        Pose2d startpose = new Pose2d(18, -60, Math.toRadians(90));
        drive.setPoseEstimate(startpose);
        arm = new Arm(telemetry, hardwareMap);
        claw = new Claw(hardwareMap, runtime);
        subsystemManager.register(arm);
        subsystemManager.register(claw);

        subsystemManager.init(telemetry);

        initTfod();
        telemetryTfod();
        telemetry.update();

        // Left trajectories
        leftTraj0 = drive.trajectoryBuilder(startpose)
                .forward(10)
                .build();

        leftTraj1 = drive.trajectoryBuilder(leftTraj0.end())
                .lineToLinearHeading(new Pose2d(30,-20, Math.toRadians(185)))
                .build();

        leftTraj2 = drive.trajectoryBuilder(leftTraj1.end())
                .lineToLinearHeading(new Pose2d(40,-15, Math.toRadians(0)))
                .build();

        leftTraj3 = drive.trajectoryBuilder(leftTraj2.end())
                .strafeLeft(15)
                .build();

        leftTraj4 = drive.trajectoryBuilder(leftTraj3.end())
                .forward(13)
                .build();


        // Center trajectories
        centerTraj0 = drive.trajectoryBuilder(startpose)
                .forward(25)
                .build();

        centerTraj1 = drive.trajectoryBuilder(centerTraj0.end())
                .strafeTo( new Vector2d(40,-27))
                .build();

        centerTraj2 = drive.trajectoryBuilder(centerTraj1.end())
                .lineToLinearHeading(new Pose2d(43,-19, Math.toRadians(8)))
                .build();

        centerTraj3 = drive.trajectoryBuilder(centerTraj2.end())
                .strafeLeft(23)
                .build();

        centerTraj4 = drive.trajectoryBuilder(centerTraj3.end())
                .forward(10)
                .build();


        // Right trajectories

        rightTraj1 = drive.trajectoryBuilder(startpose)
                .lineToLinearHeading(new Pose2d(28,-45, Math.toRadians(90)))
                .build();

        rightTraj2 = drive.trajectoryBuilder(rightTraj1.end())
                .back(5)
                .build();

        rightTraj3 = drive.trajectoryBuilder(rightTraj2.end())
                .lineToLinearHeading(new Pose2d(45,-25, Math.toRadians(0)))
                .build();

        rightTraj4 = drive.trajectoryBuilder(rightTraj3.end())
                .back(7)
                .build();

        rightTraj5 = drive.trajectoryBuilder(rightTraj4.end())
                .strafeLeft(30)
                .build();

        rightTraj6 = drive.trajectoryBuilder(rightTraj5.end())
                .forward(10)
                .build();


        waitForStart();

        telemetryTfod();
        telemetry.update();

        sleep(20);


        if (telemetryTfod() > 0 && telemetryTfod() < 200) {
            drive.followTrajectory(leftTraj0);
            claw.groundPos();
            drive.followTrajectory(leftTraj1);
            claw.dropPixelSM();
            sleep(500);
            claw.restPosition();
            drive.followTrajectory(leftTraj2);
            arm.goToBackdrop1Line();
            drive.followTrajectory(leftTraj3);
            drive.followTrajectory(leftTraj4);
        } else if (telemetryTfod() > 200 && telemetryTfod() < 500) {
            claw.groundPos();
            drive.followTrajectory(centerTraj0);
            claw.dropPixelSM();
            sleep(500);
            claw.restPosition();
            drive.followTrajectory(centerTraj1);
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
            drive.followTrajectory(rightTraj5);
            drive.followTrajectory(rightTraj6);
        }


        subsystemManager.update();
        PoseStorage.currentPose = drive.getPoseEstimate();

    }
}
