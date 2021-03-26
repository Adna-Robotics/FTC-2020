package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Eocv_rings_detection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static org.firstinspires.ftc.teamcode.Telelop_2020.ShooterF;
import static org.firstinspires.ftc.teamcode.Telelop_2020.ShooterI;
import static org.firstinspires.ftc.teamcode.Telelop_2020.ShooterP;
import static org.firstinspires.ftc.teamcode.Telelop_2020.Velocity;
import static org.firstinspires.ftc.teamcode.Telelop_2020.WobbleGrabPosition;
import static org.firstinspires.ftc.teamcode.Telelop_2020.WobbleMechanismP;
import static org.firstinspires.ftc.teamcode.Telelop_2020.WobblePosition;
@Disabled
@Autonomous(name="Inner Red 2020")

public class Inner_Red_Auto_2020 extends LinearOpMode {

    Eocv_rings_detection vision = null;

    public int i = 0;



    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;
    RINGS height = null;

    enum RINGS {
        ZERO,
        ONE,
        FOUR
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Init:", "Started");
        telemetry.update();


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Servo WobbleGrabber = hardwareMap.get(Servo.class, "Wobble Grabber");
        Motor Wobble_Goal = new Motor(hardwareMap, "Wobble_Goal", Motor.GoBILDA.RPM_223);
        Wobble_Goal.setRunMode(Motor.RunMode.PositionControl);



        Wobble_Goal.setPositionCoefficient(WobbleMechanismP);
        Wobble_Goal.setPositionTolerance(10);

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        telemetry.addData("Hardware:", "Initialized");
        telemetry.update();

        //Camera stuff
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        FtcDashboard.getInstance().startCameraStream(camera, 30);

        telemetry.addData("Camera:", "Initialized");
        telemetry.update();

        WobbleGrabber.setPosition(WobbleGrabPosition);



        //First 3 Shots
        Pose2d startPose = new Pose2d(-63, -48, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        Trajectory Shots = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(0, -24, Math.toRadians(181)), Math.toRadians(75))
                .build();

        telemetry.addData("Shots:", "Built");
        telemetry.update();


        //0 ring auto
        Trajectory R0_1 = drive.trajectoryBuilder(Shots.end())
                .addDisplacementMarker(() ->{
                    Wobble_Goal.setTargetPosition(WobblePosition);
                    Wobble_Goal.set(0.5);
                })
                .splineToSplineHeading(new Pose2d(0, -40, Math.toRadians(120)), Math.toRadians(110))//first wobble drop
                .addDisplacementMarker(18, () ->{
                    WobbleGrabber.setPosition(0.65);
                })
                .splineToSplineHeading(new Pose2d(-24, -32, Math.toRadians(0)), Math.toRadians(180))//grab second wobble
                .addDisplacementMarker(32, () ->{
                    WobbleGrabber.setPosition(WobbleGrabPosition);
                })
                .splineToSplineHeading(new Pose2d(0, -40, Math.toRadians(125)), Math.toRadians(-40))//drop second wobble
                .addDisplacementMarker(66, () ->{
                    WobbleGrabber.setPosition(0.65);
                })
                .addDisplacementMarker(() ->{
                    Wobble_Goal.setTargetPosition(0);
                    Wobble_Goal.set(0);
                })
                .splineToSplineHeading(new Pose2d(12, 0, Math.toRadians(90)), Math.toRadians(90))//park
                .build();

        telemetry.addData("0 Rings:", "Built");
        telemetry.update();

        //1 ring auto
        Trajectory R1_1 = drive.trajectoryBuilder(Shots.end())
                .addDisplacementMarker(() -> {
                    Intake(1);
                })
                .splineToConstantHeading(new Vector2d(-18, -24), Math.toRadians(180))
                .addDisplacementMarker(() ->{
                    Intake(0);
                })
                .build();

        Trajectory R1_2 = drive.trajectoryBuilder(R1_1.end())
                .addDisplacementMarker(() ->{
                Wobble_Goal.setTargetPosition(WobblePosition);
                Wobble_Goal.set(0.5);
                })

                .splineToConstantHeading(new Vector2d(20, -32), Math.toRadians(-10))//place 1st wobble
                .addDisplacementMarker(24, () ->{
                    WobbleGrabber.setPosition(0.65);
                })

                .splineToSplineHeading(new Pose2d(-24, -32, Math.toRadians(0)), Math.toRadians(180))//pick up 2nd wobble
                .addDisplacementMarker(48, () ->{
                    WobbleGrabber.setPosition(WobbleGrabPosition);
                })

                .splineToSplineHeading(new Pose2d(20, -32, Math.toRadians(180)), Math.toRadians(170))//place 2nd wobble
                .addDisplacementMarker(48, () ->{
                    WobbleGrabber.setPosition(0.65);
                })
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(0);
                    Wobble_Goal.setTargetPosition(0);
                    Wobble_Goal.set(0.5);
                })
                .splineToSplineHeading(new Pose2d(12, 0, Math.toRadians(90)), Math.toRadians(90))//park
                .build();

        telemetry.addData("1 Ring:", "Built");
        telemetry.update();


        //4 ring auto
        Trajectory R4_1 = drive.trajectoryBuilder(Shots.end())
                .addDisplacementMarker(() ->{
                    Intake(1);
                })
                .splineToConstantHeading(new Vector2d(-18, -24), Math.toRadians(180))//collect 2 rings
                .addDisplacementMarker(() ->{
                    Intake(0);
                })
                .build();


        Trajectory R4_2  = drive.trajectoryBuilder(R4_1.end())
                .addDisplacementMarker(() ->{
                    Intake(1);
                })
                .splineToConstantHeading(new Vector2d(-24, -24), Math.toRadians(180))//collect other 2 rings
                .addDisplacementMarker(() ->{
                    Intake(0);
                })
                .build();

        Trajectory R4_3 = drive.trajectoryBuilder(R4_2.end())
                .addDisplacementMarker(() ->{
                    Wobble_Goal.setTargetPosition(WobblePosition);
                    Wobble_Goal.set(0.5);
                })
                .splineToSplineHeading(new Pose2d(40, -40, Math.toRadians(150)), Math.toRadians(10))//place first wobble
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(0.65);
                    Wobble_Goal.setTargetPosition(WobblePosition);
                    Wobble_Goal.set(0.5);
                })
                .splineToSplineHeading(new Pose2d(-24, -32, Math.toRadians(0)), Math.toRadians(180))//grab other wobble
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(WobbleGrabPosition);
                })
                .build();

        Trajectory R4_4 = drive.trajectoryBuilder(R4_3.end())
                .addDisplacementMarker(() ->{
                    Wobble_Goal.setTargetPosition(0);
                    Wobble_Goal.set(0.5);
                })
                .splineToSplineHeading(new Pose2d(40, -40, Math.toRadians(150)), Math.toRadians(-65))//place 2nd wobble
                .addDisplacementMarker(50, () ->{
                    Wobble_Goal.set(WobblePosition);
                    Wobble_Goal.set(0.5);
                })
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(0.65);
                    Wobble_Goal.setTargetPosition(0);
                    Wobble_Goal.set(0.5);
                })
                .splineToSplineHeading(new Pose2d(12, 0, Math.toRadians(90)), Math.toRadians(90))//park
                .build();

        telemetry.addData("4 Rings:", "Built");
        telemetry.update();



        while (!isStarted()) {
            String height = " " + pipeline.getHeight();
            telemetry.addData("Ring Stack", height);
            telemetry.update();
        }

        telemetry.addData("Autonomous:", "Ready");
        telemetry.update();


        waitForStart();

        UGContourRingPipeline.Height rings = pipeline.getHeight();

        camera.stopStreaming();
        camera.closeCameraDevice();

        switch (rings) {
            case ZERO:
                drive.followTrajectory(Shots);
                /*
                Shoot(Velocity, 3);

                drive.followTrajectory(R0_1);
                break;

                 */

            case ONE:
                drive.followTrajectory(Shots);
                Shoot(Velocity, 3);


                drive.followTrajectory(R1_1);
                Shoot(-1700, 1);

                drive.followTrajectory(R1_2);


                break;

            case FOUR:
                drive.followTrajectory(Shots);
                Shoot(Velocity, 3);

                drive.followTrajectory(R4_1);
                Shoot(-1800, 3);

                drive.followTrajectory(R4_2);
                Shoot(-1700, 3);

                drive.followTrajectory(R4_3);

                drive.followTrajectory(R4_4);
                break;
        }
    }



    public void Shoot(int velocity, int amount) {
        DcMotorEx shooter_1 = hardwareMap.get(DcMotorEx.class, "Shooter_1");
        DcMotorEx shooter_2 = hardwareMap.get(DcMotorEx.class, "Shooter_2");
        shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_1.setVelocityPIDFCoefficients(ShooterP, ShooterI, 0, ShooterF);
        shooter_2.setVelocityPIDFCoefficients(ShooterP, ShooterI, 0, ShooterF);

        CRServo Indexer = hardwareMap.get(CRServo.class, "Indexer");


        for (i = 0; i < amount; i++){
            shooter_1.setVelocity(velocity);
            shooter_2.setVelocity(shooter_1.getVelocity());
            sleep(150);
            Indexer.setPower(0.3);
            sleep(150);
            Indexer.setPower(0);
            sleep(150);
        }
        shooter_1.setVelocity(0);
        shooter_2.setVelocity(0);
    }
    public void Intake(double power){
        Motor intake = new Motor(hardwareMap, "Intake", 560, 300);
        CRServo intakeRight = hardwareMap.get(CRServo.class, "Right Intake");
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "Left Intake");

            intake.set(power);
            intakeRight.setPower(power);
            intakeLeft.setPower(-power);
    }
}