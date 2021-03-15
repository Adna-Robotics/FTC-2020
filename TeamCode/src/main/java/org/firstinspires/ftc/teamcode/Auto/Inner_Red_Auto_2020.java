package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Eocv_rings_detection;
import org.firstinspires.ftc.teamcode.UniversalStuff.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.jetbrains.annotations.NotNull;
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

@Autonomous(name="Inner Red 2020")

public class Inner_Red_Auto_2020 extends LinearOpMode {

    Eocv_rings_detection vision = null;


    public MotorEx frontLeft, frontRight, backLeft, backRight, Wobble_Goal, intake;
    //private RevColorSensorV3 IndexColor;
    public CRServo Indexer, intakeRight, intakeLeft;
    public Servo WobbleGrabber;

    //toggles
    boolean WobbleOut = false; //false=in and true=out
    boolean WobbleToggle = true;
    boolean ShooterRunning = false;
    boolean ShooterToggle = true;

    public enum RunMode {
        VelocityControl, PositionControl, RawPower
    }

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


        //powershots
        /*
        Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

         */

        Trajectory ps1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        Trajectory ps2 = drive.trajectoryBuilder(ps1.end())
                .lineToConstantHeading(new Vector2d(-56, -11.5))
                .build();

        Trajectory ps3 = drive.trajectoryBuilder(ps2.end())
                .lineToConstantHeading(new Vector2d(-56, -3.5))
                .build();

        telemetry.addData("Powershots:", "Built");
        telemetry.update();

        //0 ring auto
        Trajectory R0_1 = drive.trajectoryBuilder(ps3.end())
                .addDisplacementMarker(() ->{
                    Wobble_Goal.setTargetPosition(WobblePosition);
                    Wobble_Goal.set(1);
                })
                .splineToSplineHeading(new Pose2d(-4, -48, Math.toRadians(160)), Math.toRadians(-135))
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(0.65);
                })
                .splineToSplineHeading(new Pose2d(-32, -50, Math.toRadians(-10)), Math.toRadians(0))
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(0);
                })
                .splineToSplineHeading(new Pose2d(-4, -48, Math.toRadians(160)), Math.toRadians(-70))
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(0.65);
                })
                .splineToSplineHeading(new Pose2d(12, -9, Math.toRadians(90)), Math.toRadians(90))
                .build();

        telemetry.addData("0 Rings:", "Built");
        telemetry.update();

        //1 ring auto
        Trajectory R1_1 = drive.trajectoryBuilder(ps3.end())
                .splineToSplineHeading(new Pose2d(-36, 0, Math.toRadians(270)), Math.toRadians(-45))
                .addDisplacementMarker(() -> {
                    Intake(1);
                })
                .splineToSplineHeading(new Pose2d(-24, -24, Math.toRadians(300)), Math.toRadians(-70))
                .splineToSplineHeading(new Pose2d(0, -36, Math.toRadians(188)), Math.toRadians(0))
                .addDisplacementMarker(() ->{
                    Intake(0);
                })
                .build();

        Trajectory R1_2 = drive.trajectoryBuilder(R1_1.end())
                .addDisplacementMarker(() ->{
                Wobble_Goal.setTargetPosition(-460);
                Wobble_Goal.set(1);
                })
                .splineToSplineHeading(new Pose2d(20, -24, Math.toRadians(135)), Math.toRadians(90))
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(WobbleGrabPosition);
                })
                .splineToSplineHeading(new Pose2d(-18, -48, Math.toRadians(0)), Math.toRadians(220))
                .splineToConstantHeading(new Vector2d(-32, -54), Math.toRadians(160))
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(0.6);
                })
                .splineToSplineHeading(new Pose2d(20, -24, Math.toRadians(160)), Math.toRadians(70))
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(WobbleGrabPosition);
                    Wobble_Goal.setTargetPosition(0);
                    Wobble_Goal.set(1);
                })
                .splineToSplineHeading(new Pose2d(12, -9, Math.toRadians(90)), Math.toRadians(180))
                .build();

        telemetry.addData("1 Ring:", "Built");
        telemetry.update();


        //4 ring auto
        Trajectory R4_1 = drive.trajectoryBuilder(ps3.end())
                .splineToSplineHeading(new Pose2d(38, -38, Math.toRadians(135)), Math.toRadians(-30))
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(WobbleGrabPosition);
                })
                .splineToSplineHeading(new Pose2d(24, -24, Math.toRadians(180)), Math.toRadians(150))
                .addDisplacementMarker(() ->{
                    Intake(1);
                })
                .splineToConstantHeading(new Vector2d(-12, -24), Math.toRadians(180))
                .build();

        Trajectory R4_2  = drive.trajectoryBuilder(R4_1.end())
                .splineToConstantHeading(new Vector2d(-18, -24), Math.toRadians(180))
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

        switch (rings) {
            case ZERO:
                drive.followTrajectory(ps1);
                //Shoot(-1700, 1);


                //drive.followTrajectory(ps2);
                //Shoot(-1700, 1);


                //drive.followTrajectory(ps3);
                //Shoot(-1700, 1);

                //drive.followTrajectory(R0_1);

                //PoseStorage.currentPose = drive.getPoseEstimate();

                break;

            case ONE:
                drive.followTrajectory(ps1);

                Shoot(-1700, 1);

                if (!drive.isBusy()) {
                drive.followTrajectory(ps2);
                Shoot(-1700, 1);
                }

                drive.followTrajectory(ps3);
                Shoot(-1700, 1);

                drive.followTrajectory(R1_1);
                Shoot(Velocity, 1);

                drive.followTrajectory(R1_2);

                PoseStorage.currentPose = drive.getPoseEstimate();
                
                break;

            case FOUR:
                drive.followTrajectory(ps1);
                Shoot(-1700, 1);

                drive.followTrajectory(ps2);
                Shoot(-1700, 1);

                drive.followTrajectory(ps3);
                Shoot(-1700, 1);

                drive.followTrajectory(R4_1);
                Shoot(-1700, 3);

                drive.followTrajectory(R4_2);
                Shoot(-1750, 3);


                PoseStorage.currentPose = drive.getPoseEstimate();

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
            sleep(500);
            Indexer.setPower(0.3);
            sleep(75);
            Indexer.setPower(0);
            sleep(100);
        }
        shooter_1.setVelocity(0);
        shooter_2.setVelocity(0);
        sleep(1000);
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