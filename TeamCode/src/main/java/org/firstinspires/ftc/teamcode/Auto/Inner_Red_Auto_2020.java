package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
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

@Autonomous(name="Inner Red 2020")

public class Inner_Red_Auto_2020 extends LinearOpMode {

    Eocv_rings_detection vision = null;

    private MecanumDrive driveTrain;
    private MotorEx frontLeft, frontRight, backLeft, backRight, Wobble_Goal, Shooter_1, Shooter_2, intake;
    //private RevColorSensorV3 IndexColor;
    private CRServo Indexer, intakeRight, intakeLeft;
    private Servo WobbleGrabber;

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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Servo WobbleGrabber = hardwareMap.get(Servo.class, "Wobble Grabber");
        CRServo Indexer = hardwareMap.get(CRServo.class, "Indexer");
        CRServo intakeRight = hardwareMap.get(CRServo.class, "Right Intake");
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "Left Intake");


        Motor Wobble_Goal = new Motor(hardwareMap, "Wobble_Goal", Motor.GoBILDA.RPM_223);
        Motor Shooter_1 = new Motor(hardwareMap, "Shooter_1", 28, 6000);
        Motor Shooter_2 = new Motor(hardwareMap, "Shooter_1", 28, 6000);
        Motor intake = new Motor(hardwareMap, "Intake", 560, 300);


        Wobble_Goal.setRunMode(Motor.RunMode.PositionControl);
        Shooter_1.setRunMode(Motor.RunMode.RawPower);
        Shooter_2.setRunMode(Motor.RunMode.RawPower);
        intake.setRunMode(Motor.RunMode.RawPower);


        Wobble_Goal.setPositionCoefficient(0.05);
        Wobble_Goal.setPositionTolerance(56);

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

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

        WobbleGrabber.setPosition(1);


        //powershots
        Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        Trajectory ps1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-56, -18, Math.toRadians(188)))
                .build();

        Trajectory ps2 = drive.trajectoryBuilder(ps1.end())
                .lineToConstantHeading(new Vector2d(-56, -11.5))
                .build();

        Trajectory ps3 = drive.trajectoryBuilder(ps2.end())
                .lineToConstantHeading(new Vector2d(-56, -3.5))
                .build();


        //0 ring auto
        Trajectory R0_1 = drive.trajectoryBuilder(ps3.end())
                .addDisplacementMarker(() ->{
                    Wobble_Goal.setTargetPosition(-460);
                    Wobble_Goal.set(1);
                })
                .splineToLinearHeading(new Pose2d(-4, -48, 160), -45)
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(0);
                })
                .splineToSplineHeading(new Pose2d(-12, -24, 0), 110)
                .splineToConstantHeading(new Vector2d(-4, -48), 90)
                .splineToSplineHeading(new Pose2d(12, -9, 90), 90)
                .build();


        //1 ring auto
        Trajectory R1_1 = drive.trajectoryBuilder(ps3.end())
                .splineToSplineHeading(new Pose2d(-36, 0, 270), -45)
                .addDisplacementMarker(() -> {
                    Intake(1);
                })
                .splineToSplineHeading(new Pose2d(-24, -24, 300), -70)
                .splineToSplineHeading(new Pose2d(0, -36, 188), 0)
                .addDisplacementMarker(() ->{
                    Intake(0);
                })
                .build();

        Trajectory R1_2 = drive.trajectoryBuilder(R1_1.end())
                .addDisplacementMarker(() ->{
                Wobble_Goal.setTargetPosition(-460);
                Wobble_Goal.set(1);
                })
                .splineToSplineHeading(new Pose2d(20, -24, 135), 90)
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(0);
                })
                .splineToSplineHeading(new Pose2d(-18, -48, 0), 220)
                .splineToConstantHeading(new Vector2d(-32, -54), 160)
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(0.6);
                })
                .splineToSplineHeading(new Pose2d(20, -24, 160), 70)
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(0);
                    Wobble_Goal.setTargetPosition(0);
                    Wobble_Goal.set(1);
                })
                .splineToSplineHeading(new Pose2d(12, -9, 90), 180)
                .build();


        //4 ring auto
        Trajectory R4_1 = drive.trajectoryBuilder(ps3.end())
                .splineToLinearHeading(new Pose2d(38, -38, 135), -30)
                .addDisplacementMarker(() ->{
                    WobbleGrabber.setPosition(0);
                })
                .splineToSplineHeading(new Pose2d(24, -24, 180), 150)
                .addDisplacementMarker(() ->{
                    Intake(1);
                })
                .splineToConstantHeading(new Vector2d(-12, -24), 180)
                .build();

        Trajectory R4_2  = drive.trajectoryBuilder(R4_1.end())
                .splineToConstantHeading(new Vector2d(-18, -24), 180)
                .build();



        while (!isStarted()) {
            String height = " " + pipeline.getHeight();
            telemetry.addData("Ring Stack", height);
            telemetry.update();
        }


        waitForStart();

        UGContourRingPipeline.Height rings = pipeline.getHeight();

        switch (rings) {
            case ZERO:
                drive.followTrajectory(ps1);
                Shoot(0.85, 1);


                drive.followTrajectory(ps2);
                Shoot(0.85, 1);


                drive.followTrajectory(ps3);
                Shoot(0.85, 1);

                drive.followTrajectory(R0_1);

                PoseStorage.currentPose = drive.getPoseEstimate();

                break;

            case ONE:
                drive.followTrajectory(ps1);
                Shoot(0.85, 1);

                drive.followTrajectory(ps2);
                Shoot(0.85, 1);

                drive.followTrajectory(ps3);
                Shoot(0.85, 1);

                drive.followTrajectory(R1_1);
                Shoot(0.85, 1);

                drive.followTrajectory(R1_2);

                PoseStorage.currentPose = drive.getPoseEstimate();
                
                break;

            case FOUR:
                drive.followTrajectory(ps1);
                Shoot(0.85, 1);

                drive.followTrajectory(ps2);
                Shoot(0.85, 1);

                drive.followTrajectory(ps3);
                Shoot(0.65, 1);

                drive.followTrajectory(R4_1);
                Shoot(0.85, 3);
                drive.followTrajectory(R4_2);
                Shoot(0.85, 3);


                PoseStorage.currentPose = drive.getPoseEstimate();

                break;
        }
    }



    private void Shoot(double power, int amount) {
        for (i = 0; i < amount; i++){
            Shooter_1.set(power);
            Shooter_2.set(Shooter_1.get());
            sleep(100);
            Indexer.setPower(1);
            sleep(100);
            Indexer.setPower(0);
            sleep(100);
        }
        Shooter_1.set(0);
        Shooter_2.set(Shooter_1.get());
    }
    private void Intake(double power){
            intake.set(power);
            intakeRight.setPower(power);
            intakeLeft.setPower(-power);
    }
}