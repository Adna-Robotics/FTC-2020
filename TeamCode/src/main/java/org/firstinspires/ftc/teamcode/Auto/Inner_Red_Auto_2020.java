package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Eocv_rings_detection;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static org.firstinspires.ftc.teamcode.Telelop_2020.IndexerPosition;
import static org.firstinspires.ftc.teamcode.Telelop_2020.ShooterF;
import static org.firstinspires.ftc.teamcode.Telelop_2020.ShooterI;
import static org.firstinspires.ftc.teamcode.Telelop_2020.ShooterP;
import static org.firstinspires.ftc.teamcode.Telelop_2020.WobbleGrabPosition;
import static org.firstinspires.ftc.teamcode.Telelop_2020.WobbleMechanismP;
import static org.firstinspires.ftc.teamcode.Telelop_2020.WobblePosition;


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

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Init", "Started");
        telemetry.update();


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Servo WobbleGrabber = hardwareMap.get(Servo.class, "Wobble Grabber");
        Servo RingBlocker = hardwareMap.get(Servo.class, "Ring Blocker");
        Motor Wobble_Goal = new Motor(hardwareMap, "Wobble_Goal", Motor.GoBILDA.RPM_223);
        Wobble_Goal.setRunMode(Motor.RunMode.PositionControl);



        Wobble_Goal.setPositionCoefficient(WobbleMechanismP);
        Wobble_Goal.setPositionTolerance(10);

        WobbleGrabber.setPosition(WobbleGrabPosition);



        telemetry.addData("Hardware:", "Initialized");
        telemetry.update();

        //First 3 Shots
        Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        Trajectory First_Shots = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(-5, -31), Math.toRadians(-40))
                .build();


        Trajectory Collect_First_2 = drive.trajectoryBuilder(First_Shots.end())
                .addDisplacementMarker(() ->{
                    Intake(1);
                })
                .splineToConstantHeading(new Vector2d(-18, -40), Math.toRadians(-10),
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(-4, -30), Math.toRadians(0))
                .addDisplacementMarker(() ->{
                    Intake(0);
                })
                .build();

        Trajectory Collect_Second_2 = drive.trajectoryBuilder(Collect_First_2.end())
                .addDisplacementMarker(() ->{
                    Intake(1);
                })
                .splineToConstantHeading(new Vector2d(-37, -40), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(0, -24), Math.toRadians(0))
                .addDisplacementMarker(() ->{
                    Intake(0);
                })
                .build();

        telemetry.addData("Shots", "Built");
        telemetry.update();


        //0 ring auto
        Trajectory Box_A = drive.trajectoryBuilder(Collect_Second_2.end())
                .lineToLinearHeading(new Pose2d(16, -40, Math.toRadians(90)))
                .addDisplacementMarker(() ->{
                    Wobble_Goal.setTargetPosition(WobblePosition);
                    Wobble_Goal.set(0.5);
                    RingBlocker.setPosition(0.9);
                })
                .build();

        telemetry.addData("0 Rings", "Built");
        telemetry.update();

        //1 ring auto
        Trajectory Box_B = drive.trajectoryBuilder(Collect_Second_2.end())
                .lineToLinearHeading(new Pose2d(20, -24, Math.toRadians(180)))
                .addDisplacementMarker(() ->{
                    Wobble_Goal.setTargetPosition(WobblePosition);
                    Wobble_Goal.set(0.5);
                    RingBlocker.setPosition(0.9);
                })
                .build();

        telemetry.addData("1 Ring", "Built");
        telemetry.update();


        //4 ring auto
        Trajectory Box_C = drive.trajectoryBuilder(Collect_Second_2.end())
                .lineToLinearHeading(new Pose2d(40, -54, Math.toRadians(180)))
                .addDisplacementMarker(() ->{
                    RingBlocker.setPosition(0.9);
                    Wobble_Goal.setTargetPosition(WobblePosition);
                    Wobble_Goal.set(0.5);
                })
                .build();

        telemetry.addData("4 Rings", "Built");
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


        while (!isStarted()) {
            String height = " " + pipeline.getHeight();
            telemetry.addData("Ring Stack", height);
            telemetry.update();
        }

        telemetry.addData("Autonomous:", "Ready");
        telemetry.update();

        UGContourRingPipeline.Height rings = pipeline.getHeight();

        camera.stopStreaming();
        camera.closeCameraDevice();

        waitForStart();

        if(rings == UGContourRingPipeline.Height.ZERO){
            drive.followTrajectory(First_Shots);
            Shoot(-1750, 4);
            drive.followTrajectory(Box_A);
            sleep(1000);
            PlaceWobble();
        }
        else if(rings == UGContourRingPipeline.Height.ONE){
            drive.followTrajectory(First_Shots);
            Shoot(-1750, 4);
            drive.followTrajectory(Collect_First_2);
            Shoot(-1750, 3);
            drive.followTrajectory(Box_B);
            PlaceWobble();
        }
        else{
            drive.followTrajectory(First_Shots);
            RingBlocker.setPosition(0.38);
            Shoot(-1750, 4);
            drive.followTrajectory(Collect_First_2);
            Shoot(-1750, 3);
            drive.followTrajectory(Collect_Second_2);
            Shoot(-1750, 3);
            drive.followTrajectory(Box_C);
            PlaceWobble();
        }


/*
        switch (rings) {
            case ZERO:
                drive.followTrajectory(traj);
                Shoot(-1750, 4);
                Intake(0);
                drive.followTrajectory(R0_1);
                drive.followTrajectory(park);

                break;

            case ONE:
                drive.followTrajectory(traj);
                Shoot(-1750, 4);
                drive.followTrajectory(traj2);
                Shoot(-1750, 3);
                Intake(0);
                drive.followTrajectory(R1_1);
                drive.followTrajectory(park);

                break;

            case FOUR:
                drive.followTrajectory(traj);
                RingBlocker.setPosition(0.38);
                Shoot(-1750, 4);
                drive.followTrajectory(traj2);
                Shoot(-1750, 3);
                drive.followTrajectory(traj3);
                Shoot(-1750, 3);
                Intake(0);
                drive.followTrajectory(R4_1);
                drive.followTrajectory(park);

                break;
        }

 */
    }


    public void PlaceWobble(){
        Motor Wobble_Goal = new Motor(hardwareMap, "Wobble_Goal", Motor.GoBILDA.RPM_223);
        Wobble_Goal.setRunMode(Motor.RunMode.PositionControl);
        Wobble_Goal.setPositionCoefficient(WobbleMechanismP);
        Wobble_Goal.setPositionTolerance(10);

        Servo WobbleGrabber = hardwareMap.get(Servo.class, "Wobble Grabber");

        WobbleGrabber.setPosition(0.65);
        sleep(500);
        Wobble_Goal.setTargetPosition(0);
        Wobble_Goal.set(0.5);
        sleep(200);
    }
    public void Shoot(int velocity, int amount) {
        DcMotorEx shooter_1 = hardwareMap.get(DcMotorEx.class, "Shooter_1");
        DcMotorEx shooter_2 = hardwareMap.get(DcMotorEx.class, "Shooter_2");
        shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_1.setVelocityPIDFCoefficients(ShooterP, ShooterI, 0, ShooterF);
        shooter_2.setVelocityPIDFCoefficients(ShooterP, ShooterI, 0, ShooterF);

        CRServo Indexer = hardwareMap.get(CRServo.class, "Indexer");

        shooter_1.setVelocity(velocity);
        shooter_2.setVelocity(shooter_1.getVelocity());
        sleep(1000);

        for (i = 0; i < amount; i++){
            Indexer.setPower(IndexerPosition);
            sleep(200);
            Indexer.setPower(0);
            sleep(200);
        }
        shooter_1.setVelocity(0);
        shooter_2.setVelocity(0);
        sleep(500);
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