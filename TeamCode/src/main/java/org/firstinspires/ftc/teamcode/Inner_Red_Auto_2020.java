package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.jetbrains.annotations.NotNull;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@Autonomous(name="Inner Red 2020")

public class Inner_Red_Auto_2020 extends LinearOpMode {

    private MecanumDrive driveTrain;
    private MotorEx frontLeft, frontRight, backLeft, backRight, Wobble_Goal, Shooter_1, Shooter_2, Intake;
    private RevColorSensorV3 IndexColor;
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


        Servo WobbleGrabber = hardwareMap.get(Servo.class, "Wobble Grabber");
        CRServo Indexer = hardwareMap.get(CRServo.class, "Indexer");
        CRServo intakeRight = hardwareMap.get(CRServo.class, "Right Intake");
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "Left Intake");

        Motor frontRight = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_1150);
        Motor frontLeft = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_1150);
        Motor backRight = new Motor(hardwareMap, "rearRight", Motor.GoBILDA.RPM_1150);
        Motor backLeft = new Motor(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_1150);
        Motor Wobble_Goal = new Motor(hardwareMap, "Wobble_Goal", Motor.GoBILDA.RPM_223);
        Motor Shooter_1 = new Motor(hardwareMap, "Shooter_1", 28, 6000);
        Motor Shooter_2 = new Motor(hardwareMap, "Shooter_1", 28, 6000);
        Motor Intake = new Motor(hardwareMap, "Intake", 560, 300);

        PIDFCoefficients veloCoeff = ((DcMotorEx) frontLeft.motor).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        backRight.setRunMode(Motor.RunMode.VelocityControl);
        backLeft.setRunMode(Motor.RunMode.VelocityControl);
        Wobble_Goal.setRunMode(Motor.RunMode.PositionControl);
        Shooter_1.setRunMode(Motor.RunMode.RawPower);
        Shooter_2.setRunMode(Motor.RunMode.RawPower);
        Intake.setRunMode(Motor.RunMode.RawPower);

        frontRight.setVeloCoefficients(veloCoeff.p, veloCoeff.i, veloCoeff.d);
        frontLeft.setVeloCoefficients(veloCoeff.p, veloCoeff.i, veloCoeff.d);
        backRight.setVeloCoefficients(veloCoeff.p, veloCoeff.i, veloCoeff.d);
        backLeft.setVeloCoefficients(veloCoeff.p, veloCoeff.i, veloCoeff.d);
        Wobble_Goal.setVeloCoefficients(veloCoeff.p, veloCoeff.i, veloCoeff.d);
        Shooter_1.setVeloCoefficients(veloCoeff.p, veloCoeff.i, veloCoeff.d);
        Shooter_2.setVeloCoefficients(veloCoeff.p, veloCoeff.i, veloCoeff.d);

        Wobble_Goal.setPositionCoefficient(0.05);
        Wobble_Goal.setPositionTolerance(56);

        MecanumDrive mecanum = new MecanumDrive(
                frontLeft, frontRight, backLeft, backRight
        );

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

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

        while (!isStarted()) {
            String height = " " + pipeline.getHeight();
            telemetry.addData("Ring Stack", height);
            telemetry.update();
        }

        waitForStart();

        UGContourRingPipeline.Height rings = pipeline.getHeight();

        switch (rings) {
            case ZERO:
                Shoot(0.85, 3);
                Intake("forward");

                break;

            case ONE:
                Shoot(0.85, 3);


                break;

            case FOUR:
                Shoot(0.85, 3);
                Intake("reverse");

                break;
        }
    }

    private void Shoot(double power, int amount) {
        Shooter_1.set(power);
        Shooter_2.set(Shooter_1.get());
        Indexer.setPower(1);
        sleep(220 * amount);
        if (IndexColor.red() > 0.5) {
            Indexer.setPower(0);
        }
    }
    private void Intake(@NotNull String direction){
        if(direction.contains("forward")){
            Intake.set(1);
            intakeRight.setPower(1);
            intakeLeft.setPower(-1);
        }
        else if(direction.contains("reverse")){
            Intake.set(-1);
            intakeRight.setPower(1);
            intakeLeft.setPower(-1);
        }
    }
}