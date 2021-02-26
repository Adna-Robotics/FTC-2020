package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.UniversalStuff.Hardware;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@Config
@TeleOp(name = "Drive 2020 (new)")
public class Telelop_2020 extends LinearOpMode {

    //Hardware hardware = new Hardware(hardwareMap);

    public static double Velocity = 0.85;

    private MecanumDrive driveTrain;
    private MotorEx frontLeft, frontRight, backLeft, backRight, Wobble_Goal, Shooter_1, Shooter_2, Intake;
    //private ServoEx Wobble;
    private RevColorSensorV3 IndexColor;
    private CRServo Indexer, intakeRight, intakeLeft;
    private Servo WobbleGrabber;


    public enum RunMode {
        VelocityControl, PositionControl, RawPower
    }

    //Wobble goal toggle switch
    boolean WobbleOut = false; //false=in and true=out
    boolean WobbleToggle = true;

    boolean WobbleGrab = false;
    boolean WobbleGrabToggle = true;

    //Shooter toggle switch
    boolean ShooterRunning = false;
    boolean ShooterToggle = true;

    //drive mode toggles
    boolean DriveMode = false;
    boolean DriveModeToggle = true;

    String TelemetryDriveMode = "Field Oriented";



    @Override
    public void runOpMode() throws InterruptedException {


        //Servos
        Servo WobbleGrabber = hardwareMap.get(Servo.class, "Wobble Grabber");
        CRServo Indexer = hardwareMap.get(CRServo.class, "Indexer");
        CRServo intakeRight = hardwareMap.get(CRServo.class, "Right Intake");
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "Left Intake");

        intakeLeft.setDirection(CRServo.Direction.REVERSE);

        //Sensors
        IndexColor = hardwareMap.get(RevColorSensorV3.class, "Index Sensor");
        IndexColor.setGain(2);

        //Motors
        Motor frontRight = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_1150);
        Motor frontLeft = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_1150);
        Motor backRight = new Motor(hardwareMap, "rearRight", Motor.GoBILDA.RPM_1150);
        Motor backLeft = new Motor(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_1150);
        Motor Wobble_Goal = new Motor(hardwareMap, "Wobble_Goal", Motor.GoBILDA.RPM_223);
        Motor Shooter_1 = new Motor(hardwareMap, "Shooter_1", 28, 6000);
        Motor Shooter_2 = new Motor(hardwareMap, "Shooter_1", 28, 6000);
        Motor Intake = new Motor(hardwareMap, "Intake", 560, 300);

            //Behavior
        frontRight.setRunMode(Motor.RunMode.RawPower);
        frontLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        Wobble_Goal.setRunMode(Motor.RunMode.RawPower);
        Shooter_1.setRunMode(Motor.RunMode.RawPower);
        Shooter_2.setRunMode(Motor.RunMode.RawPower);
        Intake.setRunMode(Motor.RunMode.RawPower);

        Wobble_Goal.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //Shooter_1.setVeloCoefficients(0, 0, 0);
        //Shooter_2.setVeloCoefficients(0, 0, 0);

        Wobble_Goal.resetEncoder();
        Wobble_Goal.setPositionCoefficient(0.05);
        Wobble_Goal.setPositionTolerance(56);



        MecanumDrive mecanum = new MecanumDrive(
                frontLeft, frontRight, backLeft, backRight
        );

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        mecanum.driveFieldCentric(-gamepad1.right_stick_x, gamepad1.right_stick_y, -gamepad1.left_stick_x, imu.getHeading());

        waitForStart();
        while (opModeIsActive()) {

            NormalizedRGBA colors = IndexColor.getNormalizedColors();

            //dt toggle
            if(gamepad1.back && DriveModeToggle){
                DriveModeToggle=false;
                if(!DriveMode){
                    mecanum.driveFieldCentric(-gamepad1.right_stick_x, gamepad1.right_stick_y, -gamepad1.left_stick_x, imu.getHeading());
                    DriveMode=true;
                }
                else if(DriveMode){
                    mecanum.driveRobotCentric(-gamepad1.right_stick_x, gamepad1.right_stick_y, -gamepad1.left_stick_x);
                    DriveMode=false;
                }
            }
            else if(!gamepad1.back && !DriveModeToggle){
                DriveModeToggle=true;
            }

            if(DriveMode){
                TelemetryDriveMode = "Field Oriented";
            }
            else{
                TelemetryDriveMode = "Robot Oriented";
            }

            //Wobble goal in/out control
            /*
            if(gamepad1.dpad_left){
                WobbleOut=false;
                Wobble_Goal.setTargetPosition(0);
                Wobble_Goal.set(0);
            }
            else if(gamepad1.dpad_right){
                WobbleOut=true;
                Wobble_Goal.setTargetPosition(460);
                Wobble_Goal.set(0);
            }
            if (!Wobble_Goal.atTargetPosition()) {
                Wobble_Goal.set(0.75);
            }
            else {
                Wobble_Goal.stopMotor();
            }
             */

            if(gamepad1.right_trigger>0.01) {
                Wobble_Goal.set(-gamepad1.right_trigger/2);
            }
            else if(gamepad1.left_trigger>0.01){
                Wobble_Goal.set(gamepad1.left_trigger/2);
            }
            else{
                Wobble_Goal.set(0);
            }


            if (gamepad1.x){
                WobbleGrabber.setPosition(1.0);
            }



            //Wobble grab control
            if (gamepad1.dpad_up && WobbleGrabToggle){
                WobbleGrabToggle=false;
                if(!WobbleGrab){
                    WobbleGrabber.setPosition(0.075);
                    WobbleGrab=true;
                }
                else{
                    WobbleGrabber.setPosition(0.6);
                    WobbleGrab=false;
                }
            }
            else if(!gamepad1.dpad_up && !WobbleGrabToggle){
                WobbleGrabToggle=true;
            }




            //Shooter on/off control
            if(gamepad1.b && ShooterToggle){
                ShooterToggle=false;
                if(!ShooterRunning){
                    Shooter_1.set(0.75);
                    Shooter_2.set(0.75);
                    ShooterRunning=true;
                }
                else if(ShooterRunning){
                    Shooter_1.set(0);
                    Shooter_2.set(0);
                    ShooterRunning=false;
                }
            }
            else if(!gamepad1.b && !ShooterToggle){
                ShooterToggle=true;
            }



            //Shooting Rings Control
            if(gamepad1.y){
                Indexer.setPower(0.3);
            }
            else{
                Indexer.setPower(0);
            }



            //Intake control
            if (gamepad1.right_bumper) {
                Intake.set(1);
                intakeLeft.setPower(1);
                intakeRight.setPower(intakeLeft.getPower());
            }
            else if(gamepad1.left_bumper){
                Intake.set(-1);
                intakeLeft.setPower(-1);
                intakeRight.setPower(intakeLeft.getPower());
            }
            else{
                Intake.set(0);
                intakeLeft.setPower(0);
                intakeRight.setPower(intakeLeft.getPower());
            }





            //Telemetry
            telemetry.addData("Drive Mode", TelemetryDriveMode);
            telemetry.addData("Shooter Running", ShooterRunning);
            telemetry.addData("Wobble Position", WobbleOut);
            telemetry.addData("Wobble Grab", WobbleGrab);
            telemetry.addData("Shooter 1 Position", Shooter_1.getCurrentPosition());
            telemetry.addData("Shooter 2 Position", Shooter_2.getCurrentPosition());
            telemetry.addData("Wobble Power", Wobble_Goal.get());
            telemetry.update();
        }
    }
}