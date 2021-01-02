package org.firstinspires.ftc.teamcode;


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


@TeleOp(name = "Drive 2020 (new)")

public class Telelop_2020 extends LinearOpMode {



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
        Wobble_Goal.setRunMode(Motor.RunMode.PositionControl);
        Shooter_1.setRunMode(Motor.RunMode.RawPower);
        Shooter_2.setRunMode(Motor.RunMode.RawPower);
        Intake.setRunMode(Motor.RunMode.RawPower);

        //Shooter_1.setVeloCoefficients(0.05, 0.01, 0.31);
        //Shooter_2.setVeloCoefficients(0.05, 0.01, 0.31);

        Wobble_Goal.setPositionCoefficient(0.05);
        Wobble_Goal.setPositionTolerance(56);

        MecanumDrive mecanum = new MecanumDrive(
                frontLeft, frontRight, backLeft, backRight
        );

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();


        waitForStart();
        while (opModeIsActive()) {

            NormalizedRGBA colors = IndexColor.getNormalizedColors();

            //dt control (top field oriented, bottom robot oriented)


            mecanum.driveFieldCentric(-gamepad1.right_stick_x, gamepad1.right_stick_y, -gamepad1.left_stick_x, imu.getHeading());
            //mecanum.driveRobotCentric(-gamepad1.right_stick_x, gamepad1.right_stick_y, -gamepad1.left_stick_x);

            //Wobble goal in/out control
            if (gamepad1.a && WobbleToggle){
                WobbleToggle=false;
                if(WobbleOut){
                    WobbleOut=false;
                    Wobble_Goal.setTargetPosition(0);
                }
                else{
                    WobbleOut=true;
                    Wobble_Goal.setTargetPosition(460);
                }
            }
            else if(!gamepad1.a && !WobbleToggle){
                WobbleToggle=true;
            }

            while (!Wobble_Goal.atTargetPosition() && WobbleOut) {
                Wobble_Goal.set(-0.75);
            }
            Wobble_Goal.stopMotor();

            while (!Wobble_Goal.atTargetPosition() && !WobbleOut) {
                Wobble_Goal.set(0.75);
            }
            Wobble_Goal.stopMotor();



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
                    Shooter_1.set(-0.85);
                    Shooter_2.set(Shooter_1.get());
                    ShooterRunning=true;
                }
                else{
                    Shooter_1.set(0);
                    Shooter_2.set(Shooter_1.get());
                    ShooterRunning=false;
                }
            }
            else if(!gamepad1.b && !ShooterToggle){
                ShooterToggle=true;
            }



            //Shooting Rings Control
            if(gamepad1.y){
                Indexer.setPower(-1);
            }
            else if((colors.red > 0.5) && !gamepad1.y){
                Indexer.setPower(0);
            }
            else {
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
            telemetry.addData("Shooter Running", ShooterRunning);
            telemetry.addData("Wobble Position", WobbleOut);
            telemetry.addData("Wobble Grab", WobbleGrab);
            telemetry.update();
        }
    }
}

