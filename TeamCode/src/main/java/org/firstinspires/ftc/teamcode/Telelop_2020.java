package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// TODO: -power shot auto aim

//@Config
@TeleOp(name = "Drive 2020")
public class Telelop_2020 extends LinearOpMode {

    public static int Velocity = -1750;
    public static double IndexerPosition = 0.3;
    public static double WobbleGrabPosition = 0.03;
    public static int WobblePosition= -220;
    public static double WobbleMechanismP = 0.005;
    public static double ShooterP = 14;
    public static double ShooterI = 0.4;
    public static double ShooterF = 13.5;
    public static int ShooterTiming = 100;
    //public double ShooterActualVelocity = 0;
    //public double ShooterTargetVelocity = 0;


    //Wobble goal toggle switch
    boolean WobbleOut = false; //false=in and true=out
    boolean WobbleToggle = true;

    boolean WobbleGrab = true;
    boolean WobbleGrabToggle = true;

    //Shooter toggle switch
    boolean ShooterRunning = false;
    boolean ShooterToggle = true;

    boolean Powershot = false;
    boolean PowershotToggle = true;











    @Override
    public void runOpMode() throws InterruptedException {


        //Servos
        Servo WobbleGrabber = hardwareMap.get(Servo.class, "Wobble Grabber");
        CRServo Indexer = hardwareMap.get(CRServo.class, "Indexer");
        CRServo intakeRight = hardwareMap.get(CRServo.class, "Right Intake");
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "Left Intake");

        intakeLeft.setDirection(CRServo.Direction.REVERSE);

        //Sensors
        RevColorSensorV3 indexColor = hardwareMap.get(RevColorSensorV3.class, "Index Sensor");
        indexColor.setGain(2);

        //Motors
        Motor frontRight = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_1150);
        Motor frontLeft = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_1150);
        Motor backRight = new Motor(hardwareMap, "rearRight", Motor.GoBILDA.RPM_1150);
        Motor backLeft = new Motor(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_1150);
        Motor Wobble_Goal = new Motor(hardwareMap, "Wobble_Goal", Motor.GoBILDA.RPM_223);
        Motor Intake = new Motor(hardwareMap, "Intake", 560, 300);

        DcMotorEx shooter_1 = hardwareMap.get(DcMotorEx.class, "Shooter_1");
        DcMotorEx shooter_2 = hardwareMap.get(DcMotorEx.class, "Shooter_2");

        //Behavior
        frontRight.setRunMode(Motor.RunMode.RawPower);
        frontLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        Wobble_Goal.setRunMode(Motor.RunMode.PositionControl);
        Intake.setRunMode(Motor.RunMode.RawPower);
        Wobble_Goal.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter_1.setVelocityPIDFCoefficients(ShooterP, ShooterI, 0, ShooterF);
        shooter_2.setVelocityPIDFCoefficients(ShooterP, ShooterI, 0, ShooterF);



        Wobble_Goal.setPositionCoefficient(WobbleMechanismP);
        Wobble_Goal.setPositionTolerance(10);


        MecanumDrive mecanum = new MecanumDrive(
                frontLeft, frontRight, backLeft, backRight
        );

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        ElapsedTime ShootTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ShootTimer.reset();





        waitForStart();

        WobbleGrabber.setPosition(WobbleGrabPosition);
        sleep(200);
        Wobble_Goal.setTargetPosition(0);

        while (opModeIsActive()) {
            //ftc dashboard tuning
            /*
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            shooter_1.setVelocityPIDFCoefficients(ShooterP, ShooterI, 0, ShooterF);
            shooter_2.setVelocityPIDFCoefficients(ShooterP, ShooterI, 0, ShooterF);

            ShooterActualVelocity = shooter_1.getVelocity();
            ShooterTargetVelocity = Velocity;


            Wobble_Goal.setPositionCoefficient(WobbleMechanismP);

             */

            mecanum.driveRobotCentric(-gamepad1.right_stick_x, gamepad1.right_stick_y, -gamepad1.left_stick_x/1.25 + (gamepad1.left_trigger/4) - (gamepad1.right_trigger/4));






            //Wobble goal in/out control
            if(gamepad2.dpad_left && WobbleToggle){
                WobbleToggle=false;
                if(!WobbleOut){
                    Wobble_Goal.setTargetPosition(WobblePosition);
                    WobbleOut=true;
                }
                else{
                    Wobble_Goal.setTargetPosition(0);
                    WobbleOut=false;
                }
            }
            else if(!gamepad2.dpad_left && !WobbleToggle){
                WobbleToggle=true;
            }

            if(!Wobble_Goal.atTargetPosition() && opModeIsActive()){
                Wobble_Goal.set(0.35);
            }
            else{
                Wobble_Goal.stopMotor();
            }



            //Wobble grab control
            if (gamepad2.dpad_up && WobbleGrabToggle){
                WobbleGrabToggle=false;
                if(!WobbleGrab){
                    WobbleGrabber.setPosition(WobbleGrabPosition);
                    WobbleGrab=true;
                }
                else{
                    WobbleGrabber.setPosition(0.65);
                    WobbleGrab=false;
                }
            }
            else if(!gamepad2.dpad_up && !WobbleGrabToggle){
                WobbleGrabToggle=true;
            }



            //Shooter on/off control
            if(gamepad2.b && ShooterToggle){
                ShooterToggle=false;
                if(!ShooterRunning){
                    shooter_1.setVelocity(Velocity);
                    shooter_2.setVelocity(Velocity);
                    ShooterRunning=true;
                }
                else{
                    shooter_1.setVelocity(0);
                    shooter_2.setVelocity(0);
                    ShooterRunning=false;
                }
            }
            else if(!gamepad2.b && !ShooterToggle){
                ShooterToggle=true;
            }



            //Shooting Rings Control
            if(gamepad2.y){
                Indexer.setPower(IndexerPosition);
            }

            else if(gamepad2.a){
                if(ShootTimer.time()<ShooterTiming){
                    Indexer.setPower(IndexerPosition);
                }
                else if(ShootTimer.time()>ShooterTiming*2){
                    Indexer.setPower(0);
                }
            }
            else{
                Indexer.setPower(0);
            }

            if(ShootTimer.time()>ShooterTiming*3){
                ShootTimer.reset();
            }


            //shooter velocity toggle
            if(gamepad1.back && PowershotToggle){
                PowershotToggle=false;
                if(!Powershot){
                    Velocity = -1500;
                    Powershot=true;
                }
                else{
                    Velocity = -1710;
                    Powershot=false;
                }
            }
            else if(!gamepad1.back && !PowershotToggle){
                PowershotToggle=true;
            }




            //Intake control
            if (gamepad1.right_bumper) {
                Intake.set(1);
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
            }
            else if(gamepad1.left_bumper){
                Intake.set(-1);
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
            }
            else{
                Intake.set(0);
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
            }

             



            //Telemetry
            telemetry.addData("Powershot Speed", Powershot);
            telemetry.addData("Shooter Running", ShooterRunning);
            telemetry.addData("Wobble Position", WobbleOut);
            telemetry.addData("Wobble Grab", WobbleGrab);
            telemetry.addData("Shooter Velocity", shooter_1.getVelocity());
            telemetry.addData("Target Velocity", Velocity);
            telemetry.update();
        }
    }
}