package org.firstinspires.ftc.teamcode.UniversalStuff;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Disabled
public class Hardware{

    private MecanumDrive driveTrain;
    public MotorEx frontLeft, frontRight, backLeft, backRight, Wobble_Goal, Shooter_1, Shooter_2, Intake;
    private CRServo Indexer, intakeRight, intakeLeft;
    private Servo WobbleGrabber;

    HardwareMap hardwareMap  =  null;


    public void initHardware() {
        Motor frontRight        =   new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_1150);
        Motor frontLeft         =   new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_1150);
        Motor backRight         =   new Motor(hardwareMap, "rearRight", Motor.GoBILDA.RPM_1150);
        Motor backLeft          =   new Motor(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_1150);
        Motor Intake            =   new Motor(hardwareMap, "Intake", 560, 300);
        Motor Shooter_1         =   new Motor(hardwareMap, "Shooter_1", 28, 6000);
        Motor Shooter_2         =   new Motor(hardwareMap, "Shooter_1", 28, 6000);
        Motor Wobble_Goal       =   new MotorEx(hardwareMap, "Wobble Goal", Motor.GoBILDA.RPM_223);

        frontRight.setRunMode(Motor.RunMode.RawPower);
        frontLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        Wobble_Goal.setRunMode(Motor.RunMode.RawPower);
        Shooter_1.setRunMode(Motor.RunMode.RawPower);
        Shooter_2.setRunMode(Motor.RunMode.RawPower);
        Intake.setRunMode(Motor.RunMode.RawPower);
        Wobble_Goal.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        Wobble_Goal.resetEncoder();
        Wobble_Goal.setPositionCoefficient(0.05);
        Wobble_Goal.setPositionTolerance(56);

        CRServo intakeRight     =   hardwareMap.get(CRServo.class, "Right Intake");
        CRServo intakeLeft      =   hardwareMap.get(CRServo.class, "Left Intake");
        CRServo Indexer         =   hardwareMap.get(CRServo.class, "Indexer");

        Servo WobbleGrabber = hardwareMap.get(Servo.class, "Wobble Grabber");

        RevIMU imu              =   new RevIMU(hardwareMap);
        imu.init();

    }
}
