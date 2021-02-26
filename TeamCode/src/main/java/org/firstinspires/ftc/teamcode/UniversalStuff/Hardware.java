package org.firstinspires.ftc.teamcode.UniversalStuff;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

public class Hardware{


    private HardwareMap hardwareMap;
    private Map<String, MotorEx> motors;
    private Map<String, Servo> servos;
    private Map<String, CRServo> CRservos;
    private Map<String, RevColorSensorV3> colorsensor;

    public Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.motors = new HashMap<>();
        this.servos = new HashMap<>();
        this.CRservos = new HashMap<>();
        this.colorsensor = new HashMap<>();

        motors.put("frontRight", (MotorEx) new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_1150));
        motors.put("frontLeft", (MotorEx) new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_1150));
        motors.put("rearRight", (MotorEx) new Motor(hardwareMap, "rearRight", Motor.GoBILDA.RPM_1150));
        motors.put("rearLeft", (MotorEx) new Motor(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_1150));

        servos.put("Wobble Grabber", hardwareMap.servo.get("Wobble Grabber"));

        CRservos.put("Indexer", hardwareMap.get(CRServo.class, "Indexer"));

        colorsensor.put("Index Sensor", hardwareMap.get(RevColorSensorV3.class, "Index Sensor"));


    }


    public HardwareMap getHardwareMap() {
            return hardwareMap;
    }

    public Map<String, MotorEx> getMotors() {
        return motors;
    }


    public Map<String, Servo> getServos() {
        return servos;
    }

    public Map<String, CRServo> getCRServos() {
        return CRservos;
    }

    public Map<String, RevColorSensorV3> getTouchSensors() {
        return colorsensor;
    }



}
