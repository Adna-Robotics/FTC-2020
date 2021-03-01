package org.firstinspires.ftc.teamcode.UniversalStuff;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Max Velocity Tuner", group = "Utilities")
public class General_Max_Velocity extends LinearOpMode {
    Motor Shooter_1, Shooter_2;
    double currentVelocity;
    double maxVelocity = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {
        Motor Shooter_1 = new Motor(hardwareMap, "Shooter_1", 28, 6000);
        Motor Shooter_2 = new Motor(hardwareMap, "Shooter_2", 28, 6000);

        waitForStart();
        while (opModeIsActive()) {
            Shooter_1.set(-1);
            Shooter_2.set(-1);

            currentVelocity = Shooter_1.getCorrectedVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}
