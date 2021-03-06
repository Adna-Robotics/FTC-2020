package org.firstinspires.ftc.teamcode.UniversalStuff;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "Velo tuner", group = "Utilities")
public class General_Max_Velocity extends LinearOpMode {

    private DcMotorEx Shooter_1, Shooter_2;
    double currentVelocity;
    double maxVelocity = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {
        Shooter_1 = hardwareMap.get(DcMotorEx.class, "Shooter_1");
        Shooter_2 = hardwareMap.get(DcMotorEx.class, "Shooter_2");

        waitForStart();
        while (opModeIsActive()) {
            Shooter_1.setPower(-1);
            Shooter_2.setPower(-1);

            currentVelocity = Shooter_1.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}
