package org.firstinspires.ftc.teamcode.UniversalStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "Velo tuner", group = "Utilities")
public class General_Max_Velocity extends LinearOpMode {

    double currentVelocity;
    double maxVelocity = 0.0;


    @Override
    public void runOpMode(){
        DcMotorEx shooter_1 = hardwareMap.get(DcMotorEx.class, "Shooter_1");
        DcMotorEx shooter_2 = hardwareMap.get(DcMotorEx.class, "Shooter_2");

        waitForStart();
        while (opModeIsActive()) {
            shooter_1.setPower(-1);
            shooter_2.setPower(-1);

            currentVelocity = shooter_1.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}
