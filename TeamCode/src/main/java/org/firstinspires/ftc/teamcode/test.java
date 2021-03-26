package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Telelop_2020.IndexerPosition;
import static org.firstinspires.ftc.teamcode.Telelop_2020.ShooterF;
import static org.firstinspires.ftc.teamcode.Telelop_2020.ShooterI;
import static org.firstinspires.ftc.teamcode.Telelop_2020.ShooterP;
import static org.firstinspires.ftc.teamcode.Telelop_2020.WobbleGrabPosition;
import static org.firstinspires.ftc.teamcode.Telelop_2020.WobbleMechanismP;
import static org.firstinspires.ftc.teamcode.Telelop_2020.WobblePosition;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "Auto 2020", group = "auto")
public class test extends LinearOpMode {


    public int i = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Motor Wobble_Goal = new Motor(hardwareMap, "Wobble_Goal", Motor.GoBILDA.RPM_223);
        Wobble_Goal.setRunMode(Motor.RunMode.PositionControl);
        Wobble_Goal.setPositionCoefficient(WobbleMechanismP);
        Wobble_Goal.setPositionTolerance(10);

        Servo WobbleGrabber = hardwareMap.get(Servo.class, "Wobble Grabber");
        WobbleGrabber.setPosition(WobbleGrabPosition);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose, true)
                .splineToSplineHeading(new Pose2d(-5, -30, Math.toRadians(181)), Math.toRadians(-20))
                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .addDisplacementMarker(() ->{
                    Intake(1);
                })
                .splineToConstantHeading(new Vector2d(-22, -40), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(0, -30), Math.toRadians(0))
                .addDisplacementMarker(() ->{
                    Intake(0);
                })
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .addDisplacementMarker(() ->{
                    Intake(1);
                })
                .splineToConstantHeading(new Vector2d(-48, -40), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(0, -30), Math.toRadians(0))
                .addDisplacementMarker(() ->{
                    Intake(0);
                })
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineToConstantHeading(new Vector2d(24, 0), Math.toRadians(90))
                .addDisplacementMarker(() ->{
                    Wobble_Goal.setTargetPosition(WobblePosition);
                    Wobble_Goal.set(0.5);
                })
                .build();

        telemetry.addData("Paths", "Built");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectory(traj);
        Shoot(-1750, 3);
        drive.followTrajectory(traj2);
        Shoot(-1750, 3);
        drive.followTrajectory(traj3);
        Shoot(-1750, 3);
        drive.followTrajectory(traj4);




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
