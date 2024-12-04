package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name = "AutonomousMecanumPID", group = "Autonomous")
public class AutonomousMecanumPID extends LinearOpMode{
    private MecanumDrive mecanumDrive;
    private PIDController pidX, pidY, pidRotation;
    private Pose2D currentPose;
    private Pose2D targetPose;


    //@Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("left_front_mtr");
        DcMotor frontRight = hardwareMap.dcMotor.get("right_back_mtr");
        DcMotor backLeft = hardwareMap.dcMotor.get("left_back_mtr");
        DcMotor backRight = hardwareMap.dcMotor.get("right_front_mtr");
        GoBildaPinpointDriver odo;

        double oldTime = 0;

        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        pidX = new PIDController(0.1, 0.01, 0.001);  // Adjust PID constants as needed
        pidY = new PIDController(0.1, 0.01, 0.001);
        pidRotation = new PIDController(0.1, 0.01, 0.001);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0);

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;

            currentPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
            targetPose = new Pose2D(DistanceUnit.MM, 50, 50, AngleUnit.DEGREES, 90);  // Example target position

            currentPose = (new Pose2D(DistanceUnit.MM, currentPose.getX(DistanceUnit.MM), currentPose.getY(DistanceUnit.MM), AngleUnit.DEGREES, currentPose.getHeading(AngleUnit.DEGREES)));  // Example update, replace with actual sensor input

            // Compute PID control values for each axis
            double pidOutputX = pidX.compute(targetPose.getX(DistanceUnit.MM), currentPose.getX(DistanceUnit.MM));
            double pidOutputY = pidY.compute(targetPose.getY(DistanceUnit.MM), currentPose.getY(DistanceUnit.MM));
            double pidOutputRotation = pidRotation.compute(targetPose.getHeading(AngleUnit.DEGREES), currentPose.getHeading(AngleUnit.DEGREES));

            // Move the robot based on PID outputs
            mecanumDrive.move(pidOutputX, pidOutputY, pidOutputRotation);

            // You can stop when the robot reaches the target position (optional)
            if (Math.abs(targetPose.getX(DistanceUnit.MM) - currentPose.getX(DistanceUnit.MM)) < 1 && Math.abs(targetPose.getY(DistanceUnit.MM) - currentPose.getY(DistanceUnit.MM)) < 1) {
                mecanumDrive.stop();
            }

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

            telemetry.addData("Status", odo.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();
        }
    }
}

