package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.virtualdevices.Arm;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum velocity your bot can achieve under load. It
 * will also calculate the effective kF value for your velocity PID.
 * <p>
 * Upon pressing start, your bot will run at max power for RUNTIME seconds.
 * <p>
 * Further fine tuning of kF may be desired.
 */
@Disabled
@Config
@Autonomous(group = "drive")
public class ArmMaxVelocityTuner extends LinearOpMode {
    public static double RUNTIME = 2.0;

    private ElapsedTime timer;
    private double maxVelocity = 0.0;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(new Hardware(hardwareMap), NanoClock.system());

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Your bot will go at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Please ensure you have enough space cleared.");
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Please align the robot arm to position 0.\n Press A when the action is complete");
        while (gamepad1.a != true && opModeIsActive()) {
            telemetry.addLine("Please align the robot arm to position 0.\n Press A when the action is complete");
            updateTelemetry(telemetry);
        }
        arm.resetArmPosition();

        telemetry.clearAll();
        telemetry.update();



        timer = new ElapsedTime();

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            arm.getArmPosition();

            maxVelocity = Math.max((arm.getArmPosition())/RUNTIME, maxVelocity);
        }



        double effectiveKf = DriveConstants.getMotorVelocityF(veloInchesToTicks(maxVelocity));

        telemetry.addData("Max Velocity", maxVelocity);
        telemetry.addData("Max Recommended Velocity", maxVelocity * 0.8);
        telemetry.addData("Voltage Compensated kF", effectiveKf * batteryVoltageSensor.getVoltage() / 12);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) idle();
    }

    private double veloInchesToTicks(double inchesPerSec) {
        return inchesPerSec / (2 * Math.PI * DriveConstants.WHEEL_RADIUS) / DriveConstants.GEAR_RATIO * DriveConstants.TICKS_PER_REV;
    }
    private double veloTicksToInches(double ticksPerSec) {
        return 0;
    }
}
