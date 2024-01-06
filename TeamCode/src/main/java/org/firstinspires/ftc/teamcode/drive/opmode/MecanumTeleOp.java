package org.firstinspires.ftc.teamcode.drive.opmode;

import android.renderscript.Sampler;

import com.acmerobotics.dashboard.config.ValueProvider;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.virtualdevices.HuskyBoy;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="Mecanum TeleOp", group = "drive")
public class MecanumTeleOp extends LinearOpMode {
    Telemetry telemetries;
    double positionsSpeed = 0.1;
    @Override
    public void runOpMode() {
        waitForStart();
        int positionOfArm = 0;
        int sliderPosition = 0;
        boolean started = true;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Hardware hardware = new Hardware(hardwareMap);
        telemetries = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().addConfigVariable("Drive", "Arm To Position Speed",         new ValueProvider<Double>() {public Double get() {return positionsSpeed; } public void set(Double value) {positionsSpeed = value; }});
        
        if (!(drive.getLocalizer() instanceof StandardTrackingWheelLocalizer)) {
            RobotLog.setGlobalErrorMsg("StandardTrackingWheelLocalizer is not being set in the "
                    + "drive class. Ensure that \"setLocalizer(new StandardTrackingWheelLocalizer"
                    + "(hardwareMap));\" is called in SampleMecanumDrive.java");
        }

        hardware.leftSlider.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        boolean manualArmControl = false;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_x;
            double x = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double ly2 = gamepad2.left_stick_y;
            double ry2 = gamepad2.right_stick_y;
            boolean a = gamepad1.a; // extend
            boolean b = gamepad1.b; // retract

            hardware.frontLeft.setPower(0.85*(y + x - rx));
            hardware.frontRight.setPower(0.85*(y - x - rx));
            hardware.backLeft.setPower(0.85*(y - x + rx));
            hardware.backRight.setPower(0.85*(y + x + rx));

            telemetries.addData("Heading Est: ", drive.getPoseEstimate().getHeading());
            telemetries.addData("X Est: ", drive.getPoseEstimate().getX());
            telemetries.addData("Y Est: ", drive.getPoseEstimate().getY());
            if(gamepad2.dpad_left) {
                positionOfArm = 0;
                telemetries.addLine("DPAD_LEFT was pressed");
                hardware.armMotor1.setTargetPosition(positionOfArm);
                hardware.armMotor2.setTargetPosition(positionOfArm);
                hardware.armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor1.setPower(positionsSpeed);
                hardware.armMotor2.setPower(positionsSpeed);

            }
            if(gamepad2.dpad_up) {
                positionOfArm = 128;
                telemetries.addLine("DPAD_UP was pressed");
                hardware.armMotor1.setTargetPosition(positionOfArm);
                hardware.armMotor2.setTargetPosition(positionOfArm);
                hardware.armMotor1.setTargetPositionTolerance(0);
                hardware.armMotor1.setTargetPositionTolerance(0);
                hardware.armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor1.setPower(positionsSpeed);
                hardware.armMotor2.setPower(positionsSpeed);
            }
            if(gamepad2.dpad_right) {
                positionOfArm = 260;


                telemetries.addLine("DPAD_RIGHT was pressed");
                hardware.armMotor1.setTargetPosition(positionOfArm);
                hardware.armMotor2.setTargetPosition(positionOfArm);
                 hardware.armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor1.setPower(positionsSpeed);
                hardware.armMotor2.setPower(positionsSpeed);

            }
            if (gamepad2.left_bumper) {
                if ( manualArmControl ) {
                    manualArmControl = false;
                    telemetries.addLine("LEFT_BUMPER pressed, manual mode inactive");
                } else {
                    manualArmControl = true;
                    telemetries.addLine("LEFT_BUMPER pressed, manual mode active");
                    hardware.armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hardware.armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hardware.armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    hardware.armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
            if ( gamepad2.right_bumper && manualArmControl) {
                hardware.armMotor1.setPower(0);
                hardware.armMotor2.setPower(0);
            }
            if (manualArmControl) {
                hardware.armMotor1.setPower(gamepad2.right_stick_y);
                hardware.armMotor2.setPower(gamepad2.right_stick_y);
                telemetries.addData("Manual Arm Control", manualArmControl);
                telemetries.addData("arm motors power input", hardware.armMotor1.getPower());
            }
            telemetries.addLine(" Arm Motor 1; \n\t Current: " + String.valueOf(hardware.armMotor1.getCurrent(CurrentUnit.AMPS)) + "\n\t Position: " + String.valueOf(hardware.armMotor1.getCurrentPosition()) + "\n\t Target: " + String.valueOf(hardware.armMotor1.getTargetPosition()));
            telemetries.addLine(" Arm Motor 2; \n\t Current: " + String.valueOf(hardware.armMotor2.getCurrent(CurrentUnit.AMPS)) + "\n\t Position: " + String.valueOf(hardware.armMotor2.getCurrentPosition()) + "\n\t Target: " + String.valueOf(hardware.armMotor2.getTargetPosition()));
            telemetries.addLine(" \n Current Voltage: " + String.valueOf(hardware.batteryVoltageSens.getVoltage()));
            telemetries.addLine(" \n");
            telemetries.addLine(" Slider Motor Left; \n\t Current: " + String.valueOf(hardware.leftSlider.getCurrent(CurrentUnit.AMPS)) + "\n\t Position: " + String.valueOf(hardware.leftSlider.getCurrentPosition()) + "\n\t Target: " + String.valueOf(hardware.leftSlider.getTargetPosition()));
            telemetries.addLine(" Slider Motor Right; \n\t Current: " + String.valueOf(hardware.rightSlider.getCurrent(CurrentUnit.AMPS)) + "\n\t Position: " + String.valueOf(hardware.rightSlider.getCurrentPosition()) + "\n\t Target: " + String.valueOf(hardware.rightSlider.getTargetPosition()));

            if (gamepad2.a) {
                hardware.clawWrist.setPosition(0.45);
            }
            if (gamepad2.b) {
                hardware.clawWrist.setPosition(0);

            }
            if (gamepad2.x) {
                hardware.clawBack.setPosition(0.15);
                hardware.clawFront.setPosition(0.15);
            }
            if (gamepad2.y) {
                hardware.clawFront.setPosition(0.05);
                hardware.clawBack.setPosition(0.05);
            }
            if(gamepad1.a) {
                sliderPosition = 0;
                telemetries.addLine("gp1 a");
                hardware.leftSlider.setTargetPosition(sliderPosition);
                hardware.rightSlider.setTargetPosition(sliderPosition);
                hardware.leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.leftSlider.setPower(positionsSpeed);
                hardware.rightSlider.setPower(positionsSpeed);
            }
            if(gamepad1.x) {
                sliderPosition = 1040;
                telemetries.addLine("gp1 x");
                hardware.leftSlider.setTargetPosition(sliderPosition);
                hardware.rightSlider.setTargetPosition(sliderPosition);
                hardware.leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.leftSlider.setPower(positionsSpeed);
                hardware.rightSlider.setPower(positionsSpeed);
            }
            if(gamepad1.y) {
                sliderPosition = 1650;
                telemetries.addLine(    "gp1 y");
                hardware.leftSlider.setTargetPosition(sliderPosition);
                hardware.rightSlider.setTargetPosition(sliderPosition);
                hardware.leftSlider.setPower(positionsSpeed);
                hardware.rightSlider.setPower(positionsSpeed);
                hardware.leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

//            HuskyBoy huskyBoy = new HuskyBoy(hardware);
//            telemetries.addLine(huskyBoy.scanTag().toString());

            drive.getLocalizer().update();
            telemetries.update();

//            if (gamepad1.a) {
//                hardware.leftSlider.setTargetPosition(0);
//                hardware.rightSlider.setTargetPosition(0);
//                telemetries.addLine("Sliders Position 0;");
//            }
//            if (gamepad1.b) {
//                hardware.leftSlider.setTargetPosition(560);
//                hardware.rightSlider.setTargetPosition(560);
//                telemetries.addLine("Sliders Position 1 Rotation;");
//            }
//            if (gamepad1.x) {
//                hardware.leftSlider.setTargetPosition(0);
//                hardware.rightSlider.setTargetPosition(0);
//                telemetries.addLine("Sliders Position 0;");
//            }
//            if (gamepad1.y) {
//                hardware.leftSlider.setTargetPosition(0);
//                hardware.rightSlider.setTargetPosition(0);
//                telemetries.addLine("Sliders Position 0;");
//            }
        }
    }
}
