package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(group = "drive")
public class MecanumTeleOp extends LinearOpMode {
    Telemetry telemetries;
    @Override
    public void runOpMode() {
        waitForStart();
        int positionOfArm = 0;
        boolean started = true;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Hardware hardware = new Hardware(hardwareMap);
        telemetries = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
//        ElapsedTime updateDelta = new ElapsedTime();
        while (opModeIsActive()) {
            double y = gamepad1.right_stick_x;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.left_stick_y;
            double ly2 = gamepad2.left_stick_y;
            double ry2 = gamepad2.right_stick_y;
            boolean a = gamepad1.a; // extend
            boolean b = gamepad1.b; // retract


            hardware.frontLeft.setPower(y + x + rx);
            hardware.frontRight.setPower(y - x - rx);
            hardware.backLeft.setPower(y - x + rx);
            hardware.backRight.setPower(y + x - rx);



            if(gamepad2.dpad_left) {
                positionOfArm = 0;
                telemetries.addLine("DPAD_LEFT was pressed");
                hardware.armMotor1.setTargetPosition(positionOfArm);
                hardware.armMotor2.setTargetPosition(positionOfArm);
                hardware.armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor1.setPower(1);
                hardware.armMotor2.setPower(1);

            }
            if(gamepad2.dpad_up) {
                positionOfArm = 1;
                telemetries.addLine("DPAD_UP was pressed");
                hardware.armMotor1.setTargetPosition(positionOfArm);
                hardware.armMotor2.setTargetPosition(positionOfArm);
                hardware.armMotor1.setTargetPositionTolerance(0);
                hardware.armMotor1.setTargetPositionTolerance(0);
                hardware.armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor1.setPower(1);
                hardware.armMotor2.setPower(1);
            }
            if(gamepad2.dpad_right) {
                positionOfArm = 3;

                telemetries.addLine("DPAD_RIGHT was pressed");
                hardware.armMotor1.setTargetPosition(positionOfArm);
                hardware.armMotor2.setTargetPosition(positionOfArm);
                hardware.armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor1.setPower(1);
                hardware.armMotor2.setPower(1);

            }
            telemetries.addLine(" Arm Motor 1; \n\t Current: " + String.valueOf(hardware.armMotor1.getCurrent(CurrentUnit.AMPS)) + "\n\t Position: " + String.valueOf(hardware.armMotor1.getCurrentPosition()) + "\n\t Target: " + String.valueOf(hardware.armMotor1.getTargetPosition()));
            telemetries.addLine(" Arm Motor 2; \n\t Current: " + String.valueOf(hardware.armMotor2.getCurrent(CurrentUnit.AMPS)) + "\n\t Position: " + String.valueOf(hardware.armMotor2.getCurrentPosition()) + "\n\t Target: " + String.valueOf(hardware.armMotor2.getTargetPosition()));
            telemetries.addLine(" \n Current Voltage: " + String.valueOf(hardware.batteryVoltageSens.getVoltage()));
            telemetries.update();

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
                hardware.clawFront.setPosition(0);
                hardware.clawBack.setPosition(0);

            }

//            SlidingArmVD arm1 = new SlidingArmV("part", "arm1", new HashMap<String, Integer>(), 0, hardware.armMotor1);
//            SlidingArmVD arm2 = new SlidingArmVD("part", "arm1", new HashMap<String, Integer>(), 0, hardware.armMotor2);
//            b
//            if(gamepad2.a) {
//                telemetry.addLine("a");
//                hardware.droneLauncher.setPosition(0.75);
//                telemetry.addLine(Double.toString(hardware.droneLauncher.getPosition()));
//            }
//            if (gamepad2.b) {
//                telemetry.addLine("b");
//                hardware.droneLauncher.setPosition(0.25);
//                telemetry.addLine(Double.toString(hardware.droneLauncher.getPosition()));
//            }
//
//            int positionOfSliderMotors =  20;
//
//
//            if (gamepad2.y) {
//                hardware.leftSlider.setTargetPosition(positionOfSliderMotors);
//                hardware.rightSlider.setTargetPosition(positionOfSliderMotors);
//                telemetry.addLine("Sliders should be moving?");
//                hardware.leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hardware.rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            telemetry.addLine(String.valueOf(hardware.leftSlider.getTargetPosition()));
//            telemetry.addLine(String.valueOf(hardware.rightSlider.getTargetPosition()));
//            telemetry.update();
//
//            int positionOfArm = 20;
//
//            if (gamepad2.x) {
//                hardware.armMotor1.setTargetPosition(positionOfArm);
//                hardware.armMotor2.setTargetPosition(positionOfArm);
//                telemetry.addLine("Arm motor should be moving?");
//                hardware.armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hardware.armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            telemetry.addLine(String.valueOf(hardware.leftSlider.getTargetPosition()));
//            telemetry.addLine(String.valueOf(hardware.rightSlider.getTargetPosition()));
//            telemetry.update();
//
//            if (gamepad2.dpad_left) {
//                hardware.clawWrist.setPosition(0.35);
//            }
//            if(gamepad2.dpad_right){
//                hardware.clawWrist.setPosition(-0.35);
//            }
//            if(gamepad2.dpad_down) {
//                hardware.clawWrist.setPosition(0);
//            }
//
//            if(gamepad2.dpad_up) {
//                hardware.clawBack.setPosition(0.1);
//                hardware.clawFront.setPosition(0.1);
//            }
//            arm1.runWithController(ly2, updateDelta);
//            arm2.runWithController(ry2, updateDelta);
//            updateDelta.reset();

//            int secondaryArmMotors = 15;
//            hardware.leftArm.setTargetPosition(secondaryArmMotors);
//            hardware.rightArm.setTargetPosition(secondaryArmMotors);

        }
    }
}
