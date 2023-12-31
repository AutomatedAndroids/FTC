package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class MecanumTeleOpTesting extends LinearOpMode {
    @Override
    public void runOpMode() {

        waitForStart();

        boolean started = true;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Hardware hardware = new Hardware(hardwareMap);
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

//            SlidingArmVD arm1 = new SlidingArmV("part", "arm1", new HashMap<String, Integer>(), 0, hardware.armMotor1);
//            SlidingArmVD arm2 = new SlidingArmVD("part", "arm1", new HashMap<String, Integer>(), 0, hardware.armMotor2);
//            b
            if(gamepad2.a) {
                telemetry.addLine("a");
                hardware.droneLauncher.setPosition(0.75);
                telemetry.addLine(Double.toString(hardware.droneLauncher.getPosition()));
            }
            if (gamepad2.b) {
                telemetry.addLine("b");
                hardware.droneLauncher.setPosition(0.01);
                telemetry.addLine(Double.toString(hardware.droneLauncher.getPosition()));
            }

            int positionOfSliderMotors =  20;


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

            int positionOfArm = 2;

            if (gamepad2.y) {
                positionOfArm = 1;

                telemetry.addLine("y:Arm motor should be moving?");
                telemetry.addData("encoder rightArmMotor", hardware.armMotor2.getCurrentPosition());
                telemetry.addData("encoder leftArmMotor", hardware.armMotor1.getCurrentPosition());
                hardware.armMotor1.setTargetPosition(positionOfArm);
                hardware.armMotor2.setTargetPosition(positionOfArm);
                hardware.armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                hardware.armMotor1.setPower(0.25);
                hardware.armMotor2.setPower(0.25);
            }
            telemetry.addLine(String.valueOf(hardware.leftSlider.getTargetPosition()));
            telemetry.addLine(String.valueOf(hardware.rightSlider.getTargetPosition()));
            telemetry.update();


            if (gamepad2.x) {
                positionOfArm = -3;

                telemetry.addLine("X:Arm motor should be moving?");
                telemetry.addData("encoder rightArmMotor", hardware.armMotor2.getCurrentPosition());
                telemetry.addData("encoder leftArmMotor", hardware.armMotor1.getCurrentPosition());
                hardware.armMotor1.setTargetPosition(positionOfArm);
                hardware.armMotor2.setTargetPosition(positionOfArm);
                hardware.armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.armMotor1.setPower(-0.25);
                hardware.armMotor2.setPower(-0.25);
            }
            telemetry.addLine(String.valueOf(hardware.leftSlider.getTargetPosition()));
            telemetry.addLine(String.valueOf(hardware.rightSlider.getTargetPosition()));
            telemetry.update();


            if (gamepad2.dpad_left) {
                hardware.clawWrist.setPosition(0.35);
            }
            if(gamepad2.dpad_right){
                hardware.clawWrist.setPosition(0.01);
            }
            if(gamepad2.dpad_down) {
                hardware.clawWrist.setPosition(0);
            }

            if(gamepad2.dpad_up) {
                hardware.clawBack.setPosition(0.1);
                hardware.clawFront.setPosition(0.1);
            }
//            arm1.runWithController(ly2, updateDelta);
//            arm2.runWithController(ry2, updateDelta);
//            updateDelta.reset();

//            int secondaryArmMotors = 15;
//            hardware.leftArm.setTargetPosition(secondaryArmMotors);
//            hardware.rightArm.setTargetPosition(secondaryArmMotors);
        }
    }
}
