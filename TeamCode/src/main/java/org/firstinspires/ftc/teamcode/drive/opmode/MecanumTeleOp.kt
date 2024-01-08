package org.firstinspires.ftc.teamcode.drive.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.Hardware

@TeleOp(name = "Mecanum TeleOp", group = "drive")
class MecanumTeleOp : LinearOpMode() {
    lateinit var telemetries: Telemetry
    var positionsSpeed = 0.2
    override fun runOpMode() {
        waitForStart()
        val positionOfArm = 0
        var sliderPosition = 0
        val started = true
        //        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        val hardware = Hardware(hardwareMap)
        this.telemetries = telemetry // new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        //        FtcDashboard.getInstance().addConfigVariable("Drive", "Arm To Position Speed",         new ValueProvider<Double>() {public Double get() {return positionsSpeed; } public void set(Double value) {positionsSpeed = value; }});

//        if (!(drive.getLocalizer() instanceof StandardTrackingWheelLocalizer)) {
//            RobotLog.setGlobalErrorMsg("StandardTrackingWheelLocalizer is not being set in the "
//                    + "drive class. Ensure that \"setLocalizer(new StandardTrackingWheelLocalizer"
//                    + "(hardwareMap));\" is called in SampleMecanumDrive.java");
//        }
        hardware.leftSlider.direction = DcMotorSimple.Direction.REVERSE
        waitForStart()
        val manualArmControl = false
        while (opModeIsActive()) {
            val y = -gamepad1.left_stick_x.toDouble()
            val x = gamepad1.left_stick_y.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()
            hardware.frontLeft.power = 0.85 * (y + x - rx)
            hardware.frontRight.power = 0.85 * (y - x - rx)
            hardware.backLeft.power = 0.85 * (y - x + rx)
            hardware.backRight.power = 0.85 * (y + x + rx)
            hardware.frontRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            hardware.frontLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            hardware.backLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            hardware.backRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            telemetry.addLine(" \n Current Voltage: " + hardware.batteryVoltageSens.voltage.toString())
            telemetries.addLine(" \n")
            telemetries.addLine(" Slider Motor Left; \n\t Current: " + hardware.leftSlider.getCurrent(CurrentUnit.AMPS).toString() + "\n\t Position: " + hardware.leftSlider.currentPosition.toString() + "\n\t Target: " + hardware.leftSlider.targetPosition.toString())
            telemetries.addLine(" Slider Motor Right; \n\t Current: " + hardware.rightSlider.getCurrent(CurrentUnit.AMPS).toString() + "\n\t Position: " + hardware.rightSlider.currentPosition.toString() + "\n\t Target: " + hardware.rightSlider.targetPosition.toString())
            if (gamepad2.dpad_down) {
                sliderPosition = 0
                telemetries.addLine("gp1 a")
                hardware.leftSlider.targetPosition = sliderPosition
                hardware.rightSlider.targetPosition = sliderPosition
                hardware.leftSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                hardware.rightSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                hardware.leftSlider.power = positionsSpeed
                hardware.rightSlider.power = positionsSpeed
            }
            if (gamepad2.dpad_left || gamepad2.dpad_right) {
                sliderPosition = 1040
                telemetries.addLine("gp1 x")
                hardware.leftSlider.targetPosition = sliderPosition
                hardware.rightSlider.targetPosition = sliderPosition
                hardware.leftSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                hardware.rightSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                hardware.leftSlider.power = positionsSpeed
                hardware.rightSlider.power = positionsSpeed
            }
            if (gamepad2.dpad_up) {
                sliderPosition = 1650
                telemetries.addLine("gp1 y")
                hardware.leftSlider.targetPosition = sliderPosition
                hardware.rightSlider.targetPosition = sliderPosition
                hardware.leftSlider.power = positionsSpeed
                hardware.rightSlider.power = positionsSpeed
                hardware.leftSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                hardware.rightSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
            }
            if (gamepad2.a) {
                hardware.droneServo.position = 0.75
            }
            if (gamepad2.b) {
                hardware.droneServo.position = 0.15
            }
            telemetries.update()
            // Claw Detached
//            if (gamepad2.a) {
//                hardware.clawWrist.setPosition(0.45);
//            }
//            if (gamepad2.b) {
//                hardware.clawWrist.setPosition(0);
//
//            }
//            if (gamepad2.x) {
//                hardware.clawBack.setPosition(0.15);
//                hardware.clawFront.setPosition(0.15);
//            }
//            if (gamepad2.y) {
//                hardware.clawFront.setPosition(0.05);
//                hardware.clawBack.setPosition(0.05);
//            }
//            telemetries.addData("Heading Est: ", drive.getPoseEstimate().getHeading());
//            telemetries.addData("X Est: ", drive.getPoseEstimate().getX());
//            telemetries.addData("Y Est: ", drive.getPoseEstimate().getY());

            // Arm Detached
//            if(gamepad2.dpad_left) {
//                positionOfArm = 0;
//                telemetries.addLine("DPAD_LEFT was pressed");
//                hardware.armMotor1.setTargetPosition(positionOfArm);
//                hardware.armMotor2.setTargetPosition(positionOfArm);
//                hardware.armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hardware.armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hardware.armMotor1.setPower(positionsSpeed);
//                hardware.armMotor2.setPower(positionsSpeed);
//
//            }
//            if(gamepad2.dpad_up) {
//                positionOfArm = 128;
//                telemetries.addLine("DPAD_UP was pressed");
//                hardware.armMotor1.setTargetPosition(positionOfArm);
//                hardware.armMotor2.setTargetPosition(positionOfArm);
//                hardware.armMotor1.setTargetPositionTolerance(0);
//                hardware.armMotor1.setTargetPositionTolerance(0);
//                hardware.armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hardware.armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hardware.armMotor1.setPower(positionsSpeed);
//                hardware.armMotor2.setPower(positionsSpeed);
//            }
//            if(gamepad2.dpad_right) {
//                positionOfArm = 260;
//
//
//                telemetries.addLine("DPAD_RIGHT was pressed");
//                hardware.armMotor1.setTargetPosition(positionOfArm);
//                hardware.armMotor2.setTargetPosition(positionOfArm);
//                 hardware.armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hardware.armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hardware.armMotor1.setPower(positionsSpeed);
//                hardware.armMotor2.setPower(positionsSpeed);
//              }


            //            if (gamepad2.left_bumper) {
//                if ( manualArmControl ) {
//                    manualArmControl = false;
//                    telemetries.addLine("LEFT_BUMPER pressed, manual mode inactive");
//                } else {
//                    manualArmControl = true;
//                    telemetries.addLine("LEFT_BUMPER pressed, manual mode active");
//                    hardware.armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    hardware.armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    hardware.armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                    hardware.armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                }
//            }
//            if ( gamepad2.right_bumper && manualArmControl) {
//                hardware.armMotor1.setPower(0);
//                hardware.armMotor2.setPower(0);
//            }
//            if (manualArmControl) {
//                hardware.armMotor1.setPower(gamepad2.right_stick_y);
//                hardware.armMotor2.setPower(gamepad2.right_stick_y);
//                telemetries.addData("Manual Arm Control", manualArmControl);
//                telemetries.addData("arm motors power input", hardware.armMotor1.getPower());
//            }
//            telemetries.addLine(" Arm Motor 1; \n\t Current: " + String.valueOf(hardware.armMotor1.getCurrent(CurrentUnit.AMPS)) + "\n\t Position: " + String.valueOf(hardware.armMotor1.getCurrentPosition()) + "\n\t Target: " + String.valueOf(hardware.armMotor1.getTargetPosition()));
//            telemetries.addLine(" Arm Motor 2; \n\t Current: " + String.valueOf(hardware.armMotor2.getCurrent(CurrentUnit.AMPS)) + "\n\t Position: " + String.valueOf(hardware.armMotor2.getCurrentPosition()) + "\n\t Target: " + String.valueOf(hardware.armMotor2.getTargetPosition()));
        }
    }
}