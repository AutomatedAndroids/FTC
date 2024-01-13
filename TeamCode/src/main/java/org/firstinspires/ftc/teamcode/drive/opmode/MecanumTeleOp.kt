package org.firstinspires.ftc.teamcode.drive.opmode

import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.Hardware

@TeleOp(name = "Mecanum TeleOp", group = "drive")
class MecanumTeleOp : LinearOpMode() {
    lateinit var telemetries: Telemetry
    var positionsSpeed = 0.4
    override fun runOpMode() {
        waitForStart()
        var positionOfArm = 0
        var sliderPosition = 0
        var clawWristPosition = 0.0

        val hardware = Hardware(hardwareMap)
        this.telemetries = telemetry // new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        //        FtcDashboard.getInstance().addConfigVariable("Drive", "Arm To Position Speed",         new ValueProvider<Double>() {public Double get() {return positionsSpeed; } public void set(Double value) {positionsSpeed = value; }});

//        if (!(drive.getLocalizer() instanceof StandardTrackingWheelLocalizer)) {
//            RobotLog.setGlobalErrorMsg("StandardTrackingWheelLocalizer is not being set in the "
//                    + "drive class. Ensure that \"setLocalizer(new StandardTrackingWheelLocalizer"
//                    + "(hardwareMap));\" is called in SampleMecanumDrive.java");
//        }
        hardware.leftSlider.direction = DcMotorSimple.Direction.REVERSE

//        hardware.clawFront.direction = Servo.Direction.FORWARD

//        hardware.armMotor1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

//        hardware.armMotor1.targetPosition = positionOfArm;

//        hardware.armMotor1.mode = DcMotor.RunMode.RUN_TO_POSITION
//        hardware.armMotor1.power = positionsSpeed;

        val droneLauncherStartPosition = hardware.droneServo.position

        waitForStart()
        var manualArmControl = false
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
            if (gamepad1.dpad_down) {
                sliderPosition = 0
                telemetries.addLine("gp1 a")
                hardware.leftSlider.targetPosition = sliderPosition
                hardware.rightSlider.targetPosition = sliderPosition
                hardware.leftSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                hardware.rightSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                hardware.leftSlider.power = positionsSpeed
                hardware.rightSlider.power = positionsSpeed
            }
            if (gamepad1.dpad_left || gamepad1.dpad_right) {
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

            // Drone Servo
            if (gamepad1.a || gamepad2.left_bumper) {
                hardware.droneServo.position = droneLauncherStartPosition + 0.3
            }
            if (gamepad1.b || gamepad2.right_bumper) {
                hardware.droneServo.position = droneLauncherStartPosition
            }

            // Claw
            if (gamepad2.a) {
                hardware.clawWrist.setPosition(0.55);
            }
            if (gamepad2.b) {
                hardware.clawWrist.setPosition(0.90);
            }
            if (gamepad2.x) {
                hardware.clawBack.position = 0.15;
                hardware.clawFront.position = 0.15;
            }
            if (gamepad2.y) {
                hardware.clawFront.position = 0.05;
                hardware.clawBack.position = 0.05;
            }

            // Arm
            if(gamepad2.dpad_left) {
                positionOfArm = 1000;
                telemetries.addLine("DPAD_LEFT was pressed");
                hardware.armMotor1.setTargetPosition(positionOfArm);
            }
            if(gamepad2.dpad_up) {
                positionOfArm = 100;
                telemetries.addLine("DPAD_UP was pressed");
                hardware.armMotor1.setTargetPosition(positionOfArm);
            }
            if(gamepad2.dpad_right) {
                positionOfArm = 260;
                telemetries.addLine("DPAD_RIGHT was pressed");
                hardware.armMotor1.setTargetPosition(positionOfArm);
              }
            if (gamepad2.dpad_down) {
                positionOfArm = 1150;
                telemetries.addLine("DPAD_DOWN");
                hardware.armMotor1.targetPosition = positionOfArm
            }
//            clawWristPosition+= gamepad2.right_stick_y/100.0
//            clawWristPosition = Range.clip(clawWristPosition, 0.0, 1.0)
//            telemetries.addLine("Claw Wrist" + clawWristPosition)
//            telemetries.addLine("Claw Wrist Position" + hardware.clawWrist.position)
//            telemetries.addLine("\nArm Position" + positionOfArm)
//            telemetries.addLine("Arm Position Real: " + hardware.armMotor1.currentPosition)
//            positionOfArm+=(gamepad2.left_stick_y*5).toInt()
//            hardware.armMotor1.targetPosition = positionOfArm
//            hardware.clawWrist.position = clawWristPosition
            telemetries.addLine("\n\n Drone Servo \t Target: " + hardware.droneServo.position)
            telemetries.update()
        }
    }
}