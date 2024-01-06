package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

public class MecanumDrive {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    public Hardware hardware;
    private NanoClock nano;

    public MecanumDrive(HardwareMap hardwareMap) {
        this.hardware = new Hardware(hardwareMap);
        this.frontLeft = hardware.frontLeft;
        this.backLeft= hardware.backLeft;
        this.backRight = hardware.backRight;
        this.frontRight = hardware.frontRight;
    }

    /** Strafes the robot write for a given amount of time at a power level from 0-1, denoted speed.*/
     public void strafeRight(double time, double speed) {
        double startTime = nano.seconds();
        while (nano.seconds()-startTime < time) {
            backRight.setPower(speed);
            backLeft.setPower(-speed);
            frontRight.setPower(-speed);
            frontLeft.setPower(speed);
        }
         backRight.setPower(0);
         backLeft.setPower(0);
         frontRight.setPower(0);
         frontLeft.setPower(0);
     }
    public void strafeLeft(double time, double speed) {
        double startTime = nano.seconds();
        while (nano.seconds()-startTime < time) {
            backRight.setPower(-speed);
            backLeft.setPower(speed);
            frontRight.setPower(speed);
            frontLeft.setPower(-speed);
        }
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
    }
    public void forward(double time, double speed) {
        double startTime = nano.seconds();
        while (nano.seconds()-startTime < time) {
            backRight.setPower(speed);
            backLeft.setPower(speed);
            frontRight.setPower(speed);
            frontLeft.setPower(speed);
        }
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
    }
    public void backward(double time, double speed) {
        double startTime = nano.seconds();
        while (nano.seconds()-startTime < time) {
            backRight.setPower(speed);
            backLeft.setPower(speed);
            frontRight.setPower(speed);
            frontLeft.setPower(speed);
        }
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
    }
    public void diagLeftUp(double time, double speed) {
        double startTime = nano.seconds();
        while (nano.seconds()-startTime < time) {
            backLeft.setPower(speed);
            frontRight.setPower(speed);
        }
        backLeft.setPower(0);
        frontRight.setPower(0);
    }
    public void diagRightUp(double time, double speed) {
        double startTime = nano.seconds();
        while (nano.seconds()-startTime < time) {
            frontLeft.setPower(speed);
            backRight.setPower(speed);
        }
        frontLeft.setPower(0);
        backRight.setPower(0);
    }
    public void diagRightDown(double time, double speed) {
        double startTime = nano.seconds();
        while (nano.seconds()-startTime < time) {
            backLeft.setPower(-speed);
            frontRight.setPower(-speed);
        }
        backLeft.setPower(0);
        frontRight.setPower(0);
    }
    public void diagLeftDown(double time, double speed) {
        double startTime = nano.seconds();
        while (nano.seconds()-startTime < time) {
            frontLeft.setPower(-speed);
            backRight.setPower(-speed);
        }
        frontLeft.setPower(0);
        backRight.setPower(0);
    }
    public void turn(double time, double speed) {
        double startTime = nano.seconds();
        while (nano.seconds()-startTime < time) {
            frontLeft.setPower(speed);
            backLeft.setPower(speed);
            frontRight.setPower(-speed);
            backRight.setPower(-speed);
        }
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
    }
}

