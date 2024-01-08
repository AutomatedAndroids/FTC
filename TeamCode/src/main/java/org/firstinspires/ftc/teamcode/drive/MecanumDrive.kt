package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.util.drive.MotorPowerCarrier
import org.firstinspires.ftc.teamcode.util.drive.NormalizedDrivePose2d

/** Note, this file is in kotlin due to its ease of use and simplicity in conversion between existing kotlin files. */
class MecanumDrive(hardwareMap: HardwareMap?) : SampleMecanumDrive(hardwareMap) {
    private val frontLeft: DcMotorEx
    private val frontRight: DcMotorEx
    private val backLeft: DcMotorEx
    private val backRight: DcMotorEx
    private val nano: NanoClock

    init {
        hardware = Hardware(hardwareMap)
        frontLeft = hardware.frontLeft
        backLeft = hardware.backLeft
        backRight = hardware.backRight
        frontRight = hardware.frontRight
        nano = NanoClock.system()
    }


    override fun setDriveSignal(driveSignal: DriveSignal) {
        // TODO: finish;
    }
    /** These numbers are adjusted for motor discrepancy and should not be adjusted by the user of the function.  */
    fun driveLikeController(time: Double, speed: Double, y: Double, x: Double, rotation: Double) {
        val startTime = nano.seconds()
        while (nano.seconds() - startTime < time) {
            hardware.frontLeft.power = speed * (-x + y - rotation)
            hardware.backLeft.power = speed * (-x - y - rotation)
            hardware.frontRight.power = speed * (-x - y + rotation)
            hardware.backRight.power = speed * (-x + y + rotation)
        }
        frontLeft.power = 0.0
        backRight.power = 0.0
        frontRight.power = 0.0
        backLeft.power = 0.0
    }
    /** Converts the vector  */
    fun mecanumWheelAdjuster(vector: NormalizedDrivePose2d): MotorPowerCarrier {
        return MotorPowerCarrier(
            -vector.x + vector.y - vector.heading,
            -vector.x - vector.y - vector.heading,
            -vector.x -vector.y + vector.heading,
            -vector.x
        )
    }

    /** Alters drive powers to account for physical and digital discrepancies*/
    fun translateWheelPower(frontLeft: Double, frontRight: Double, backLeft: Double, backRight: Double) {
        motorsToPoseUnadjusted(frontLeft, backLeft, frontRight, backRight)
    }

    /** Strafes the robot right for a given amount of time at a power level from 0-1, denoted speed. */

    fun strafeRight(time: Double, speed: Double) {
        driveLikeController(time, speed, 0.0, 1.0, 0.0)
    }
    /** Creates a normalized controller like pose from the motor powers. The equations are adjusted for strange power inputs however an alternate [motorsToPoseUnadjusted] is possible for an unadjusted result.*/
    fun motorsToPoseAdjusted(leftFrontPower: Double, leftRearPower: Double, rightRearPower: Double, rightFrontPower: Double): NormalizedDrivePose2d {
        val x = (leftFrontPower+leftRearPower+rightRearPower+rightFrontPower)/-4
        return NormalizedDrivePose2d(x,(rightFrontPower+rightRearPower)/-2+x,(leftRearPower+rightFrontPower)/-2 + x)
    }
    fun motorsToPoseUnadjusted(leftFrontPower: Double, leftRearPower: Double, rightRearPower: Double, rightFrontPower: Double): NormalizedDrivePose2d {
        val y = (leftFrontPower + leftRearPower + rightRearPower + rightFrontPower) / -4;
        return NormalizedDrivePose2d((leftRearPower + rightFrontPower)/-2-y,y,0.0)
    }

    fun strafeLeft(time: Double, speed: Double) {
        driveLikeController(time, speed, 0.0, -0.8, 0.0)
    }

    fun forward(time: Double, speed: Double) {
        driveLikeController(time, speed, 1.0, 0.0, 0.0)
    }

    fun backward(time: Double, speed: Double) {
        driveLikeController(time, speed, -1.0, 0.0, 0.0)
    }

    fun diagLeftUp(time: Double, speed: Double) {
        driveLikeController(time, speed, 1.0, -1.0, 0.0)
    }

    fun diagRightUp(time: Double, speed: Double) {
        driveLikeController(time, speed, 1.0, 1.0, 0.0)
    }

    fun diagRightDown(time: Double, speed: Double) {
        driveLikeController(time, speed, -1.0, 1.0, 0.0)
    }

    fun diagLeftDown(time: Double, speed: Double) {
        driveLikeController(time, speed, -1.0, -1.0, 0.0)
    }

    fun turn(time: Double, speed: Double) {
        driveLikeController(time, speed, 0.0, 0.0, 1.0)
    }
}
