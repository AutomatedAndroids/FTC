package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.virtualdevices.HuskyBoy;

@Autonomous(group = "Auto Production", name = "Near Backdrop Red")
public class NearBackdropRed extends LinearOpMode {
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive=new MecanumDrive(hardwareMap);
//        NearBackdropBlue.scanPropPosition(new HuskyBoy(new Hardware(hardwareMap)), NanoClock.system(), this::isStopRequested, this.drive);
        waitForStart();
        drive.strafeRight(1.5,0.7);
    }
}
