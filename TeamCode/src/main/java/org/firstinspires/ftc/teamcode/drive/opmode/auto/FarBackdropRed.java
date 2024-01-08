package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.virtualdevices.HuskyBoy;

@Autonomous(group = "Auto Production", name = "Far Backdrop Red")
public class FarBackdropRed extends LinearOpMode {
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap);
//        NearBackdropBlue.scanPropPosition(new HuskyBoy(new Hardware(hardwareMap)), NanoClock.system(), this::isStopRequested, this.drive);
        waitForStart();
    }
}
