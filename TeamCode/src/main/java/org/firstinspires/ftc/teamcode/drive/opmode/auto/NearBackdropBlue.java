package org.firstinspires.ftc.teamcode.drive.opmode.auto;


import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.virtualdevices.HuskyBoy;

import java.util.concurrent.Callable;

@Autonomous(group = "Auto Production", name = "Near Backdrop Blue")
public class NearBackdropBlue extends LinearOpMode {
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(this.hardwareMap);
//        scanPropPosition(new HuskyBoy(new Hardware(hardwareMap)), NanoClock.system(), this::isStopRequested, this.drive);
//        drive.backward(1,0.2);
        waitForStart();
        drive.strafeLeft(1.5,0.7);
    }
    public static void scanPropPosition(HuskyBoy husky, NanoClock nano, Callable<Boolean> stopFunction, MecanumDrive driveProvided) {
        HuskyLens.Block[] blocks;
        double startTime = nano.seconds();
        HuskyBoy.PropLocation location;
        try {
            while (stopFunction.call() && nano.seconds() - startTime < 5) {
                blocks = husky.getHusky().blocks();
                HuskyLens.Block block;
                try {
                    if (blocks.length != 0) {
                        block = blocks[0];
                        if (block.x < HuskyBoy.rightPixelBound) {
                            location = HuskyBoy.PropLocation.CENTER;
                        } else {
                            location = HuskyBoy.PropLocation.RIGHT;
                        }
                    }
                } catch (ArrayIndexOutOfBoundsException ignore) {
                }
            }
        } catch (Exception ignore) {
        }
        location = HuskyBoy.PropLocation.LEFT;
        switch (location) {
            case LEFT:
                driveProvided.forward(1, 0.5);
                driveProvided.turn(1, -0.5);
                driveProvided.backward(1, 0.2);
                break;
            case CENTER:
                driveProvided.forward(1, 0.5);
                driveProvided.backward(1, 0.2);
                break;
            case RIGHT:
                driveProvided.forward(1, 0.5);
                driveProvided.turn(1, 0.5);
                driveProvided.backward(1, 0.2);
                break;
        }
    }
}
