package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.virtualdevices.HuskyBoy;

@Autonomous(name = "Far Backdrop", preselectTeleOp="Mechanum TeleOp")
public class Auto2 extends LinearOpMode {
    private HuskyLens husky;

    // Constants
    private int pixelLeftBound = -100;
    private int pixelRightBound = 300;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        husky = drive.hardware.huskyLens;
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(20, -60, Math.toRadians(90)))
                .strafeRight(80)
                .build();
        waitForStart();
        drive.followTrajectory(trajectory);

    }
    public void inits() {
        HuskyLens.Block block = husky.blocks()[0];
        int classification = 0;
        if (block.x < pixelLeftBound) {
            classification = 0;
        } else if (block.x > pixelLeftBound && block.x < pixelRightBound) {
            classification = 1;
        } else {
            classification = 2;
        }
    }
}
