package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Near Backdrop", preselectTeleOp="Mechanum TeleOp")
public class Auto1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d pose = new Pose2d(20, -60, Math.toRadians(90));
        drive.setPoseEstimate(pose);
        Trajectory trajectory = drive.trajectoryBuilder(pose)
                .strafeRight(40)
                .build();
        waitForStart();
        drive.followTrajectory(trajectory);

    }
}
