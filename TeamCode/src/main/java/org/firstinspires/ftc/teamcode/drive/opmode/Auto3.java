package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "Far Backdrop 2", preselectTeleOp="Mechanum TeleOp")
public class Auto3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d pose = new Pose2d(-35, -60, Math.toRadians(90));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(pose);
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(pose)
                .splineTo(new Vector2d(60, 0), 90)
                .forward(60)
                .turn(-Math.toRadians(90))
                .turn(Math.toRadians(180))
                .back(95)
                .strafeLeft(10)
                .strafeRight(60)
                .forward(15)
                .build();
        waitForStart();
        drive.followTrajectorySequence(trajectory);

    }
}
