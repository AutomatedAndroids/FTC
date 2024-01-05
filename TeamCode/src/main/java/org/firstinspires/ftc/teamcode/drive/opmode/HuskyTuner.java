package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.virtualdevices.HuskyBoy;

import java.util.ArrayList;

@TeleOp(name = "Husky Tuner")
public class HuskyTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);
        Telemetry telemetries = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        HuskyBoy huskyBoy = new HuskyBoy(hardware);

        FtcDashboard.getInstance().addConfigVariable("Primaries", "Left Pixel Bound", new ValueProvider<Integer>() {
            @Override
            public Integer get() {
                return HuskyBoy.leftPixelBound;
            }

            @Override
            public void set(Integer value) {
                HuskyBoy.leftPixelBound = value;
            }
        });
        FtcDashboard.getInstance().addConfigVariable("Primaries", "Right Pixel bound", new ValueProvider<Integer>() {
            @Override
            public Integer get() {
                return HuskyBoy.rightPixelBound;
            }

            @Override
            public void set(Integer value) {
                HuskyBoy.rightPixelBound = value;
            }
        });

        waitForStart();
        HuskyLens.Block[] blocks;
        while (opModeIsActive()) {
            blocks = hardware.huskyLens.blocks();
            HuskyLens.Block block;
            if (blocks.length != 0); {
                block = blocks[0];
                if (block.x < HuskyBoy.leftPixelBound) {
                    telemetries.addLine("LEFT STRIP");
                } else if (block.x < HuskyBoy.rightPixelBound) {
                    telemetries.addLine("CENTER STRIP");
                } else {
                    telemetries.addLine("RIGHT STRIP");
                }
            }
        }
    }
}
