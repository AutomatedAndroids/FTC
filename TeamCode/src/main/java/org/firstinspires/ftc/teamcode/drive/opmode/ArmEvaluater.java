package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Disabled
@Autonomous(name = "Arm Evaluate")
public class ArmEvaluater extends LinearOpMode {
    Hardware hardware;
    DcMotorEx leftSlider;
    DcMotorEx rightSlider;
    Encoder encoder;

    public static int targetHold = 260;
    public static int milliTargetHoldTestTime = 2000;
    public static int milliNormCycleTime = 1000;
    public static double powerIncrement = 0.025;
    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(hardwareMap);
        leftSlider = hardware.leftSlider;
        rightSlider = hardware.rightSlider;
        Telemetry telemetries = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        double startTime;
        double accTime;
        double accPower=0;

        encoder = new Encoder(leftSlider);

        // Allows for editing during build;
        FtcDashboard.getInstance().addConfigVariable("Config", "Target Hold Test Time (ms)", new ValueProvider<Integer>()  {@Override public Integer get() {return milliTargetHoldTestTime;}@Override public void set(Integer value) {milliTargetHoldTestTime = value;}});
        FtcDashboard.getInstance().addConfigVariable("Config", "Target Hold Position", new ValueProvider<Integer>()  {@Override public Integer get() {return targetHold;}@Override public void set(Integer value) {targetHold = value;}});
        FtcDashboard.getInstance().addConfigVariable("Config", "Normal Cycle Length", new ValueProvider<Integer>()  {@Override public Integer get() {return milliNormCycleTime;}@Override public void set(Integer value) {milliNormCycleTime = value;}});
        FtcDashboard.getInstance().addConfigVariable("Config", "Power Increment", new ValueProvider<Double>()  {@Override public Double get() {return powerIncrement;}@Override public void set(Double value) {powerIncrement = value;}});


        NanoClock nano = NanoClock.system();
        waitForStart();

        startTime = nano.seconds();
        accTime = startTime;
        telemetries.addLine("STARTED");

        while(opModeIsActive()) {
            accTime = nano.seconds();
            if ( accTime - startTime >= milliNormCycleTime/1000 && accPower != 1) {
                accPower+=powerIncrement;
                accTime = 0;
            }
            if (encoder.getCurrentPosition() == targetHold) {
                telemetries.addLine("POSITION FOUND, TESTING FOR CHANGE");
                Thread.sleep(milliTargetHoldTestTime);
                if (encoder.getCurrentPosition() != targetHold) {
                    telemetries.clearAll();
                    telemetries.addLine("POSITION LOST");
                }
                else {
                    telemetries.clearAll();
                    telemetries.addLine("POSITION LOCKED?");
                    break;
                }
            }
            telemetries.addData("Acc Timer", accTime);
            telemetries.addData("Power", accPower);
            telemetries.addData("Elapsed Time", nano.seconds() - startTime);
        }
        while(opModeIsActive()) {
            telemetries.addLine("Power Used:"+accPower);
            telemetries.addLine("Time at Event" + accTime+"\n");

            telemetries.addLine("Waiting for end.");

        }
    }
}
