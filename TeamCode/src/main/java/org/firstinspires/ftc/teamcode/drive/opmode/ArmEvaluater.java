package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.virtualdevices.Arm;

import java.util.ArrayList;

@TeleOp(group = "Tuner", name = "Arm Evaluate")
public class ArmEvaluater extends LinearOpMode {
    Hardware hardware;
    DcMotorEx leftArm;
    DcMotorEx rightArm;
    Encoder encoder;

    public static int targetHold = 260;
    public static int milliTargetHoldTestTime = 2000;
    public static int milliNormCycleTime = 1000;
    public static double powerIncrement = 0.025;


    // Not to be touched.
    double setPoint = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(hardwareMap);
        leftArm = hardware.armMotor1;
        rightArm = hardware.armMotor2;
        Telemetry telemetries = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        double startTime;
        double accTime;
        double accPower=0;


        Arm arm = new Arm(hardware,NanoClock.system());

        encoder = new Encoder(leftArm);

        Boolean started = true;

        // Allows for editing during build;

        ArrayList<String> tuned = new ArrayList<String>();

        FtcDashboard.getInstance().addConfigVariable("Config", "kS", new ValueProvider<Double>() {
            @Override
            public Double get() {
                return arm.kS;
            }

            @Override
            public void set(Double value) {
                arm.kS = value;
            }
        });
        FtcDashboard.getInstance().addConfigVariable("Config", "kA", new ValueProvider<Double>() {
            @Override
            public Double get() {
                return arm.kA;
            }

            @Override
            public void set(Double value) {
                arm.kA = value;
            }
        });
        FtcDashboard.getInstance().addConfigVariable("Config", "Max Velocity", new ValueProvider<Double>() {
            @Override
            public Double get() {
                return arm.maxVel;
            }

            @Override
            public void set(Double value) {
                arm.maxVel = value;
            }
        });
        FtcDashboard.getInstance().addConfigVariable("Config", "Max Acceleration", new ValueProvider<Double>() {
            @Override
            public Double get() {
                return arm.maxAccel;
            }

            @Override
            public void set(Double value) {
                arm.maxAccel = value;
            }
        });
        FtcDashboard.getInstance().addConfigVariable("Config", "Degree Tolerance", new ValueProvider<Integer>() {
            @Override
            public Integer get() {
                return arm.degreeTolerance;
            }

            @Override
            public void set(Integer value) {
                arm.degreeTolerance = value;
            }
        });


        waitForStart();
        while (opModeIsActive()) {
            telemetries.addLine("Arm Tuner. Please one of the following keybinds to begin a tuning session.");
            telemetries.addLine(
                    "X: kG Tuner\n" +
                            "B: kV, kG Tuner\n" +
                            "Y: PID Tuner\n"

            );
            if (gamepad1.x) { // kG tuner
                FtcDashboard.getInstance().addConfigVariable("Tuned", "Target Hold Test Time (ms)", new ValueProvider<Integer>()  {@Override public Integer get() {return milliTargetHoldTestTime;}@Override public void set(Integer value) {milliTargetHoldTestTime = value;}});
                FtcDashboard.getInstance().addConfigVariable("Tuned", "Target Hold Position", new ValueProvider<Integer>()  {@Override public Integer get() {return targetHold;}@Override public void set(Integer value) {targetHold = value;}});
                FtcDashboard.getInstance().addConfigVariable("Tuned", "Normal Cycle Length", new ValueProvider<Integer>()  {@Override public Integer get() {return milliNormCycleTime;}@Override public void set(Integer value) {milliNormCycleTime = value;}});
                FtcDashboard.getInstance().addConfigVariable("Tuned", "Power Increment", new ValueProvider<Double>()  {@Override public Double get() {return powerIncrement;}@Override public void set(Double value) {powerIncrement = value;}});


                NanoClock nano = NanoClock.system();

                startTime = nano.seconds();
                accTime = startTime;
                telemetries.addLine("STARTED SIMPLE KG TUNING");

                while(started && opModeIsActive()) {
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
                            started = false;
                        }
                    }
                    telemetries.addData("Acc Timer", accTime);
                    telemetries.addData("Power", accPower);
                    telemetries.addData("Elapsed Time", nano.seconds() - startTime);
                    telemetries.update();
                }
                while(opModeIsActive()) {
                    telemetries.addLine("Power Used:"+accPower);
                    telemetries.addLine("Time at Event" + accTime+"\n");
                    if (gamepad1.start && gamepad1.back) {
                        FtcDashboard.getInstance().removeConfigVariable("Tuned", "Target Hold Test");
                        FtcDashboard.getInstance().removeConfigVariable("Tuned", "Target Hold Position");
                        FtcDashboard.getInstance().removeConfigVariable("Tuned", "Normal Cycle Length");
                        FtcDashboard.getInstance().removeConfigVariable("Tuned", "Power");
                        tuned.add(0, "Simple kG_" + accPower);
                        tuned.add("Target Hold Test_" + milliTargetHoldTestTime);
                        tuned.add("Target Hold Position_" + targetHold);
                        tuned.add("Normal Cycle Length_" + milliNormCycleTime);
                        tuned.add("Power Increment_" + powerIncrement);
                        break;
                    }
                    telemetries.addLine("Hold start and back to end the current tuning session.");

                    telemetries.update();

                }
            } // kG Simple tuner
            if (gamepad1.b) {
                wait(500);
                telemetries.addLine("Please align the robot arm to position 0.\n Press A when the action is complete");
                while (gamepad1.a != true && opModeIsActive()) {
                    telemetries.addLine("Please align the robot arm to position 0.\n Press A when the action is complete");
                    updateTelemetry(telemetries);
                }
                arm.resetArmPosition();
                telemetries.addLine("Encoder Reading reset to 0;");
                telemetries.addLine("Activating kV, kG tuner");
                telemetries.update();
                wait(2500);
                while (gamepad1.a!=true && opModeIsActive()) {
                    telemetries.addLine(
                            "Note this tuning mode requires the driver to manually and repeatedly move the arm" +
                                    "using the setpoint system in ftc dashboard. Continous alteration of the position is " +
                                    "(technically) supported, however it is not functional with gamepads and is not programmed." +
                                    "\n\n Please press A to acknowledge and advance."
                    );
                }

                FtcDashboard.getInstance().addConfigVariable("Tuning", "kV", new ValueProvider<Double>() {
                    @Override
                    public Double get() {
                        return arm.kV;
                    }

                    @Override
                    public void set(Double value) {
                        arm.kV = value;
                    }
                });
                FtcDashboard.getInstance().addConfigVariable("Tuning", "kG", new ValueProvider<Double>() {
                    @Override
                    public Double get() {
                        return arm.kG;
                    }

                    @Override
                    public void set(Double value) {
                        arm.kG = value;
                    }
                });

                /* Note: The following config variable is used to drive the motor where the value is an integer degree.*/
                FtcDashboard.getInstance().addConfigVariable("Drive", "Set Point", new ValueProvider<Double>() {
                    @Override
                    public Double get() {
                        return setPoint;
                    }

                    @Override
                    public void set(Double value) {
                        setPoint = value;
                    }

                });

                while (opModeIsActive()) {
                    telemetries.addLine("Arm Encoder Reading"+arm.getArmPosition());
                    arm.setArmTargetPosition(setPoint);
                }

            } // kV, kG Tuner
            if (gamepad1.y) {
                wait(500);
                telemetries.addLine("Please align the robot arm to position 0.\n Press A when the action is complete");
                while (gamepad1.a != true && opModeIsActive()) {
                    telemetries.addLine("Please align the robot arm to position 0.\n Press A when the action is complete");
                    updateTelemetry(telemetries);
                }
                arm.resetArmPosition();
                telemetries.addLine("Encoder Reading reset to 0;");
                telemetries.addLine("Activating PID tuner");
                telemetries.update();
                wait(2500);
                while (gamepad1.a!=true && opModeIsActive()) {
                    telemetries.addLine(
                            "Note this tuning mode requires the driver to manually and repeatedly move the arm" +
                                    "using the setpoint system in ftc dashboard. Continous alteration of the position is " +
                                    "(technically) supported, however it is not functional with gamepads and is not programmed." +
                                    "\n\n Please press A to acknowledge and advance."
                    );
                }

                FtcDashboard.getInstance().addConfigVariable("Tuning", "kP", new ValueProvider<Double>() {
                    @Override
                    public Double get() {
                        return arm.kP;
                    }

                    @Override
                    public void set(Double value) {
                        arm.kP = value;
                    }
                });
                FtcDashboard.getInstance().addConfigVariable("Tuning", "kI", new ValueProvider<Double>() {
                    @Override
                    public Double get() {
                        return arm.kI;
                    }

                    @Override
                    public void set(Double value) {
                        arm.kI = value;
                    }
                });
                FtcDashboard.getInstance().addConfigVariable("Tuning", "kD", new ValueProvider<Double>() {
                    @Override
                    public Double get() {
                        return arm.kD;
                    }

                    @Override
                    public void set(Double value) {
                        arm.kD = value;
                    }
                });

                /* Note: The following config variable is used to drive the motor where the value is an integer degree.*/
                FtcDashboard.getInstance().addConfigVariable("Drive", "Set Point", new ValueProvider<Double>() {
                    @Override
                    public Double get() {
                        return setPoint;
                    }

                    @Override
                    public void set(Double value) {
                        setPoint = value;
                    }

                });

                while (opModeIsActive()) {
                    telemetries.addLine("Arm Encoder Reading"+arm.getArmPosition());
                    arm.setArmTargetPosition(setPoint);
                }

            } // PID tuner; kP, kI, kD Tuner
            if (gamepad1.a) {
                wait(500);
                telemetries.addLine("Please align the robot arm to position 0.\n Press A when the action is complete");
                while (gamepad1.a != true && opModeIsActive()) {
                    telemetries.addLine("Please align the robot arm to position 0.\n Press A when the action is complete");
                    updateTelemetry(telemetries);
                }
                arm.resetArmPosition();
                telemetries.addLine("Encoder Reading reset to 0;");
                telemetries.addLine("Activating kV, kG tuner");
                telemetries.update();
                wait(2500);
                while (gamepad1.a!=true && opModeIsActive()) {
                    telemetries.addLine(
                            "Note this tuning mode requires the driver to manually and repeatedly move the arm" +
                                    "using the setpoint system in ftc dashboard. Continous alteration of the position is " +
                                    "(technically) supported, however it is not functional with gamepads and is not programmed." +
                                    "\n\n Please press A to acknowledge and advance."
                    );
                }

                FtcDashboard.getInstance().addConfigVariable("Tuning", "kV", new ValueProvider<Double>() {
                    @Override
                    public Double get() {
                        return arm.kV;
                    }

                    @Override
                    public void set(Double value) {
                        arm.kV = value;
                    }
                });
                FtcDashboard.getInstance().addConfigVariable("Tuning", "kG", new ValueProvider<Double>() {
                    @Override
                    public Double get() {
                        return arm.kG;
                    }

                    @Override
                    public void set(Double value) {
                        arm.kG = value;
                    }
                });
                FtcDashboard.getInstance().addConfigVariable("Tuning", "kP", new ValueProvider<Double>() {
                    @Override
                    public Double get() {
                        return arm.kP;
                    }

                    @Override
                    public void set(Double value) {
                        arm.kP = value;
                    }
                });
                FtcDashboard.getInstance().addConfigVariable("Tuning", "kI", new ValueProvider<Double>() {
                    @Override
                    public Double get() {
                        return arm.kI;
                    }

                    @Override
                    public void set(Double value) {
                        arm.kI = value;
                    }
                });
                FtcDashboard.getInstance().addConfigVariable("Tuning", "kD", new ValueProvider<Double>() {
                    @Override
                    public Double get() {
                        return arm.kD;
                    }

                    @Override
                    public void set(Double value) {
                        arm.kD = value;
                    }
                });

                /* Note: The following config variable is used to drive the motor where the value is an integer degree.*/
                FtcDashboard.getInstance().addConfigVariable("Drive", "Set Point", new ValueProvider<Double>() {
                    @Override
                    public Double get() {
                        return setPoint;
                    }

                    @Override
                    public void set(Double value) {
                        setPoint = value;
                    }

                });

                while (opModeIsActive()) {
                    telemetries.addLine("Arm Encoder Reading"+arm.getArmPosition());
                    arm.setArmTargetPosition(setPoint);
                }

            } // Total Tuner

            // tuned variables display
            telemetries.addLine("\n Tuned Variables:");
            for (String elm : tuned) {
                telemetries.addLine(elm.replace("_", ": "));
            }

            telemetries.update();
        }
    }
}