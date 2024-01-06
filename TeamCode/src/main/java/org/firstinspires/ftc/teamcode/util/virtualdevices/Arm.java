package org.firstinspires.ftc.teamcode.util.virtualdevices;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Arm {
    /*
     * This requries "volts" however since those variables are more difficult to read and influence, the values are extrapolated by a constant.
     * It represents the ratio from power to voltage, however is not used in any of the following functions.
     * */

    // Constants;
    public double extrapConst;
    /*
       The following definintions are the staic constants which are required for the feedfoward
       feedback tuning of the arm. Further info can be found here: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html
       Addtionally, the paper in which this tuning system is based can be found here:
       https://www.chiefdelphi.com/uploads/default/original/3X/f/7/f79d24101e6f1487e76099774e4ba60683e86cda.pdf
     */
    public double kS; // Voltage required to overcome static friction in the system. Unit: Volts;
    public double kV; // Voltage required to maintain a velocity, which is scaled as velocity increases. Unit: VOLTS;
    public double kA = 0; // Voltage required to produce a given acceleration in the shaft where V = kA * d**.
    public double kG; // Voltage required to overcome gravity, during the maximum position. See ArmEvaulator for tuning. Unit: VOLTS;

    // The follinwg are PID tuning values and are therefore feedback constants. They are more mathematical, and for tuning see the link in chat.google.com.
    public double kP;
    public double kI;
    public double kD;



    // The following constants are used for the motion profile.
    public double maxVel;
    public double maxAccel;

    // The following are motor specifc constants;
    private final int ticksPerRev = 537;
    public int degreeTolerance = 5;

    // Non Constants
    /* Note: Due to the fact that we only have one arm, the target angle can be made static.
    However, if funcitonality for multiple arms is desired, changes must be made. */

    public double currentTargetAngle;
    public double positionError;

    private TrapezoidProfile profile;

    // Other
    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
    private ArmFeedforward armFeedforward = new ArmFeedforward(kS, kG, kV, kA);
    Hardware hardware;
    private PIDCoefficients coefficients;
    private DcMotorControllerEx controller;
    protected double controlEffort;
    private Encoder encoder;

    private DcMotorEx armMotor1;
    private DcMotorEx armMotor2;

    private TrapezoidProfile.State start;
    private TrapezoidProfile.State end;


    private NanoClock nano;
    private double armBuild;
    private double execAcc;
    private double totalProfileTime;
    public double timeRemaining;
    public Arm(Hardware hardware, NanoClock nanoClock) {
        this.hardware = hardware;
        this.encoder = new Encoder(hardware.armMotor1);
        this.controlEffort = 0;
        this.armMotor1 = hardware.armMotor1;
        this.armMotor2 = hardware.armMotor2;
        updateConstants();
        this.nano = nanoClock;
    }
    /** This method only updates the methods which require the constants. */
    public void updateConstants() {
        this.coefficients = new PIDCoefficients(kP, kI, kD);
        this.armFeedforward = new ArmFeedforward(kS,kG,kV,kA);
        armMotor1.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        armMotor2.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }
    public void updateMotionProfile() {
        profile = new TrapezoidProfile(constraints,new TrapezoidProfile.State(), end );
        totalProfileTime = profile.totalTime();
    }
    public PIDCoefficients getCoefficients() {
        this.coefficients = new PIDCoefficients(kP, kI, kD);
        return this.coefficients;
    }
    /** Provided degrees on a scale of 0 - 360 degrees. */
    public void setArmTargetPosition(int targetAngle) {
        if (targetAngle >=0 && targetAngle <= 270) {
            currentTargetAngle = targetAngle;
            end = new TrapezoidProfile.State(currentTargetAngle,0);
            updateMotionProfile();
        }
    }
    /** Provided raidans for the arm.*/
    public void setArmTargetPosition(double targetRadian) {
        currentTargetAngle = Math.toDegrees(targetRadian);
    }
    public void resetArmPosition() {
        DcMotor.RunMode preexistingRunmode = hardware.armMotor1.getMode();
        hardware.armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor1.setMode(preexistingRunmode);
        hardware.armMotor2.setMode(preexistingRunmode);
    }
    public void execute() {
        execAcc=nano.seconds()-execAcc;
        controlEffort+=armFeedforward.calculate(Math.toRadians(profile.calculate(execAcc).position),profile.calculate(execAcc).velocity);
        positionError = this.currentTargetAngle - (double) encoder.getCurrentPosition() / 560*360;
        armMotor1.setPower(controlEffort);
        armMotor2.setPower(controlEffort);
    }
    public void tuningExecute() {
        execAcc=nano.seconds()-execAcc;
        controlEffort+=armFeedforward.calculate(Math.toRadians(profile.calculate(execAcc).position), profile.calculate(execAcc).velocity);
        positionError = this.currentTargetAngle - (double) encoder.getCurrentPosition() / 560*360;
        updateConstants();
        armMotor1.setPower(controlEffort);
        armMotor2.setPower(controlEffort);
    }
    public int getArmPosition() {
        return this.encoder.getCurrentPosition();
    }
}

