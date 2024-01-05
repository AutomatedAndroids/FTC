package org.firstinspires.ftc.teamcode.util.virtualdevices;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.Encoder;

import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class Arm{

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

    public static double currentTargetAngle;

    // Other
    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
    private ArmFeedforward armFeedforward = new ArmFeedforward(kS, kG, kV, kA);
    Hardware hardware;
    private final Command setArmNonSequential = new SetArmPosition(new ArmSubsystem(this.hardware), this);
    private PIDCoefficients coefficients;

    public Arm(Hardware hardware) {
        this.hardware = hardware;
    }

    public PIDCoefficients getCoefficients() {
        this.coefficients = new PIDCoefficients(kP, kI, kD);
        return this.coefficients;
    }

    public ArmFeedforward getArmFeedforward() {

        return armFeedforward;
    }
    public void setArmPosition(double targetAngle) {
        currentTargetAngle = targetAngle;
        CommandScheduler.getInstance().schedule(setArmNonSequential);
    }
    public void resetArmPosition() {
        DcMotor.RunMode preexistingRunmode = hardware.armMotor1.getMode();
        hardware.armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armMotor1.setMode(preexistingRunmode);
        hardware.armMotor2.setMode(preexistingRunmode);    }
    public int getArmPosition() {
        return new Encoder(hardware.armMotor1).getCurrentPosition();
    }
}

class ArmSubsystem extends SubsystemBase {
    private final DcMotorEx armMotor1;
    private final DcMotorEx armMotor2;
    public ArmSubsystem(Hardware hardware) {
        armMotor1 = hardware.armMotor1;
        armMotor2 = hardware.armMotor2;
    }

    public DcMotorEx getArmMotor1() {
        return armMotor1;
    }

    public DcMotorEx getArmMotor2() {
        return armMotor2;
    }
    public void initialize() {

    }
}

class SetArmPosition extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ArmSubsystem armSub;
    private final DcMotorEx armMotor1;
    private final DcMotorEx armMotor2;
    private Encoder encoder;
    private int motorPort;
    DcMotorControllerEx controller;

    private Arm arm;

    protected double controlEffort;
    protected double positionError;

    SetArmPosition(ArmSubsystem armSubSystem, Arm arm) {
        this.armSub = armSubSystem;
        this.armMotor1 = armSub.getArmMotor1();
        this.armMotor2 = armSub.getArmMotor2();
        this.arm = arm;
        addRequirements(armSubSystem);
        this.motorPort = armMotor1.getPortNumber();
    }
    @SuppressWarnings({})
    @Override
    public void initialize() {
        armSub.initialize();
        encoder = new Encoder(armMotor1);
        armMotor1.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, arm.getCoefficients());
        controlEffort = 0;
    }

    @Override
    public void execute() {
        controlEffort+=arm.getArmFeedforward().calculate((Math.toRadians(encoder.getCurrentPosition())/560*360), encoder.getCorrectedVelocity());
        positionError = Arm.currentTargetAngle - (double) encoder.getCurrentPosition() /560*360;
        controller.setPIDCoefficients(motorPort,DcMotor.RunMode.RUN_USING_ENCODER, arm.getCoefficients());
        controller.setMotorPower(motorPort, controlEffort);
    }

    @Override
    public boolean isFinished() {
        return positionError<arm.degreeTolerance;
    }
}