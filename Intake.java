package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DeviceId;

public class Intake extends SubsystemBase {

    public enum LifterState {
        Up(Constants.Intake.LIFTER_UP),
        Down(Constants.Intake.LIFTER_DOWN),
        Zero(Constants.Intake.LIFTER_ZERO),
        Slide(Constants.Intake.LIFTER_SLIDE),
        OperateControl(0.0);

        // Units: rotation
        public final double angle;

        LifterState(double angle) {
            this.angle = angle;
        }
    }

    public enum RollerState {
        Off(Constants.Intake.ROLLER_OFF),
        Rest(Constants.Intake.ROLLER_REST),
        SlowIn(Constants.Intake.ROLLER_SLOW_IN),
        In(Constants.Intake.ROLLER_IN),
        OperateControl(0.0);

        public final double volts;

        RollerState(double volts) {
            this.volts = volts;
        }
    }

    private final TalonFX lifterMotor = new TalonFX(DeviceId.Intake.LIFTER);
    private final TalonFX rollerMotor = new TalonFX(DeviceId.Intake.ROLLER);
    private final MotionMagicVoltage lifterRequest = new MotionMagicVoltage(0.0);
    private final VoltageOut rollerRequest = new VoltageOut(0.0);
    private final Timer stateTime = new Timer();

    private final BooleanSupplier wantIntake;

    private LifterState lifterState = LifterState.Up;
    private RollerState rollerState = RollerState.Off;

    public Intake(BooleanSupplier wantIntake) {
        this.wantIntake = wantIntake;

        TalonFXConfiguration lifterConfig = new TalonFXConfiguration();
        lifterConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(Constants.Intake.LIFTER_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive);
        lifterConfig.Slot0.kP = Constants.Intake.LIFTER_KP;
        lifterConfig.Slot0.kD = Constants.Intake.LIFTER_KD;
        lifterConfig.Slot0.kS = Constants.Intake.LIFTER_KS;
        lifterConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.LIFTER_CRUISE_VELOCITY;
        lifterConfig.MotionMagic.MotionMagicAcceleration = Constants.Intake.LIFTER_ACCELERATION;
        lifterConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.LIFTER_CURRENT_LIMIT;
        lifterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        lifterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        lifterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Intake.LIFTER_MAX;
        lifterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        lifterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Intake.LIFTER_MIN;
        this.lifterMotor.getConfigurator().apply(lifterConfig);
        this.lifterMotor.setPosition(0.0); // 開機時滑軌需在收回位置

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(Constants.Intake.ROLLER_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive);
        rollerConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.ROLLER_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        this.rollerMotor.getConfigurator().apply(rollerConfig);

        this.stateTime.start();
    }

    public void setStates(LifterState lifterState, RollerState rollerState) {
        if (lifterState != this.lifterState)
            this.stateTime.restart();
        this.lifterState = lifterState;
        this.rollerState = rollerState;
    }

    public double getEffectiveLifterLength() {
        if (this.lifterState == LifterState.Slide)
            return this.lifterState.angle - Math.abs(
                Math.sin(Constants.Intake.SLIDE_FREQUENCY * this.stateTime.get())
                    * Constants.Intake.SLIDE_AMPLITUDE);
        else if (this.lifterState != LifterState.OperateControl)
            return this.lifterState.angle;
        else
            return LifterState.Down.angle;
    }

    public RollerState getEffectiveRollerState() {
        if (this.rollerState != RollerState.OperateControl)
            return this.rollerState;
        else if (this.wantIntake.getAsBoolean())
            return RollerState.In;
        else
            return RollerState.Off;
    }

    @Override
    public void periodic() {
        double lifterTarget = MathUtil.clamp(
            getEffectiveLifterLength(), Constants.Intake.LIFTER_MIN, Constants.Intake.LIFTER_MAX);
        RollerState roller = getEffectiveRollerState();

        this.lifterMotor.setControl(this.lifterRequest.withPosition(lifterTarget));
        this.rollerMotor.setControl(this.rollerRequest.withOutput(roller.volts));

        SmartDashboard.putNumber("Intake/LifterTarget", lifterTarget);
        SmartDashboard.putNumber("Intake/LifterPosition", this.lifterMotor.getPosition().getValueAsDouble());
        SmartDashboard.putString("Intake/RollerState", roller.name());
    }
}
