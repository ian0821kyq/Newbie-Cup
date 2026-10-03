package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DeviceId;

public class Hopper extends SubsystemBase {

    public enum HopperState {
        Off(0.0, 0.0),
        Convey(Constants.Hopper.ROLLER_CONVEY_VOLTS, Constants.Hopper.CENTER_CONVEY_VOLTS);

        public final double rollerVolts;
        public final double centerVolts;

        HopperState(double rollerVolts, double centerVolts) {
            this.rollerVolts = rollerVolts;
            this.centerVolts = centerVolts;
        }
    }

    private final TalonFX rollerMotor = new TalonFX(DeviceId.Hopper.ROLLER);
    private final TalonFX centerMotor = new TalonFX(DeviceId.Hopper.CENTER);
    private final VoltageOut rollerRequest = new VoltageOut(0.0);
    private final VoltageOut centerRequest = new VoltageOut(0.0);

    private HopperState hopperState = HopperState.Off;

    public Hopper() {
        configure(this.rollerMotor, Constants.Hopper.ROLLER_INVERTED);
        configure(this.centerMotor, Constants.Hopper.CENTER_INVERTED);
    }

    private static void configure(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(inverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive);
        config.CurrentLimits.StatorCurrentLimit = Constants.Hopper.CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        motor.getConfigurator().apply(config);
    }

    public void setState(HopperState hopperState) {
        this.hopperState = hopperState;
    }

    @Override
    public void periodic() {
        this.rollerMotor.setControl(this.rollerRequest.withOutput(this.hopperState.rollerVolts));
        this.centerMotor.setControl(this.centerRequest.withOutput(this.hopperState.centerVolts));

        SmartDashboard.putString("Hopper/State", this.hopperState.name());
    }
}
