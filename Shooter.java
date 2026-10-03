package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DeviceId;

public class Shooter extends SubsystemBase {

    public enum FlywheelState {
        Off(0.0),
        Rest(100.0),
        Auto(3150.0),
        Home(3500.0),
        SlowShoot(2000.0);

        // RPM
        public final double speed;

        FlywheelState(double speed) {
            this.speed = speed;
        }
    }

    public enum HoodState {
        Default(0.0),
        AutoAim(0.0),
        Home(25.0),
        Return(0.0);

        // Degree
        public final double angle;

        HoodState(double angle) {
            this.angle = angle;
        }
    }

    public enum FeederState {
        Off(0.0),
        Push(5.0),
        SlowPush(0.0);

        public final double volts;

        FeederState(double volts) {
            this.volts = volts;
        }
    }

    private final TalonFX flywheelLeader = new TalonFX(DeviceId.Shooter.FLYWHEEL_LEADER);
    private final TalonFX flywheelFollower = new TalonFX(DeviceId.Shooter.FLYWHEEL_FOLLOWER);
    private final TalonFX hoodMotor = new TalonFX(DeviceId.Shooter.HOOD);
    private final TalonFX feederMotor = new TalonFX(DeviceId.Shooter.FEEDER);

    private final VelocityVoltage flywheelRequest = new VelocityVoltage(0.0);
    private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0.0);
    private final VoltageOut feederRequest = new VoltageOut(0.0);

    // 距離(公尺) -> 仰角(度) / 轉速(RPM)
    private final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap flywheelRpmMap = new InterpolatingDoubleTreeMap();

    private final DoubleSupplier distanceToHub;

    private FlywheelState flywheelState = FlywheelState.Off;
    private HoodState hoodState = HoodState.Default;
    private FeederState feederState = FeederState.Off;

    public Shooter(DoubleSupplier distanceToHub) {
        this.distanceToHub = distanceToHub;

        // TODO: 換成實測資料
        hoodAngleMap.put(1.5, 5.0);
        hoodAngleMap.put(3.0, 12.0);
        hoodAngleMap.put(4.5, 20.0);
        hoodAngleMap.put(6.0, 25.0);
        flywheelRpmMap.put(1.5, 2800.0);
        flywheelRpmMap.put(3.0, 3150.0);
        flywheelRpmMap.put(4.5, 3400.0);
        flywheelRpmMap.put(6.0, 3700.0);

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.Slot0.kV = Constants.Shooter.FLYWHEEL_KV;
        flywheelConfig.Slot0.kP = Constants.Shooter.FLYWHEEL_KP;
        flywheelConfig.Slot0.kS = Constants.Shooter.FLYWHEEL_KS;
        this.flywheelLeader.getConfigurator().apply(flywheelConfig);
        this.flywheelFollower.getConfigurator().apply(flywheelConfig);
        this.flywheelFollower.setControl(new Follower(
            DeviceId.Shooter.FLYWHEEL_LEADER,
            Constants.Shooter.FLYWHEEL_FOLLOWER_OPPOSED
                ? MotorAlignmentValue.Opposed
                : MotorAlignmentValue.Aligned));

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.HOOD_GEAR_RATIO;
        hoodConfig.Slot0.kP = Constants.Shooter.HOOD_KP;
        hoodConfig.Slot0.kD = Constants.Shooter.HOOD_KD;
        hoodConfig.Slot0.kS = Constants.Shooter.HOOD_KS;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Shooter.HOOD_CRUISE_VELOCITY;
        hoodConfig.MotionMagic.MotionMagicAcceleration = Constants.Shooter.HOOD_ACCELERATION;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            Units.degreesToRotations(Constants.Shooter.HOOD_MAX_DEG);
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            Units.degreesToRotations(Constants.Shooter.HOOD_MIN_DEG);
        this.hoodMotor.getConfigurator().apply(hoodConfig);
        this.hoodMotor.setPosition(0.0); // 開機時仰角需在 0 度

        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        this.feederMotor.getConfigurator().apply(feederConfig);
    }

    public void setStates(FlywheelState flywheelState, HoodState hoodState, FeederState feederState) {
        this.flywheelState = flywheelState;
        this.hoodState = hoodState;
        this.feederState = feederState;
    }

    // 單位: 度
    public double getDesiredPosition() {
        double angle = this.hoodState != HoodState.AutoAim
            ? this.hoodState.angle
            : this.hoodAngleMap.get(this.distanceToHub.getAsDouble());
        return MathUtil.clamp(angle, Constants.Shooter.HOOD_MIN_DEG, Constants.Shooter.HOOD_MAX_DEG);
    }

    // 單位: RPM
    public double getDesiredVelocity() {
        if (this.flywheelState == FlywheelState.Off)
            return 0.0;
        if (RobotState.isAutonomous() || this.flywheelState == FlywheelState.Auto)
            return this.flywheelRpmMap.get(this.distanceToHub.getAsDouble());
        return this.flywheelState.speed;
    }

    public boolean flywheelAtSetpoint() {
        return Math.abs(this.flywheelLeader.getVelocity().getValueAsDouble() - this.getDesiredVelocity() / 60.0)
                < Constants.Shooter.FLYWHEEL_TOLERANCE;
    }

    public boolean hoodAtSetpoint() {
        return Math.abs(Units.rotationsToDegrees(this.hoodMotor.getPosition().getValueAsDouble())
                - this.getDesiredPosition()) < Constants.Shooter.HOOD_TOLERANCE;
    }

    public boolean flywheelAndHoodAtSetpoint() {
        return this.flywheelAtSetpoint() && this.hoodAtSetpoint();
    }

    @Override
    public void periodic() {
        double rpm = this.getDesiredVelocity();
        double angle = this.getDesiredPosition();

        if (rpm <= 0.0)
            this.flywheelLeader.stopMotor();
        else
            this.flywheelLeader.setControl(this.flywheelRequest.withVelocity(rpm / 60.0));
        this.hoodMotor.setControl(this.hoodRequest.withPosition(Units.degreesToRotations(angle)));
        this.feederMotor.setControl(this.feederRequest.withOutput(this.feederState.volts));

        SmartDashboard.putNumber("Shooter/DesiredRPM", rpm);
        SmartDashboard.putNumber("Shooter/DesiredHoodDeg", angle);
        SmartDashboard.putBoolean("Shooter/AtSetpoint", this.flywheelAndHoodAtSetpoint());
    }
}
