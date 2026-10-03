package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DeviceId;

public class Conveyer extends SubsystemBase {
    private final TalonFX motor;

    public Conveyer() {
        // 創建 motor 物件
        this.motor = new TalonFX(DeviceId.Conveyer.CONVEYER);

        // Talon 設定
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake) // Brake 停止後鎖住馬達, Coast 停止後保持慣性
            .withInverted(InvertedValue.Clockwise_Positive); // 是否反轉

        // Apply 到 motor
        this.motor.getConfigurator().apply(config);
    }

    public void move(double speed) {
        SmartDashboard.putNumber("Talon Speed", speed * Constants.Conveyer.MAX_DRIVE_SPEED);
        this.motor.set(speed * Constants.Conveyer.MAX_DRIVE_SPEED);
    }

    public void stop() {
        this.motor.stopMotor();
    }
}