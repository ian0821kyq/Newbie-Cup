package frc.robot.subsystems;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class DriveMotorModule {
    private final SparkMax frontMotor;
    private final SparkMax backMotor;

    public DriveMotorModule(
        int frontId,
        int backId,
        boolean frontInverted,
        boolean backInverted
    ) {
        this.frontMotor = new SparkMax(frontId, MotorType.kBrushless);
        this.backMotor = new SparkMax(backId, MotorType.kBrushless);

        SparkMaxConfig frontConfig = new SparkMaxConfig();
        frontConfig
            .idleMode(IdleMode.kBrake)
            .inverted(frontInverted)
            .smartCurrentLimit(40);

        SparkMaxConfig backConfig = new SparkMaxConfig();
        backConfig
            .idleMode(IdleMode.kBrake)
            .inverted(backInverted)
            .smartCurrentLimit(40);

        this.frontMotor.configure(
            frontConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        this.backMotor.configure(
            backConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void set(double speed) {
        this.frontMotor.set(speed);
        this.backMotor.set(speed);
    }

    public void stop() {
        this.frontMotor.stopMotor();
        this.backMotor.stopMotor();
    }
}
