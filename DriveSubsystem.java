package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.DeviceId.DriveMotor;

public class DriveSubsystem extends SubsystemBase {
    private final DriveMotorModule leftModule;
    private final DriveMotorModule rightModule;

    public DriveSubsystem() {
        this.leftModule = new DriveMotorModule(
            DriveMotor.FRONT_LEFT,
            DriveMotor.BACK_LEFT,
            Drive.FRONT_LEFT_INVERTED,
            Drive.BACK_LEFT_INVERTED
        );
        this.rightModule = new DriveMotorModule(
            DriveMotor.FRONT_RIGHT,
            DriveMotor.BACK_RIGHT,
            Drive.FRONT_RIGHT_INVERTED,
            Drive.BACK_RIGHT_INVERTED
        );
    }

    public void drive(double leftSpeed, double rightSpeed) {
        this.leftModule.set(leftSpeed);
        this.rightModule.set(rightSpeed);
    }

    public void stop() {
        this.leftModule.stop();
        this.rightModule.stop();
    }
}
