package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCmd extends Command {

    private final DriveSubsystem driveSubsystem;
    private final XboxController joystick;

    public DriveCmd(DriveSubsystem driveSubsystem, XboxController joystick) {
        this.driveSubsystem = driveSubsystem;
        this.joystick = joystick;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double forward = -MathUtil.applyDeadband(joystick.getLeftY(), Constants.Drive.DEADBAND)
            * Constants.Drive.MAX_SPEED;
        double turn = MathUtil.applyDeadband(joystick.getRightX(), Constants.Drive.DEADBAND)
            * Constants.Drive.MAX_TURN_SPEED;

        double leftSpeed = MathUtil.clamp(forward + turn, -1.0, 1.0);
        double rightSpeed = MathUtil.clamp(forward - turn, -1.0, 1.0);

        driveSubsystem.drive(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
