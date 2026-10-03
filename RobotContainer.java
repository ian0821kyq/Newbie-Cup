package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.FeederState;
import frc.robot.subsystems.Shooter.FlywheelState;
import frc.robot.subsystems.Shooter.HoodState;
import frc.robot.subsystems.Intake.LifterState;
import frc.robot.subsystems.Intake.RollerState;

public class RobotContainer {

    private final XboxController driverController =
        new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final Intake intake = new Intake(this.driverController::getAButton);

    private final Hopper hopper = new Hopper();

    private final Shooter shooter = new Shooter(() -> Constants.Shooter.DEFAULT_DISTANCE);

    public RobotContainer() {
        this.driveSubsystem.setDefaultCommand(
            new DriveCmd(
                this.driveSubsystem,
                this.driverController
            )
        );

        // 預設：收回、滾輪停止
        this.intake.setDefaultCommand(
            Commands.run(() -> this.intake.setStates(LifterState.Up, RollerState.Off), this.intake)
        );

        // 按住 A：放下並吸球
        new Trigger(this.driverController::getAButton).whileTrue(
            Commands.run(() -> this.intake.setStates(LifterState.Down, RollerState.In), this.intake)
        );

        // 按住 B：滑動脫離擋板並慢速吸球
        new Trigger(this.driverController::getBButton).whileTrue(
            Commands.run(() -> this.intake.setStates(LifterState.Slide, RollerState.SlowIn), this.intake)
        );

        // 預設：輸送停止
        this.hopper.setDefaultCommand(
            Commands.run(() -> this.hopper.setState(HopperState.Off), this.hopper)
        );

        // 按住 X：輸送球往 Shooter
        new Trigger(this.driverController::getXButton).whileTrue(
            Commands.run(() -> this.hopper.setState(HopperState.Convey), this.hopper)
        );

        // 預設：飛輪停止、仰角歸零、推球停止
        this.shooter.setDefaultCommand(
            Commands.run(() -> this.shooter.setStates(
                FlywheelState.Off, HoodState.Default, FeederState.Off), this.shooter)
        );

        // 按住 Y：預轉飛輪並自動調仰角，到位後才推球並啟動 Hopper
        new Trigger(this.driverController::getYButton).whileTrue(
            Commands.run(() -> {
                boolean ready = this.shooter.flywheelAndHoodAtSetpoint();
                this.shooter.setStates(
                    FlywheelState.Auto,
                    HoodState.AutoAim,
                    ready ? FeederState.Push : FeederState.Off);
                this.hopper.setState(ready ? HopperState.Convey : HopperState.Off);
            }, this.shooter, this.hopper)
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
