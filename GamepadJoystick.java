package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class GamepadJoystick extends XboxController {
    public GamepadJoystick(int port) {
        super(port);
    }
    public static final int CONTROLLER_PORT = 0;
}