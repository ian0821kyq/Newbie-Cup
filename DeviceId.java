package frc.robot;

public final class DeviceId {

    public static final class DriveMotor {
        public static final int FRONT_LEFT = 1;
        public static final int BACK_LEFT = 2;
        public static final int FRONT_RIGHT = 3;
        public static final int BACK_RIGHT = 4;
    }

    public static final class Conveyer {
        public static final int CONVEYER = 0;
    }

    public static final class Intake {
        public static final int LIFTER = 5; // TODO: 改成實際 CAN ID
        public static final int ROLLER = 6; // TODO: 改成實際 CAN ID
    }

    public static final class Hopper {
        public static final int ROLLER = 7; // TODO: 改成實際 CAN ID
        public static final int CENTER = 8; // TODO: 改成實際 CAN ID
    }

    public static final class Shooter {
        public static final int FLYWHEEL_LEADER = 9;     // TODO: 改成實際 CAN ID
        public static final int FLYWHEEL_FOLLOWER = 10;  // TODO: 改成實際 CAN ID
        public static final int HOOD = 11;               // TODO: 改成實際 CAN ID
        public static final int FEEDER = 12;             // TODO: 改成實際 CAN ID
    }

    private DeviceId() {
    }
}
