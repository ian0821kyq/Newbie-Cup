package frc.robot;

public final class Constants {

    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static final class Drive {
        public static final double MAX_SPEED = 0.5;
        public static final double MAX_TURN_SPEED = 0.7;
        public static final double DEADBAND = 0.05;

        public static final boolean FRONT_LEFT_INVERTED = false;
        public static final boolean BACK_LEFT_INVERTED = false;
        public static final boolean FRONT_RIGHT_INVERTED = true;
        public static final boolean BACK_RIGHT_INVERTED = true;
    }

    public static final class Conveyer {
        public static final double MAX_DRIVE_SPEED = 0.5;
    }

    public static final class Intake {
        // Lifter (單位: 馬達圈數)
        public static final double LIFTER_UP = 0.05;
        public static final double LIFTER_DOWN = 0.29;
        public static final double LIFTER_ZERO = 0.03;
        public static final double LIFTER_SLIDE = 0.29;
        public static final double SLIDE_AMPLITUDE = 0.05;
        public static final double SLIDE_FREQUENCY = 4.0;

        public static final double LIFTER_KP = 60.0;
        public static final double LIFTER_KD = 0.5;
        public static final double LIFTER_KS = 0.1;
        public static final double LIFTER_CRUISE_VELOCITY = 2.0;
        public static final double LIFTER_ACCELERATION = 8.0;
        public static final double LIFTER_CURRENT_LIMIT = 40.0;
        public static final boolean LIFTER_INVERTED = false;
        public static final double LIFTER_MIN = 0.0;  // 圈
        public static final double LIFTER_MAX = 0.3;  // 圈

        // Roller (單位: 伏特)
        public static final double ROLLER_OFF = 0.0;
        public static final double ROLLER_REST = 1.0;
        public static final double ROLLER_SLOW_IN = 5.0;
        public static final double ROLLER_IN = 7.0;
        public static final double ROLLER_CURRENT_LIMIT = 60.0;
        public static final boolean ROLLER_INVERTED = true;
    }

    public static final class Hopper {
        public static final double ROLLER_CONVEY_VOLTS = 6.0;
        public static final double CENTER_CONVEY_VOLTS = 6.5;
        public static final double CURRENT_LIMIT = 60.0;
        public static final boolean ROLLER_INVERTED = false;
        public static final boolean CENTER_INVERTED = true;
    }

    public static final class Shooter {
        public static final double FLYWHEEL_KV = 0.12;
        public static final double FLYWHEEL_KP = 0.3;
        public static final double FLYWHEEL_KS = 0.1;
        public static final boolean FLYWHEEL_FOLLOWER_OPPOSED = true;
        public static final double FLYWHEEL_TOLERANCE = 2.0; // rps

        public static final double HOOD_GEAR_RATIO = 294.0;
        public static final double HOOD_KP = 60.0;
        public static final double HOOD_KD = 0.5;
        public static final double HOOD_KS = 0.1;
        public static final double HOOD_CRUISE_VELOCITY = 1.0;
        public static final double HOOD_ACCELERATION = 4.0;
        public static final double HOOD_TOLERANCE = 1.0; // degree
        public static final double HOOD_MIN_DEG = 0.0;
        public static final double HOOD_MAX_DEG = 30.0;

        // 沒有定位系統，暫用固定距離 (公尺)
        public static final double DEFAULT_DISTANCE = 3.0;
    }

    private Constants() {
    }
}
