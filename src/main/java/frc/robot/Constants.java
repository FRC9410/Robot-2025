package frc.robot;
import java.util.List;

public final class Constants {

    public static final class OIConstants {
         public static final int DRIVER_CONTROLLER_PORT = 0;  // Typically, USB port 0
         public static final int OPERATOR_CONTROLLER_PORT = 1;  // Typically, USB port 1
         // Deadband constant for joystick/controller input (placeholder, adjust as needed)
         public static final double DEADBAND = 0.05;
    }
    
    public static final class CanBusConstants {
         public static final String CANIVORE_BUS = "canivore";
    }
    
    public static final class AutoConstants {
         // Autonomous mode settings (placeholders; adjust as needed)
         public static final double MAX_SPEED_METERS_PER_SECOND = 3.0;
         public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;
         // Ramsete controller constants (placeholders; tune as needed)
         public static final double RAMSETE_B = 2.0;
         public static final double RAMSETE_ZETA = 0.7;
    }
    
    public static final class VisionConstants {
        // Vision system constants (placeholders—adjust as needed)
        public static final String CAMERA_NAME = "limelight";
        public static final double CAMERA_FOV_DEGREES = 60.0;
        public static final int IMAGE_WIDTH = 320;
        public static final int IMAGE_HEIGHT = 240;
        // Example threshold and distance constants
        public static final double TARGET_AREA_THRESHOLD = 1.0;
        public static final double MAX_TARGET_DISTANCE_METERS = 5.0;
        
        // Additional Vision constants for Reef Vision:
        public static final String LEFT_PERIMETER_TABLE = "left-perimeter";  // Placeholder table name
        public static final String RIGHT_PERIMETER_TABLE = "right-perimeter";  // Placeholder table name
        public static final String LEFT_REEF_TABLE = "left-reef";  // Placeholder table name
        public static final String RIGHT_REEF_TABLE = "right-reef";  // Placeholder table name
        
        // Lists for reef vision excluded tags and perimeter excluded tags (placeholders)
        public static final List<Integer> REEF_VISION_EXCLUDED_TAGS = List.of(1, 2, 3);
        public static final List<Integer> PERIMETER_EXCLUDED_TAGS = List.of(4, 5, 6);
    }
    
    public static final class SensorConstants {
        // Intake Laser constants
        public static final int INTAKE_LASER_CAN_ID = 17;
        public static final int OUTTAKE_LASER_CAN_ID = 18;
    }
    
    public static final class AlgaeIntakeConstants {
        public static final int CAN_ID = 13;
        // PID stall control constants for the Algae Intake (placeholders—adjust as needed)
        public static final int PID_SLOT = 0;
        // PID Constants (placeholders—adjust as needed)
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
        public static final double STALL_FEEDFORWARD = 0.0;
        // Speeds for intake and outtake (placeholder values; adjust as needed)
        public static final double INTAKE_VOLTAGE = 0.7;
        public static final double OUTTAKE_VOLTAGE = -0.7;
        public static final double STOP_VOLTAGE = 0.0;
    }
    
    public static final class AlgaeWristConstants {
        public static final int CAN_ID = 14; 
        // PID Constants (placeholders—adjust as needed)
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;

        public static final double MIN_POSITION = 0.0;  // Placeholder
        public static final double MAX_POSITION = 0.0;    // Placeholder
        public static final double OFFSET_POSITION = 0.0;       // Placeholder
        // Additional preset angles for Algae Wrist positions (in units; placeholders)
        public static final double REEF_POSITION = 0.0;    // Placeholder value for reef angle
        public static final double LAYUP_POSITION = 0.0;   // Placeholder value for layup angle
        public static final double PLACE_POSITION = 0.0;   // Placeholder value for place angle
        public static final double WRIST_VOLTAGE = 0.0;
        public static final double STOP_VOLTAGE = 0.0;

        // SmartMotion parameters for wrist PID control (placeholders—adjust as needed)
        public static final double MAX_ACC = 25000.0;       // Maximum acceleration (units/sec^2)
        public static final double MAX_VEL = 11000.0;        // Maximum velocity (units/sec)
        public static final double ALLOWED_ERROR = 5.0;    // Allowed closed-loop error (units)
    }
    
    public static final class MapConstants {
        public static final String ELEVATOR_POSITION = "elevatorPosition";
        public static final String END_EFFECTOR_VOLTAGE = "endEffectorVoltage";
        public static final String HOPPER_VOLTAGE = "hopperVoltage";
        public static final String CLIMBER_POSITION = "climberPosition";
        public static final String WRIST_POSITION = "wristPosition";
        public static final String INTAKE_VOLTAGE = "intakeSpeed";


    }

    public static final class ClimberConstants {
        public static final int CAN_ID = 10;
        public static final double CLIMBER_DEFAULT_POSITION = 0.0; // Placeholder value
        
        // PID Constants (placeholders—adjust as needed)
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
        
        // Additional constants for Climber: winch extension and climb units (placeholders)
        public static final double WINCH_EXTENSION_POSITION = 2.0; // Placeholder value
        public static final double WINCH_CLIMB_POSITION = 1.8;     // Placeholder value
        public static final double CLIMB_VOLTAGE = 0.0;
        public static final double STOP_VOLTAGE = 0.0;
    }
    
    public static final class ElevatorConstants {
        public static final int CAN_ID = 9;
        public static final double ELEVATOR_DEFAULT_HEIGHT = 0.0; // Placeholder value
        
        // PID Constants (placeholders—adjust as needed)
        public static final double kP = 80.0; 
        public static final double kI = 0.0; 
        public static final double kD = 0.0; 
        public static final double kG = 0.05;
        
        // Elevator preset heights (in units; placeholders for L1-L4, home, algae layup, and algae place)
        public static final double HOME_POSITION = 0.0;
        public static final double L1_SCORE_POSITION = 0.5;
        public static final double L2_SCORE_POSITION = 0.75;
        public static final double L3_SCORE_POSITION = 1.0;
        public static final double L4_SCORE_POSITION = 1.25;
        public static final double ALGAE_LAYUP_POSITIONS = 0.6;
        public static final double ALGAE_PLACE_POSITION = 0.8;
    }
    
    public static final class EndEffectorConstants {
        public static final int CAN_ID = 16;
        public static final double END_EFFECTOR_VOLTAGE = 0.8;
        public static final double STOP_VOLTAGE = 0.0;
    }
    
    public static final class HopperConstants {
        public static final int PRIMARY_CAN_ID = 11;
        public static final int SECONDARY_CAN_ID = 12;
        // Hopper speed constant (placeholder value; adjust as needed)
        public static final double HOPPER_VOLTAGE = 0.8;
        public static final double STOP_VOLTAGE = 0.0;
    }
}