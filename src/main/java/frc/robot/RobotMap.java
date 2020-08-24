package frc.robot;
public class RobotMap {
    // Vision Processing
    public static final int MIN_DIST = 60; // minimum distance to reflective tape in inches for vision processing
    
    // CAN IDs
    public static final int LEFT_DRIVE_MASTER_ID = 4; // left drive train master can id
    public static final int LEFT_DRIVE_SLAVE_ID = 3; // left drive train slave can id
    public static final int RIGHT_DRIVE_MASTER_ID = 1; // right drive train master can id
    public static final int RIGHT_DRIVE_SLAVE_ID = 2; // right drive train slave can id
    public static final int CLIMB_DRIVE_ID = 5;
    public static final int ELEVATOR_MASTER_ID = 6;
    public static final int ELEVATOR_SLAVE1_ID = 7;
    public static final int ELEVATOR_SLAVE2_ID = 8;
    public static final int ELEVATOR_SLAVE3_ID = 9;
    public static final int ARM_MASTER_ID = 10;
    public static final int ARM_SLAVE_ID = 11;
    public static final int INTAKE_TOP_ID = 12;
    public static final int INTAKE_BOTTOM_ID = 13;
    public static final int PCM_ID = 14; // pneumatics control module can id
    
    // PCM Ports
    public static final int CLIMB_HOOK_FORWARD = 6;
    public static final int CLIMB_HOOK_REVERSE = 7;
    public static final int INTAKE_PISTON_FORWARD = 0;
    public static final int INTAKE_PISTON_REVERSE = 1;
    public static final int INTAKE_PISTON2_FORWARD = 2;
    public static final int INTAKE_PISTON2_REVERSE = 3;

    // Limit Switches
    public static final int ELEVATOR_LIMIT_SWITCH = 0;
}