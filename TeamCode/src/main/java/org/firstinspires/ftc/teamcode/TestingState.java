package org.firstinspires.ftc.teamcode;

public enum TestingState {
    GLOBAL_ENTRY,
    CIRCLE_ENTRY,
        POWER_MOTORS,
    MAZE_ENTRY,
        ROTATE_SERVO_NORTH,
        ROTATE_SERVO_EAST,
        ROTATE_SERVO_WEST,
        CLASSIC_MOVE_UNTIL_WALL,
        CLASSIC_CHECK_RIGHT_VALIDITY,
        CLASSIC_TURN_RIGHT,
        CLASSIC_CHECK_LEFT_VALIDITY,
        CLASSIC_TURN_LEFT,
        CLASSIC_CHECK_BACKUP_NECESSARY,
        CLASSIC_MOVE_BACKUP,
        CLASSIC_ADJUST_FOR_BACKUP,
        S_REGULATE_ANGLE,
    GLOBAL_END,
}

