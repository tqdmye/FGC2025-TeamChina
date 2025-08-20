package org.firstinspires.ftc.teamcode.common.constants;

public enum OtherConstants {
    VISION_DESIRED_DISTANCE(22.0),
    VISION_SPEED_GAIN(0.04),
    VISION_TURN_GAIN(0.02),
    VISION_MAX_AUTO_SPEED(1),
    VISION_MAX_AUTO_TURN(1),

    SHOOTER_P(5),
    SHOOTER_I(0.0001),
    SHOOTER_D(0.5),
    SHOOTER_PREPARE_POWER(0.3),
    SHOOTER_POWER(0.9),
    SHOOTER_MIN_VELOCITY(1460),
    PRESHOOTER_POWER(0.8),
    ;

    public final double value;

    OtherConstants(double value) {
        this.value = value;
    }
}
