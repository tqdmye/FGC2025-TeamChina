package org.firstinspires.ftc.teamcode.common.constants;

public enum OtherConstants {
    VISION_DESIRED_DISTANCE(120.0),
    VISION_SPEED_GAIN(0.04),
    VISION_TURN_GAIN(0.02),
    VISION_MAX_AUTO_SPEED(1),
    VISION_MAX_AUTO_TURN(1),

    SHOOTER_P(2),
    SHOOTER_I(0),
    SHOOTER_D(0),
    SHOOTER_F(14.5),
    SHOOTER_PREPARE_VELOCITY(1000.0),
    SHOOTER_VELOCITY(1400.0),
    SHOOTER_MIN_VELOCITY(1160.0),
    PRESHOOTER_POWER(0.8),

    BLENDER_POWER(1),
    INTAKE_POWER(1)
    ;

    public final double value;

    OtherConstants(double value) {
        this.value = value;
    }
}
