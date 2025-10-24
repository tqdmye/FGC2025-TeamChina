package org.firstinspires.ftc.teamcode.common.constants;

public enum Constants {
    VOLT_TO_DEG(81.1),

    VISION_DESIRED_DISTANCE(100.0),
    VISION_SPEED_GAIN(0.04),
    VISION_TURN_GAIN(0.02),
    VISION_MAX_AUTO_SPEED(1),
    VISION_MAX_AUTO_TURN(1),

    SHOOTER_P(2),
    SHOOTER_I(0),
    SHOOTER_D(0),
    SHOOTER_F(14.5),
    SHOOTER_PREPARE_VELOCITY(0),
    FRONT_SHOOTER_VELOCITY(1400.0),
    FRONT_SHOOTER_ACCELERATE_VELOCITY(1500.0),
    BACK_SHOOTER_VELOCITY(1600.0),
    SHOOTER_MIN_VELOCITY(1060.0),
    PRESHOOTER_POWER(0.9),

    BLENDER_POWER(1),
    INTAKE_POWER(1),
    ;

    public final double value;

    Constants(double value) {
        this.value = value;
    }
}
