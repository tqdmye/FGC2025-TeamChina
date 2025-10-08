package org.firstinspires.ftc.teamcode.common.constants;

public enum ServoConstants {
    LEFTHOLDER_HOLD(0.9),
    RIGHTHOLDER_HOLD(0.15),
    LEFTHOLDER_PREPARE(0.7),
    RIGHTHOLDER_PREPARE(0.35),
    LEFTHOLDER_OPEN(0.05),
    RIGHTHOLDER_OPEN(1),

    BRAKE_ASCENT_LOCK(0.3),
    BRAKE_ASCENT_UNLOCK(0.45)
    ;

    public final double value;

    ServoConstants(double value) {
        this.value = value;
    }
}
