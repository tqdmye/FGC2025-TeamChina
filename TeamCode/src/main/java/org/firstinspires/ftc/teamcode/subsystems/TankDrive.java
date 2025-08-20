package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class TankDrive extends SubsystemBase {
    public final Motor leftDrive;
    public final Motor rightDrive;
    private final IMU imu;
    private final DifferentialOdometry odometry;


    public TankDrive (final HardwareMap hardwareMap) {
        leftDrive  = new Motor(hardwareMap, "leftDrive");
        rightDrive = new Motor(hardwareMap, "rightDrive");

        leftDrive.setInverted(true);
        rightDrive.setInverted(false);
        leftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftDrive.setRunMode(Motor.RunMode.RawPower);
        rightDrive.setRunMode(Motor.RunMode.RawPower);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,  //todo
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));  //todo
        imu.initialize(parameters);

        odometry = new DifferentialOdometry(new Pose2d(
                0, 0, Rotation2d.fromDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))
        ), 16.9291);
    }

    public void moveRobot(double x, double yaw) {
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;

        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        leftDrive.set(leftPower);
        rightDrive.set(rightPower);
    }

    public Pose2d getPose() {
        return odometry.getPose();
    }

    @Override
    public void periodic() {
        odometry.updatePosition(leftDrive.getDistance(), rightDrive.getDistance());
    }
}
