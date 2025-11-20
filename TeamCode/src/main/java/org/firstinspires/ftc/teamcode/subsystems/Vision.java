package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.constants.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import global.first.EcoEquilibriumGameDatabase;

 public class Vision{
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1) +ve is forward
    double  turn            = 0;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise

    public Vision(Telemetry telemetry, final HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(EcoEquilibriumGameDatabase.getEcoEquilibriumTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        setManualExposure(telemetry,6, 250);  // Use low exposure time to reduce motion blur
    }

    public void driveWithVision(TankDrive tankDrive, Telemetry telemetry, boolean
 isDriveWithVision) {
        drive = 0;
        turn = 0;
        targetFound = false;
        desiredTag  = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                targetFound = true;
                desiredTag = detection;
                break;
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f centimeters", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }


        if (isDriveWithVision && targetFound) {
            double  rangeError   = (desiredTag.ftcPose.range - Constants.VISION_DESIRED_DISTANCE.value);
            double  headingError = desiredTag.ftcPose.bearing;


            drive = Range.clip(rangeError * Constants.VISION_SPEED_GAIN.value, -Constants.VISION_MAX_AUTO_SPEED.value, Constants.VISION_MAX_AUTO_SPEED.value);
            turn  = Range.clip(headingError * Constants.VISION_TURN_GAIN.value,
 -Constants.VISION_MAX_AUTO_TURN.value, Constants.VISION_MAX_AUTO_TURN.value) ;

            telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
        }
        telemetry.update();

        moveRobotVision(tankDrive, drive, turn);
    }

    private void moveRobotVision(TankDrive tankDrive, double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower    = x + yaw;
        double rightPower   = x - yaw;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max >1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        tankDrive.leftDrive.set(leftPower);
        tankDrive.rightDrive.set(rightPower);
    }

    private void setManualExposure(Telemetry telemetry, int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleep(20);
        telemetry.addData("Camera", "Ready");
        telemetry.update();
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
 }
