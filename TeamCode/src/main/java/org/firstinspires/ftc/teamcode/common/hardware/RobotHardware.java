package org.firstinspires.ftc.teamcode.common.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//@Config
public class RobotHardware {
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private DcMotorEx strafeDrive = null;
    private DcMotorEx frontSlide = null;
    private DcMotorEx backSlide = null;

    private Servo strafe = null;
    private Servo backDoor = null;
    private Servo frontDoor_left = null;
    private Servo frontDoor_right = null;

    private IMU imu = null;


    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final String webCamName = "WebCamFGC";

//    private static RobotHardware instance = null;
//
//    public static RobotHardware getInstance() {
//        if (instance == null) {
//            instance = new RobotHardware();
//        }
//        return instance;
//    }

    public RobotHardware(@NonNull final HardwareMap hardwareMap) {
        this.leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        this.rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        this.strafeDrive = hardwareMap.get(DcMotorEx.class,"strafe_drive");

        this.frontSlide = hardwareMap.get(DcMotorEx.class, "frontSlide");
        this.backSlide = hardwareMap.get(DcMotorEx.class,"backSlide");
        this.strafe = hardwareMap.get(Servo.class,"strafe");

        this.backDoor = hardwareMap.get(Servo.class,"doorBack");
        this.frontDoor_left = hardwareMap.get(Servo.class,"doorLeft");
        this.frontDoor_right = hardwareMap.get(Servo.class,"doorRight");
    }

//    public void startCamera() {
//        aprilTag = new AprilTagProcessor.Builder()
//                .setTagLibrary(FeedingTheFutureGameDatabase.getFeedingTheFutureTagLibrary())
//                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
//                .build();
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, webCamName))
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .addProcessors(aprilTag)
//                .enableLiveView(true)
//                .build();
//
//        visionPortal.setProcessorEnabled(aprilTag, true);
//    }
//
//    public Pose2d getApriltagDetection() {
//        if(aprilTag != null) {
//            List<AprilTagDetection> detections = aprilTag.getDetections();
//            for (AprilTagDetection detection : detections) {
//                if(detection.metadata != null) {
//
//                }
//            }
//            return new Pose2d();
//        }
//        else {
//            return null;
//        }
//    }


}
