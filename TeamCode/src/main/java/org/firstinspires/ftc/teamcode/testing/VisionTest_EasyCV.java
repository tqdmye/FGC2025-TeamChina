//package org.firstinspires.ftc.teamcode.testing;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import android.annotation.SuppressLint;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.apriltag.AprilTagDetectorJNI;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//import java.util.ArrayList;
//
//class Vision extends OpenCvPipeline {
//    private long nativeApriltagPtr;
//    private Mat gray = new Mat();
//    private ArrayList<AprilTagDetection> detections = new ArrayList<>();
//
//    // 相机参数 - 需要根据你的相机校准结果调整
//    private double fx = 578.272;
//    private double fy = 578.272;
//    private double cx = 402.145;
//    private double cy = 221.506;
//
//    // AprilTag参数
//    private double tagsize = 0.166; // 单位:米
//
//    public Vision() {
//        // 初始化AprilTag检测器
//        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(
//                "tag36h11",  // 标签家族
//                3,
//                3
//        );
//    }
//
//    @Override
//    public Mat processFrame(Mat input) {
//        // 转换为灰度图
//        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
//
//        // 检测AprilTags
//        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(
//                nativeApriltagPtr,
//                gray,
//                tagsize,
//                fx, fy, cx, cy
//        );
//
//        // 绘制检测结果
//        for(AprilTagDetection detection : detections) {
//            // 绘制边界框
//            Point[] corners = detection.corners;
//            for(int i = 0; i < 4; i++) {
//                Imgproc.line(input, corners[i], corners[(i+1)%4], new Scalar(0, 255, 0), 2);
//            }
//
//            // 绘制中心点和ID
//            Point center = detection.center;
//            Imgproc.circle(input, center, 5, new Scalar(0, 0, 255), -1);
//            Imgproc.putText(input, ""+detection.id,
//                    new Point(center.x - 10, center.y - 10),
//                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(0, 0, 255), 2);
//        }
//
//        return input;
//    }
//
//    public ArrayList<AprilTagDetection> getLatestDetections() {
//        return detections;
//    }
//
//    @Override
//    public void finalize() {
//        // 释放资源
//        if(nativeApriltagPtr != 0) {
//            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
//            nativeApriltagPtr = 0;
//        }
//        gray.release();
//    }
//}
//
//@TeleOp(name = "Vision Test")
//public class VisionTest extends LinearOpMode {
//    OpenCvWebcam webcam;
//    Vision pipeline;
//
//    @SuppressLint("DefaultLocale")
//    @Override
//    public void runOpMode() {
//        // 初始化摄像头
//        int cameraMonitorViewId = hardwareMap.appContext.getResources()
//                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        pipeline = new Vision();
//        webcam.setPipeline(pipeline);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Error", errorCode);
//                telemetry.update();
//            }
//        });
//
//        waitForStart();
//
//        while(opModeIsActive()) {
//            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();
//
//            telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//            // Step through the list of detections and display info for each one
//            for (AprilTagDetection detection : currentDetections) {
//                telemetry.addLine(String.format("\n==== (ID %d)", detection.id));
//                telemetry.addLine(String.format("ID: ", detection.id));
//
//                // 位置信息（单位：米，转换为厘米）
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)",
//                        detection.pose.x*100, detection.pose.y*100, detection.pose.z*100));
//
////                 旋转信息（弧度转角度）
//                MatrixF rotationMatrix = detection.pose.R;
//                double pitch = Math.atan2(-rotationMatrix.get(2, 0),
//                        Math.sqrt(rotationMatrix.get(2, 1) * rotationMatrix.get(2, 1) +
//                                rotationMatrix.get(2, 2) * rotationMatrix.get(2, 2)));
//                double roll = Math.atan2(rotationMatrix.get(2, 1), rotationMatrix.get(2, 2));
//                double yaw = Math.atan2(rotationMatrix.get(1, 0), rotationMatrix.get(0, 0));
//
//                // 转换为角度
//                pitch = Math.toDegrees(pitch);
//                roll = Math.toDegrees(roll);
//                yaw = Math.toDegrees(yaw);
//
//                telemetry.addLine(String.format("\n (ID %d)", detection.id));
//                telemetry.addLine(String.format("XYZ %.1f %.1f %.1f (cm)",
//                        detection.pose.x * 100, detection.pose.y * 100, detection.pose.z * 100));
//                telemetry.addLine(String.format("PRY %.1f %.1f %.1f (deg)", pitch, roll, yaw));
//                telemetry.addLine(String.format("Center %.0f %.0f (pixels)",
//                        detection.center.x, detection.center.y));
//
//                // 计算范围、方位角和仰角
//                double range = Math.sqrt(
//                        detection.pose.x*detection.pose.x +
//                                detection.pose.y*detection.pose.y +
//                                detection.pose.z*detection.pose.z)*100;
//                double bearing = Math.toDegrees(Math.atan2(detection.pose.y, detection.pose.x));
//                double elevation = Math.toDegrees(Math.atan2(detection.pose.z,
//                        Math.sqrt(detection.pose.x*detection.pose.x + detection.pose.y*detection.pose.y)));
//
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)",
//                        range, bearing, elevation));
//
//                // 中心点坐标
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)",
//                        detection.center.x, detection.center.y));
//            }
//
//            // 添加解释信息
//            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//            telemetry.addLine("RBE = Range, Bearing & Elevation");
//
//            sleep(20);
//        }
//
//        webcam.stopStreaming();
//    }
//}