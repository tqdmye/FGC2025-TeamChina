 package org.firstinspires.ftc.teamcode.testing;

 import com.acmerobotics.dashboard.FtcDashboard;
 import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
 import com.arcrobotics.ftclib.gamepad.GamepadEx;
 import com.arcrobotics.ftclib.gamepad.GamepadKeys;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

 import org.firstinspires.ftc.robotcore.external.Telemetry;
 import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
 import org.firstinspires.ftc.teamcode.commands.TankDriveCommand;
 import org.firstinspires.ftc.teamcode.common.util.SlewRateLimiter;
 import org.firstinspires.ftc.teamcode.subsystems.TankDrive;
 import org.firstinspires.ftc.vision.VisionPortal;
 import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
 import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

 import java.util.List;

 import global.first.EcoEquilibriumGameDatabase;


 @TeleOp(name = "Test Vision", group = "test")
 public class VisionTest extends LinearOpMode {
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry,
 FtcDashboard.getInstance().getTelemetry());
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private TankDrive tankDrive;
    private SlewRateLimiter driverLimiter;
    private GamepadEx gamepadEx1;

    public void initialize() {
        tankDrive = new TankDrive(hardwareMap);
        initAprilTag();
        tankDrive.setDefaultCommand(
                new TankDriveCommand(
                        tankDrive,
                        () -> -driverLimiter.calculate(gamepadEx1.getLeftY()) ,
                        () -> gamepadEx1.getRightX(),
                        () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5));
    }

    @Override
    public void runOpMode() {

        initialize();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetryAprilTag();
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
        }


        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }


    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(EcoEquilibriumGameDatabase.getEcoEquilibriumTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }


    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d)", detection.id));
                telemetry.addLine(detection.metadata.name);
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)",
 detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
 detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)",
 detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)",
 detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
 }
