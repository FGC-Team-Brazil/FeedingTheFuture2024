package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.Pose2d;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.GamepadManager;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.robot.constants.AutonomousConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary.Builder;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;

public class WebcamAprilTags implements Subsystem {
    private static WebcamAprilTags instance;
    private Telemetry telemetry;
    private static AprilTagProcessor aprilTag;
    static double CAM_DIST_TO_CENTER = 0;
    // distancia da camera até o centro do robo
    static double CAM_ANGLE_TO_GROUND =0;
    //angulação da câmera (0º = paralelo ao chão, 90° = olhando para cima)
    static double actualHead;
    static double xtrans;
    static double ytrans;
    Pose2d prevDetectedPose;
    private static VisionPortal visionPortal;
    public static AprilTagLibrary getFeedingTheFutureTagLibrary() {
        return (new Builder())
                .addTag(100, "Blue Nexus Goal - Field Center - Facing Platform", 160.0D, DistanceUnit.MM)
                .addTag(101, "Red Nexus Goal - Field Center - Facing Platform", 160.0D, DistanceUnit.MM)
                .addTag(102, "Red Nexus Goal - Field Center - Facing Food Warehouse", 160.0D, DistanceUnit.MM)
                .addTag(103, "Blue Nexus Goal - Field Center - Facing Food Warehouse", 160.0D, DistanceUnit.MM)
                .addTag(104, "Blue Nexus Goal - Field Edge - Alliance Station", 160.0D, DistanceUnit.MM)
                .addTag(105, "Blue Nexus Goal - Field Edge - Center Field", 160.0D, DistanceUnit.MM)
                .addTag(106, "Red Nexus Goal - Field Edge - Center Field", 160.0D, DistanceUnit.MM)
                .addTag(107, "Red Nexus Goal - Field Edge - Alliance Station", 160.0D, DistanceUnit.MM)
                .build();
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    public Pose2d LocateWithAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        if (currentDetections.size() > 0) {
            double avgX = 0;
            double avgY = 0;
            double avgHead = 0;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    double xtag = 0;
                    double ytag = 0;
                    double pos = 0;
                    switch (detection.id) {
                        //tem que ajustar os numeros para corresponder a arena
                        case 100:
                            xtag = 0;
                            ytag = 0;
                            break;
                        case 101:
                            xtag = 0;
                            ytag = 0;
                            break;
                        case 102:
                            xtag = 0;
                            ytag = 0;
                            break;
                        case 103:
                            xtag = 0;
                            ytag = 0;
                            break;
                        case 104:
                            xtag = 0;
                            ytag = 0;
                            break;
                        case 105:
                            xtag = 0;
                            ytag = 0;
                            break;
                        case 106:
                            xtag = 0;
                            ytag = 0;
                            break;
                        case 107:
                            xtag = 0;
                            ytag = 0;
                            break;
                    }
                    actualHead = -(detection.ftcPose.yaw - detection.ftcPose.bearing);
                    double actualRange = (Math.cos(detection.ftcPose.elevation+CAM_ANGLE_TO_GROUND)*detection.ftcPose.range);
                    ytrans = Math.sin(Math.toRadians(actualHead)) * actualRange * pos;
                    xtrans = Math.cos(Math.toRadians(actualHead)) * actualRange * pos;



                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("Xtrans %6.1f Ytrans %6.1f",xtrans,ytrans));
                    telemetry.addLine(String.format("CAM FIELD XY %6.1f %6.1f", xtag - xtrans, ytag - ytrans));
                    //telemetry.addLine(String.format("BOT FIELD XY HEAD %6.1f %6.1f %6.1f",
                            //(xtag - xtrans) - (Math.cos(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER),
                            //(ytag - ytrans) - (Math.sin(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER),
                            //detection.ftcPose.yaw));
                    //telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    //telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (mm, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));


                    avgX += (xtag - xtrans) - (Math.cos(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER);
                    avgY += (ytag - ytrans) - (Math.sin(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER);
                    avgHead += detection.ftcPose.yaw + 180;
                } else {
                    //telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    //telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            // Add "key" information to telemetry
            //telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            //telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            //telemetry.addLine("RBE = Range, Bearing & Elevation");
            XDrive.getInstance().setCurrentPose(new Pose2d(avgX,avgY,Math.toRadians(avgHead)));
            return new Pose2d(avgX,avgY,Math.toRadians(avgHead));
        } else{
            return XDrive.getInstance().getCurrentPose();
        }
    }

    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry NewTelemetry) {
        telemetry=NewTelemetry;
        // Create the AprilTag processor the easy way.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .setTagLibrary(getFeedingTheFutureTagLibrary())
                .build();

        // Create the vision portal the easy way.

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .build();
        telemetry.addLine("aprilTag Subsystem initialized");
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void execute(GamepadManager gamepadManager) {
        Pose2d detectedPose = LocateWithAprilTag();
        if (detectedPose != prevDetectedPose){
            XDrive.getInstance().setCurrentPose(detectedPose);
        }
        prevDetectedPose = detectedPose;
        XDrive.getInstance().relativeOdometryUpdate();
        XDrive.getInstance().alignAtTag(XDrive.getInstance().getCurrentPose(), AutonomousConstants.ALIGN_AT_TAG_DISTANCE);
    }
    public static WebcamAprilTags getInstance() {
        if (instance == null) {
            instance = new WebcamAprilTags();
        }
        return instance;
    }
}
