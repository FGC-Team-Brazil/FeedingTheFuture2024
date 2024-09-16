package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.constants.AutonomousConstants.CAM_ANGLE_TO_GROUND;
import static org.firstinspires.ftc.teamcode.robot.constants.AutonomousConstants.CAM_DIST_TO_CENTER;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.AprilTagFieldData;
import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.Pose2d;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.GamepadManager;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.robot.constants.AutonomousConstants;
import org.firstinspires.ftc.teamcode.robot.constants.DrivetrainState;
import org.firstinspires.ftc.teamcode.robot.constants.FieldSections;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary.Builder;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class WebcamAprilTags implements Subsystem {
    private static WebcamAprilTags instance;
    private Telemetry telemetry;
    private static AprilTagProcessor aprilTag;
    static double actualHead;
    private static double xTrans;
    private static double yTrans;
    private FieldSections currentSection = FieldSections.UNKNOWN_SECTION;
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
        FieldSections section = FieldSections.UNKNOWN_SECTION;
        currentSection = FieldSections.UNKNOWN_SECTION;

        // Step through the list of detections and display info for each one.
        if (currentDetections.size() > 0) {
            double distX = 0;
            double distY = 0;
            double avgHead = 0;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    AprilTagFieldData tagFieldData;
                    switch (detection.id) {
                        case 100:
                            tagFieldData = new AprilTagFieldData(-41.12, -70.5, FieldSections.BLUE_PLATAFORM_NEXUSES);
                            break;
                        case 101:
                            tagFieldData = new AprilTagFieldData(-41.12,70.5, FieldSections.RED_PLATAFORM_NEXUSES);
                            break;
                        case 102:
                            tagFieldData = new AprilTagFieldData(46.12,-70.5, section = FieldSections.RED_BACKWARDS_NEXUSES);
                            break;
                        case 103:
                            tagFieldData = new AprilTagFieldData(46.12,70.5, FieldSections.BLUE_BACKWARDS_NEXUSES);
                            break;
                        case 104:
                            tagFieldData = new AprilTagFieldData(0,0, FieldSections.BLUE_ALLIANCE_SIDE_3_NEXUSES);
                            break;
                        case 105:
                            tagFieldData = new AprilTagFieldData(0,0, FieldSections.BLUE_MIDDLE_3_NEXUSES);
                            break;
                        case 106:
                            tagFieldData = new AprilTagFieldData(0,0, FieldSections.RED_MIDDLE_3_NEXUSES);
                            break;
                        case 107:
                            tagFieldData = new AprilTagFieldData(0,0, FieldSections.RED_ALLIANCE_SIDE_3_NEXUSES);
                            break;
                    }
                    actualHead = -(detection.ftcPose.yaw - detection.ftcPose.bearing);
                    double actualRange = (Math.cos(detection.ftcPose.elevation+Math.toRadians(CAM_ANGLE_TO_GROUND))*detection.ftcPose.range);
                    yTrans = Math.sin(Math.toRadians(actualHead)) * actualRange;
                    xTrans = Math.cos(Math.toRadians(actualHead)) * actualRange;



                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("Xtrans %6.1f Ytrans %6.1f",xTrans,yTrans));
                    telemetry.addLine(String.format("CAM FIELD XY %6.1f %6.1f", xTrans, yTrans));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));


                    distX += (xTrans) + (Math.cos(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER);
                    distY += (yTrans) + (Math.sin(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER);
                    //y is useless for our method but we still calculate it for displaying
                    avgHead += detection.ftcPose.yaw;
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            return new Pose2d(distX/currentDetections.size()
                    ,distY/currentDetections.size()
                    ,Math.toRadians(avgHead)/currentDetections.size()
            );
        } else{
            return XDrive.getInstance().getCurrentPose();
        }
    }
    public double getTagFieldX(AprilTagFieldData data){
        return data.getPosX();
    }
    public double getTagFieldY(AprilTagFieldData data){
        return data.getPosY();
    }
    public FieldSections getTagFieldSection(AprilTagFieldData data){
        return data.getTagSection();
    }

    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry newTelemetry) {
        telemetry=newTelemetry;
        // Create the AprilTag processor the easy way.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setTagLibrary(getFeedingTheFutureTagLibrary())
                .build();

        // Create the vision portal the easy way.

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hardwareMap.get(WebcamName.class, AutonomousConstants.CAM_HARDWARE_MAP_NAME))
                .setCameraResolution(new Size(640,480))
                .build();

        telemetry.addLine("aprilTag Subsystem initialized");
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        visionPortal.stopStreaming();
    }

    @Override
    public void execute(GamepadManager gamepadManager) {
        Pose2d detectedPose = LocateWithAprilTag();
        XDrive.getInstance().setCurrentPose(detectedPose);
        XDrive.getInstance().relativeOdometryUpdate();


        if (gamepadManager.getDriver().isButtonX()){
            XDrive.getInstance().currentDriveState = DrivetrainState.APRIL_TAG_ALIGNMENT;
        }
        if (XDrive.getInstance().currentDriveState == DrivetrainState.APRIL_TAG_ALIGNMENT) {
            XDrive.getInstance().alignAtTag(XDrive.getInstance().getCurrentPose(), gamepadManager.getDriver());
        }
    }
    public static WebcamAprilTags getInstance() {
        if (instance == null) {
            instance = new WebcamAprilTags();
        }
        return instance;
    }
}
