package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.constants.XDriveConstants.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.MotorVelocityData;
import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.Pose2d;

import org.firstinspires.ftc.teamcode.core.lib.gamepad.GamepadManager;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.core.lib.pid.PIDController;
import org.firstinspires.ftc.teamcode.robot.constants.AutonomousConstants;
import org.firstinspires.ftc.teamcode.robot.constants.DrivetrainBuilderConstants;
import org.firstinspires.ftc.teamcode.robot.constants.DrivetrainState;
import org.firstinspires.ftc.teamcode.robot.constants.XDriveConstants;
import org.opencv.core.Mat;

public class XDrive implements Subsystem {
    public DrivetrainState currentDriveState = DrivetrainState.MANUAL_CONTROL;
    private  Telemetry telemetry;
    private static XDrive instance;
    private double reset_angle = 0;
    private DcMotor front_left = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private DcMotor front_right = null;
    private IMU imu;


    private PIDController XpositionPID=AutonomousConstants.stopAtPointPID;
    private PIDController YpositionPID=AutonomousConstants.stopAtPointPID;
    private PIDController HpositionPID=AutonomousConstants.stopAtPointPID;

    Pose2d currentPose = START_POSITION;
    private int prevFLTicks1 = 0,prevFRTicks1 = 0,prevBLTicks1 = 0,prevBRTicks1 = 0,prevFLTicks2 = 0,prevFRTicks2 = 0,prevBLTicks2 = 0,prevBRTicks2 = 0;

    private XDrive() {
    }

    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        front_left = hardwareMap.dcMotor.get(MOTOR_FRONT_LEFT);
        back_left = hardwareMap.dcMotor.get(MOTOR_BACK_LEFT);
        back_right = hardwareMap.dcMotor.get(MOTOR_BACK_RIGHT);
        front_right = hardwareMap.dcMotor.get(MOTOR_FRONT_RIGHT);

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.telemetry = telemetry;

        imu = hardwareMap.get(IMU.class, "imu");
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
    }

    @Override
    public void execute(GamepadManager gamepadManager) {
        currentDriveState=controlDriveState(gamepadManager.getDriver());
        if (currentDriveState == DrivetrainState.MANUAL_CONTROL) {
            drive(gamepadManager.getDriver());
            resetAngle(gamepadManager.getDriver());
        }
    }
    public DrivetrainState controlDriveState(SmartGamepad driver){
        if (Math.abs(driver.getLeftStickY())> XDriveConstants.APRIL_TAG_BREAK_TOLERANCE ||Math.abs(driver.getRightStickX())>XDriveConstants.APRIL_TAG_BREAK_TOLERANCE){
            return DrivetrainState.MANUAL_CONTROL;
        } else{
            return DrivetrainState.APRIL_TAG_ALIGNMENT;
        }
    }

    public void drive(SmartGamepad driver) {
        double rotate =  -driver.getRightStickX();
        double stick_x = driver.getLeftStickX();
        double stick_y = driver.getLeftStickY();

        double Px = 0;
        double Py = 0;

        double gyroAngle = getHeading() * Math.PI / 180;
        gyroAngle = capGyroAngle(gyroAngle);

        //MOVEMENT
        Py = stick_x * Math.cos(gyroAngle) + stick_y * Math.sin(gyroAngle);
        Px = stick_x * Math.sin(-gyroAngle) + stick_y * Math.cos(-gyroAngle);

        double maxValue = Math.abs(Px)+Math.abs(Py)+Math.abs(rotate);
        double divisor = Math.max(maxValue,1);
        double fL = (Py + Px + rotate) * 0.8 / divisor;
        double fR = (Py - Px - rotate) * 0.8 / divisor;
        double bL = (Py - Px + rotate) * 0.8 / divisor;
        double bR = (Py + Px - rotate) * 0.8 / divisor;

        front_left.setPower(fL);
        back_left.setPower(bL);
        back_right.setPower(bR);
        front_right.setPower(fR);
    }
    private double capGyroAngle(double gyroAngle){
        if (gyroAngle <= 0) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
        }
        return gyroAngle;
    }

    public void resetAngle(SmartGamepad driver){
        if(driver.isButtonA()){
            reset_angle = getHeading() + reset_angle;
        }
    }

    public double getHeading(){
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double heading = angles.getYaw(AngleUnit.DEGREES);
        if(heading < -180) {
            heading = heading + 360;
        }
        else if(heading > 180){
            heading = heading - 360;
        }
        heading = heading - reset_angle;
        return heading;
    }

    public static synchronized XDrive getInstance() {
        if (instance == null) {
            instance = new XDrive();
        }
        return instance;
    }
    public void setPower(double FLSpeed,double FRSpeed,double BLSpeed,double BRSpeed){
        front_left.setPower(FLSpeed);
        front_right.setPower(FRSpeed);
        back_left.setPower(BLSpeed);
        back_right.setPower(BRSpeed);
    }

    public void alignAtTag(Pose2d tagPosition,SmartGamepad driver){
        double VX = -YpositionPID.calculate(AutonomousConstants.ALIGN_AT_TAG_DISTANCE,tagPosition.getY());
        double VY = driver.getLeftStickX();
        double VH = -HpositionPID.calculate(0,tagPosition.getHeadingRadians())*AutonomousConstants.HEADING_APRIL_CONSTANT;
        MotorVelocityData wheelVels = getDesiredWheelVelocities(new Pose2d(VX,VY,VH));
        telemetry.addData("VX",VX);
        telemetry.addData("VY",VY);
        telemetry.addData("VH",VH);
        telemetry.addData("X position detected from tags:", currentPose.getX());
        telemetry.addData("Y position detected from tags:", currentPose.getY());
        telemetry.addData("Heading detected from tags:", currentPose.getHeadingDegrees());
        telemetry.addData("Target X position: ",AutonomousConstants.ALIGN_AT_TAG_DISTANCE);
        telemetry.addData("Target Heading: ",0.0);


        setPower(wheelVels.velocityFrontLeft,wheelVels.velocityFrontRight, wheelVels.velocityBackLeft,wheelVels.velocityBackRight);
    }
    public MotorVelocityData getDesiredWheelVelocities(Pose2d botRelativeVelocity){
        return new MotorVelocityData().updateAppliedVelocities(botRelativeVelocity).normalize();
    }
    public Pose2d getCurrentPose(){
        return currentPose;
    }
    public void setCurrentPose(Pose2d NewPose){currentPose = NewPose;}

    public  void relativeOdometryUpdate() {
        double dtheta;
        Pose2d robotPoseDelta = wheelToRobotVelocities();

        dtheta=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-currentPose.getHeadingRadians()>Math.PI?
            -2*Math.PI+imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-currentPose.getHeadingRadians():
            imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-currentPose.getHeadingRadians();


        double botXComponentRelativeToField = robotPoseDelta.getX()*Math.sin(currentPose.getHeadingRadians()) - robotPoseDelta.getY()*Math.cos(currentPose.getHeadingRadians());
        double botYComponentRelativeToField = robotPoseDelta.getY()*Math.sin(currentPose.getHeadingRadians()) + robotPoseDelta.getX()*Math.cos(currentPose.getHeadingRadians());

        Pose2d fieldPoseDelta = new Pose2d(botXComponentRelativeToField,botYComponentRelativeToField,
                robotPoseDelta.getHeadingDegrees());
        currentPose.updatePose(
                currentPose.getX() + fieldPoseDelta.getX(),
                currentPose.getY() + fieldPoseDelta.getY(),
                currentPose.getHeadingRadians() + dtheta);
    }
    public Pose2d wheelToRobotVelocities() {
        double frontLeft = (front_left.getCurrentPosition()-prevFLTicks1)*AutonomousConstants.TICK_TO_CM_CONVERSION_VALUE;
        double rearLeft = (back_left.getCurrentPosition()-prevBLTicks1)*AutonomousConstants.TICK_TO_CM_CONVERSION_VALUE;
        double rearRight = (back_right.getCurrentPosition()-prevBRTicks1)*AutonomousConstants.TICK_TO_CM_CONVERSION_VALUE;
        double frontRight = (front_right.getCurrentPosition()-prevFRTicks1)*AutonomousConstants.TICK_TO_CM_CONVERSION_VALUE;

        prevFLTicks2 = prevFLTicks1;
        prevFRTicks2 = prevFRTicks1;
        prevBRTicks2 = prevBRTicks1;
        prevBLTicks2 = prevBLTicks1;

        prevFLTicks1 = front_left.getCurrentPosition();
        prevFRTicks1 = front_right.getCurrentPosition();
        prevBLTicks1 = back_left.getCurrentPosition();
        prevBRTicks1 = back_right.getCurrentPosition();

        return new Pose2d(
                frontLeft+frontRight+rearLeft+rearRight/4,
                (rearLeft + frontRight - frontLeft - rearRight)/4,
                imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate*1
        );
    }

    public Pose2d configTickToCMConstant(){
        Pose2d robotPoseDelta = wheelToRobotVelocities();
        currentPose.updatePose(
                currentPose.getX() + robotPoseDelta.getX()/AutonomousConstants.TICK_TO_CM_CONVERSION_VALUE,
                currentPose.getY() + robotPoseDelta.getY()/AutonomousConstants.TICK_TO_CM_CONVERSION_VALUE,
                0);
        return currentPose;
    }
}
