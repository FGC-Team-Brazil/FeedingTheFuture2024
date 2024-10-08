package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.constants.XDriveConstants.*;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.MotorVelocityData;
import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.Pose2d;

import org.firstinspires.ftc.teamcode.core.lib.gamepad.GamepadManager;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.core.lib.pid.PIDController;
import org.firstinspires.ftc.teamcode.core.util.MathUtils;
import org.firstinspires.ftc.teamcode.robot.constants.AutonomousConstants;
import org.firstinspires.ftc.teamcode.robot.constants.DrivetrainState;
import org.firstinspires.ftc.teamcode.robot.constants.XDriveConstants;

import java.util.concurrent.atomic.AtomicReference;

public class XDrive implements Subsystem {
    public DrivetrainState currentDriveState = DrivetrainState.MANUAL_CONTROL;
    private  Telemetry telemetry;
    private SmartGamepad driver;
    private static XDrive instance;

    private double reset_angle = 0;
    private PIDController XpositionPID=AutonomousConstants.stopAtPointPID;
    private PIDController YpositionPID=AutonomousConstants.stopAtPointPID;
    private PIDController HpositionPID=AutonomousConstants.stopAtPointPID;

    Pose2d currentPose = START_POSITION;
    private int prevFLTicks1 = 0,prevFRTicks1 = 0,prevBLTicks1 = 0,prevBRTicks1 = 0,prevFLTicks2 = 0,prevFRTicks2 = 0,prevBLTicks2 = 0,prevBRTicks2 = 0;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private IMU imu;


    private XDrive() {
    }

    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        frontLeft = hardwareMap.dcMotor.get(MOTOR_FRONT_LEFT);
        backLeft = hardwareMap.dcMotor.get(MOTOR_BACK_LEFT);
        backRight = hardwareMap.dcMotor.get(MOTOR_BACK_RIGHT);
        frontRight = hardwareMap.dcMotor.get(MOTOR_FRONT_RIGHT);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.telemetry = telemetry;


        RevHubOrientationOnRobot robotOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(robotOrientation);
        imu.initialize(parameters);
    }

    @Override
    public void start() {
        imu.resetYaw();
    }

    @Override
    public void stop() {
    }

    @Override
    public void execute(GamepadManager gamepadManager) {
        /*currentDriveState=controlDriveState(gamepadManager.getDriver());
        if (currentDriveState == DrivetrainState.MANUAL_CONTROL) {
            drive(gamepadManager.getDriver());
            resetAngle(gamepadManager.getDriver());
        }*/
       drive(gamepadManager.getDriver());
       resetAngle(gamepadManager.getDriver());
    }
    public DrivetrainState controlDriveState(SmartGamepad driver){
        if (Math.abs(driver.getLeftStickY())> XDriveConstants.APRIL_TAG_BREAK_TOLERANCE ||Math.abs(driver.getRightStickX())>XDriveConstants.APRIL_TAG_BREAK_TOLERANCE){
            return DrivetrainState.MANUAL_CONTROL;
        } else{
            return DrivetrainState.APRIL_TAG_ALIGNMENT;
        }
    }

    public void drive(SmartGamepad driver) {
        double rotate = driver.getRightStickX()/4;
        AtomicReference<Double> stick_x = new AtomicReference<>(driver.getLeftStickX() * Math.sqrt(Math.pow(1 - Math.abs(rotate), 2) / 2));
        AtomicReference<Double> stick_y = new AtomicReference<>(driver.getLeftStickY() * Math.sqrt(Math.pow(1 - Math.abs(rotate), 2) / 2));
        double theta;
        double pX;
        double pY;
        double gyroAngle = -getHeading();

        driver.whileButtonDPadUp().run(
                () -> {
                    stick_y.set(-1.0);}
        );
        driver.whileButtonDPadDown().run(
                () -> {stick_y.set(1.0);}
        );
        driver.whileButtonDPadLeft().run(
                () -> {stick_x.set(-1.0);}
        );
        driver.whileButtonDPadRight().run(
                () -> {
                    stick_x.set(1.0);}
        );

        //MOVEMENT
        theta = Math.atan2(stick_y.get(), stick_x.get()) - gyroAngle - (Math.PI / 2);
        pX = Math.sqrt(Math.pow(stick_x.get(), 2) + Math.pow(stick_y.get(), 2)) * (Math.sin(theta + Math.PI / 4));
        pY = Math.sqrt(Math.pow(stick_x.get(), 2) + Math.pow(stick_y.get(), 2)) * (Math.sin(theta - Math.PI / 4));

        telemetry.addData("Stick_X", stick_x);
        telemetry.addData("Stick_Y", stick_y);
        telemetry.addData("imu", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("Front Left", pY - rotate);
        telemetry.addData("Back Left", pX - rotate);
        telemetry.addData("Back Right", pY + rotate);
        telemetry.addData("Front Right", pX + rotate);

        frontLeft.setPower(pY - rotate);
        backLeft.setPower(pX - rotate);
        backRight.setPower(pY + rotate);
        frontRight.setPower(pX + rotate);
    }

    public void resetAngle(SmartGamepad driver){
        if(driver.isButtonA()){
            imu.resetYaw();
        }
    }

    public double getHeading(){
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double gyroAngle = MathUtils.degreesToRadians(angles.firstAngle);
        if (gyroAngle <= 0) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
        }
        return gyroAngle;
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

    public static synchronized XDrive getInstance() {
        if (instance == null) {
            instance = new XDrive();
        }
        return instance;
    }
    public void setPower(double FLSpeed,double FRSpeed,double BLSpeed,double BRSpeed){
        frontLeft.setPower(FLSpeed);
        frontRight.setPower(FRSpeed);
        backLeft.setPower(BLSpeed);
        backRight.setPower(BRSpeed);
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
        double frontLeftPos = (frontLeft.getCurrentPosition()-prevFLTicks1)*AutonomousConstants.TICK_TO_CM_CONVERSION_VALUE;
        double rearLeftPos = (backLeft.getCurrentPosition()-prevBLTicks1)*AutonomousConstants.TICK_TO_CM_CONVERSION_VALUE;
        double rearRightPos = (backRight.getCurrentPosition()-prevBRTicks1)*AutonomousConstants.TICK_TO_CM_CONVERSION_VALUE;
        double frontRightPos = (frontRight.getCurrentPosition()-prevFRTicks1)*AutonomousConstants.TICK_TO_CM_CONVERSION_VALUE;

        prevFLTicks2 = prevFLTicks1;
        prevFRTicks2 = prevFRTicks1;
        prevBRTicks2 = prevBRTicks1;
        prevBLTicks2 = prevBLTicks1;

        prevFLTicks1 = frontLeft.getCurrentPosition();
        prevFRTicks1 = frontRight.getCurrentPosition();
        prevBLTicks1 = backLeft.getCurrentPosition();
        prevBRTicks1 = backRight.getCurrentPosition();

        return new Pose2d(
                frontLeftPos+frontRightPos+rearLeftPos+rearRightPos/4,
                (rearLeftPos + frontRightPos - frontLeftPos - rearRightPos)/4,
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
