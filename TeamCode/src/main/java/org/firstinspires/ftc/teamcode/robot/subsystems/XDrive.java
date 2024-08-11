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
import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.RobotMovementState;
import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.TargetVelocityData;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.GamepadManager;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.core.lib.pid.PIDController;
import org.firstinspires.ftc.teamcode.robot.constants.AutonomousConstants;


public class XDrive implements Subsystem {

    private  Telemetry telemetry;
    private static XDrive instance;
    private double reset_angle = 0;
    private DcMotor front_left = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private DcMotor front_right = null;
    private IMU imu;


    private PIDController motorPID = AutonomousConstants.pathFollowingPID;
    double currVX;
    double currVY;
    double currentHeading;
    double desiredVX;
    double desiredVY;
    double Headingerror;
    public RobotMovementState movementState = new RobotMovementState(0,0);
    Pose2d currentPose = START_POSITION;
    int prevFLTicks1 = 0;
    int prevFRTicks1 = 0;
    int prevBLTicks1 = 0;
    int prevBRTicks1 = 0;
    int prevFLTicks2 = 0;
    int prevFRTicks2 = 0;
    int prevBLTicks2 = 0;
    int prevBRTicks2 = 0;

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
        drive(gamepadManager.getDriver());
        resetAngle(gamepadManager.getDriver());
    }

    public void drive(SmartGamepad driver) {
        double rotate = driver.getRightStickX()/4;
        double stick_x = driver.getLeftStickX() * Math.sqrt(Math.pow(1-Math.abs(rotate), 2)/2);
        double stick_y = driver.getLeftStickY() * Math.sqrt(Math.pow(1-Math.abs(rotate), 2)/2);
        double theta = 0;
        double Px = 0;
        double Py = 0;

        double gyroAngle = getHeading() * Math.PI / 180;
        if (gyroAngle <= 0) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
        }
        gyroAngle = -1 * gyroAngle;

        if(driver.isButtonRightBumper()){
            gyroAngle = -Math.PI/2;
        }

        if(driver.isButtonDPadRight()){
            stick_x = 0.5;
        }
        else if(driver.isButtonDPadLeft()){
            stick_x = -0.5;
        }
        if(driver.isButtonDPadUp()){
            stick_y = -0.5;
        }
        else if(driver.isButtonDPadDown()){
            stick_y = 0.5;
        }


        //MOVEMENT
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));

        telemetry.addData("Stick_X", stick_x);
        telemetry.addData("Stick_Y", stick_y);
        telemetry.addData("Magnitude",  Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)));
        telemetry.addData("Front Left", Py - rotate);
        telemetry.addData("Back Left", Px - rotate);
        telemetry.addData("Back Right", Py + rotate);
        telemetry.addData("Front Right", Px + rotate);

        front_left.setPower(Py - rotate);
        back_left.setPower(Px - rotate);
        back_right.setPower(Py + rotate);
        front_right.setPower(Px + rotate);
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

    public void controlBasedOnVelocity(@NonNull TargetVelocityData movementState, double elapsedTime){

        relativeOdometryUpdate();

        desiredVX += (movementState.getXV()-currVX)*elapsedTime*(AutonomousConstants.MAXACCELERATION/AutonomousConstants.MAXSPEED);
        desiredVY += (movementState.getYV()-currVY)*elapsedTime*(AutonomousConstants.MAXACCELERATION/AutonomousConstants.MAXSPEED);
        Headingerror = (movementState.getH()-currentHeading);
        Pose2d desiredVelocityField = new Pose2d(desiredVX,desiredVY,Headingerror*elapsedTime*AutonomousConstants.HeadK);
        Pose2d desiredVelocityBot = fieldToRobotVelocity(desiredVelocityField);

        MotorVelocityData desiredMotorSpeed = getDesiredWheelVelocities(desiredVelocityBot);
        MotorVelocityData actualMotorSpeed = getRegistredWheelVelocities(elapsedTime);
        //pid for motor speeds
        double appliedFL = motorPID.calculate(desiredMotorSpeed.velocityFrontLeft,actualMotorSpeed.velocityFrontLeft);
        double appliedFR = motorPID.calculate(desiredMotorSpeed.velocityFrontRight,actualMotorSpeed.velocityFrontRight);
        double appliedBL = motorPID.calculate(desiredMotorSpeed.velocityBackLeft,actualMotorSpeed.velocityBackLeft);
        double appliedBR = motorPID.calculate(desiredMotorSpeed.velocityBackRight,actualMotorSpeed.velocityBackRight);

        setPower(appliedFL,appliedFR,appliedBL,appliedBR);
    }

    public MotorVelocityData getDesiredWheelVelocities(Pose2d botRelativeVelocity){
        return new MotorVelocityData().updateAppliedVelocities(botRelativeVelocity).normalize();
    }
    public MotorVelocityData getRegistredWheelVelocities(double elapsedTime){
        double frontLeft = (front_left.getCurrentPosition()-prevFLTicks2)/2.0/AutonomousConstants.MOTOR_REDUCTION*AutonomousConstants.WHEEL_RADIUS/elapsedTime;
        double rearLeft = (back_left.getCurrentPosition()-prevBLTicks2)/2.0/AutonomousConstants.MOTOR_REDUCTION*AutonomousConstants.WHEEL_RADIUS/elapsedTime;
        double rearRight = (back_left.getCurrentPosition()-prevBRTicks2)/2.0/AutonomousConstants.MOTOR_REDUCTION*AutonomousConstants.WHEEL_RADIUS/elapsedTime;
        double frontRight = (front_right.getCurrentPosition()-prevFRTicks2)/2.0/AutonomousConstants.MOTOR_REDUCTION*AutonomousConstants.WHEEL_RADIUS/elapsedTime;

        return new MotorVelocityData(frontLeft,frontRight,rearLeft,rearRight);
    }
    public void setPose2d(Pose2d newPose){
        currentPose =  newPose;
    }
    public Pose2d getCurrentPose(){
        return currentPose;
    }

    public  void relativeOdometryUpdate() {
        double dtheta;
        Pose2d robotPoseDelta = wheelToRobotVelocities();

        //double dtheta = currentPose.getHeadingRadians()+(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-currentPose.getHeadingRadians());
        if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-currentPose.getHeadingRadians()>Math.PI){
            dtheta = -2*Math.PI+imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-currentPose.getHeadingRadians();
        } else{
            dtheta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-currentPose.getHeadingRadians();
        }

        double botXComponentRelativeToField = robotPoseDelta.getX()*Math.sin(currentPose.getHeadingRadians()) - robotPoseDelta.getY()*Math.cos(currentPose.getHeadingRadians()); // i dont know if this is right, gotta test it
        double botYComponentRelativeToField = robotPoseDelta.getY()*Math.sin(currentPose.getHeadingRadians()) + robotPoseDelta.getX()*Math.cos(currentPose.getHeadingRadians());

        //Pair var8 = var10000;
        //double sineTerm = Math.sin(dtheta);
        //double cosTerm = Math.cos(dtheta);
        //Vector2d fieldPositionDelta = new Vector2d(sineTerm * robotPoseDelta.getX() - cosTerm * robotPoseDelta.getY(), cosTerm * robotPoseDelta.getX() + sineTerm * robotPoseDelta.getY()); probably closer to this

        Pose2d fieldPoseDelta = new Pose2d(botXComponentRelativeToField,botYComponentRelativeToField,
                robotPoseDelta.getHeadingDegrees());
        currentPose.updatePose(
                currentPose.getX() + fieldPoseDelta.getX(),
                currentPose.getY() + fieldPoseDelta.getY(),
                currentPose.getHeadingRadians() + dtheta);
    }
    public Pose2d wheelToRobotVelocities() {
        //double k = (AutonomousConstants.TRACK_WIDTH + AutonomousConstants.WHEEL_BASE) / 2.0;
        double frontLeft = (front_left.getCurrentPosition()-prevFLTicks1);
        double rearLeft = (back_left.getCurrentPosition()-prevBLTicks1);
        double rearRight = (back_right.getCurrentPosition()-prevBRTicks1);
        double frontRight = (front_right.getCurrentPosition()-prevFRTicks1);

        prevFLTicks2 = prevFLTicks1;
        prevFRTicks2 = prevFRTicks1;
        prevBRTicks2 = prevBRTicks1;
        prevBLTicks2 = prevBLTicks1;

        prevFLTicks1 = front_left.getCurrentPosition();
        prevFRTicks1 = front_right.getCurrentPosition();
        prevBLTicks1 = back_left.getCurrentPosition();
        prevBRTicks1 = back_right.getCurrentPosition();

        return new Pose2d(
                (frontLeft+frontRight+rearLeft+rearRight)*AutonomousConstants.TICK_TO_CM_CONVERSION_VALUE/4,
                (rearLeft + frontRight - frontLeft - rearRight)*AutonomousConstants.TICK_TO_CM_CONVERSION_VALUE/4,
                imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate*1
                //(rearRight + frontRight - frontLeft - rearLeft) / k/4 //can be used if no imu
        );
        //(novo x = (x * cos a - y* sin a), novo y = () )
    }
    public Pose2d fieldToRobotVelocity(@NonNull Pose2d fieldVel) {
        return new Pose2d(
                fieldVel.getX()*Math.sin(-currentPose.getHeadingRadians()) - fieldVel.getY()*Math.cos(-currentPose.getHeadingRadians()),
                fieldVel.getY()*Math.sin(-currentPose.getHeadingRadians()) + fieldVel.getX()*Math.cos(-currentPose.getHeadingRadians())
                ,currentPose.getHeadingDegrees());
    }





}
