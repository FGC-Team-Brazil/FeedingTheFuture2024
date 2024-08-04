package org.firstinspires.ftc.teamcode.core.lib.builders;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.GamepadManager;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;


/**
 * XDrivetrainBuilder is a helper class that assists on the creation
 * of a Drivetrain Subsystem.
 * It contains some usual boilerplate for creating a subsystem.
 * Use it when creating X-Drive drivetrains with four motors.
 * <p>
 * Caution: This class don't support tankDrive
 * drivetrains
 */
public class XDrivetrainBuilder implements Subsystem {
    private static XDrivetrainBuilder instance;
    private DcMotorSimple.Direction motorRightForwardDirection;
    private DcMotorSimple.Direction motorLeftForwardDirection;
    private DcMotorSimple.Direction motorRightBackwardDirection;
    private DcMotorSimple.Direction motorLeftBackwardDirection;
    private String motorLeftForwardName;
    private String motorRightForwardName;
    private String motorRightBackwardName;
    private String motorLeftBackwardName;
    private static boolean isDriveInverted;
    private DcMotor motorRightForward;
    private DcMotor motorLeftForward;
    private DcMotor motorRightBackward;
    private DcMotor motorLeftBackward;
    private Telemetry telemetry;
    private SmartGamepad driver;

    private static byte invertDriveHandler = 0;

    private double reset_angle = 0;
    private static BNO055IMU imu;
    private static BNO055IMU.Parameters imuParameters;

    private XDrivetrainBuilder() {
    }

    public static XDrivetrainBuilder build(@NonNull String motorRightBackwardName, @NonNull String motorLeftBackwardName, @NonNull String motorRightForwardName, @NonNull String motorLeftForwardName, boolean isInverted) {
        getInstance();

        isDriveInverted = isInverted;

        instance.motorRightBackwardName = motorRightBackwardName;
        instance.motorLeftBackwardName = motorLeftBackwardName;
        instance.motorRightForwardName = motorRightForwardName;
        instance.motorLeftForwardName = motorLeftForwardName;
        instance.motorLeftForwardDirection = isDriveInverted
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        instance.motorRightForwardDirection = isDriveInverted
                ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        instance.motorLeftBackwardDirection = isDriveInverted
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        instance.motorRightBackwardDirection = isDriveInverted
                ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;

        return instance;
    }

    public static XDrivetrainBuilder build(@NonNull String[] xDriveMotorsNames, boolean isInverted) {
        getInstance();

        instance.motorRightBackwardName = xDriveMotorsNames[0];
        instance.motorLeftBackwardName = xDriveMotorsNames[1];
        instance.motorRightForwardName = xDriveMotorsNames[2];
        instance.motorLeftForwardName = xDriveMotorsNames[3];
        instance.motorLeftForwardDirection = isInverted
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        instance.motorRightForwardDirection = isInverted
                ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        instance.motorLeftBackwardDirection = isInverted
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        instance.motorRightBackwardDirection = isInverted
                ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;

        imuParameters = new BNO055IMU.Parameters();
        return instance;
    }

    /**
     * Initialize method from Subsystem Interface
     */
    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        motorLeftForward = hardwareMap.get(DcMotor.class, motorLeftForwardName);
        motorRightForward = hardwareMap.get(DcMotor.class, motorRightForwardName);
        motorLeftBackward = hardwareMap.get(DcMotor.class, motorLeftBackwardName);
        motorRightBackward = hardwareMap.get(DcMotor.class, motorRightBackwardName);
        motorLeftForward.setDirection(motorLeftForwardDirection);
        motorRightForward.setDirection(motorRightForwardDirection);
        motorLeftBackward.setDirection(motorLeftBackwardDirection);
        motorRightBackward.setDirection(motorRightBackwardDirection);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
    }

    /**
     * Start method from Subsystem Interface
     */
    @Override
    public void start() {

    }

    /**
     * Execute method from Subsystem Interface
     * @param gamepadConfig
     */
    @Override
    public void execute(GamepadManager gamepadConfig) {
        driver = gamepadConfig.getDriver();

        telemetry.addData("XDrivetrainBuilder Subsystem", "Running");
        arcadeDrive(-driver.getLeftStickY(), -driver.getRightStickX(), driver);

        if(driver.isButtonX() && invertDriveHandler == 0){
            invertDrive(true);
            invertDriveHandler = 1;
        } else if(!driver.isButtonX() && invertDriveHandler == 1){
            invertDriveHandler = 2;
        } else if(driver.isButtonX() && invertDriveHandler == 2){
            invertDrive(false);
            invertDriveHandler = 0;
        }
    }

    /**
     * Stop method from Subsystem Interface
     */
    @Override
    public void stop() {

    }

    /**
     * Default Method for drive control. This method is for
     * tank drivetrains.
     * @param xSpeed
     * @param zRotation
     * @param driver
     */
    public void arcadeDrive(double xSpeed, double zRotation, SmartGamepad driver) {
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

        motorLeftForward.setPower(Py - rotate);
        motorLeftBackward.setPower(Px - rotate);
        motorRightBackward.setPower(Py + rotate);
        motorRightBackward.setPower(Px + rotate);
    }

    public static void invertDrive(boolean isInverted) {
        instance.motorLeftForwardDirection = isInverted
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        instance.motorRightForwardDirection = isInverted
                ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        instance.motorLeftBackwardDirection = isInverted
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        instance.motorRightBackwardDirection = isInverted
                ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
    }

    public void resetAngle(SmartGamepad driver){
        if(driver.isButtonA()){
            reset_angle = getHeading() + reset_angle;
        }
    }

    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        if(heading < -180) {
            heading = heading + 360;
        }
        else if(heading > 180){
            heading = heading - 360;
        }
        heading = heading - reset_angle;
        return heading;
    }

    // Set Methods
    public static void setIMUMode(BNO055IMU.SensorMode mode) {imuParameters.mode = mode; updateParameters();}
    public static void setIMUAngleUnit(BNO055IMU.AngleUnit unit) {imuParameters.angleUnit = unit; updateParameters();}
    public static void setIMULoggingEnabled(boolean enabled) {imuParameters.loggingEnabled = enabled; updateParameters();}
    public static void setIMUAccelUnit(BNO055IMU.AccelUnit unit) {imuParameters.accelUnit = unit; updateParameters();}
    public static void setInverted(boolean isInverted) {isDriveInverted = isInverted; updateParameters();}
    public static void updateParameters() {imu.initialize(imuParameters);}

    // Get Methods
    public static BNO055IMU.SensorMode getIMUMode() {return imuParameters.mode;}
    public static BNO055IMU.AngleUnit getIMUAngleUnit() {return imuParameters.angleUnit;}
    public static boolean getIMULoggingEnabled() {return imuParameters.loggingEnabled;}
    public static BNO055IMU.AccelUnit getIMUAccelUnit() {return imuParameters.accelUnit;}

    /**
     * getInstance is a method used to create a instance of the subsystem.
     * It's not good to have many objects of the same subsystem, so every
     * subsystem in FGCLib will have just one instance, that is created
     * with the getInstance method
     */
    public static void getInstance() {
        if (instance == null) {
            synchronized (XDrivetrainBuilder.class) {
                if (instance == null) {
                    instance = new XDrivetrainBuilder();
                }
            }
        }
    }

}
