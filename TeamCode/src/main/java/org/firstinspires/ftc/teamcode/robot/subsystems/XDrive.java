package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.constants.XDriveConstants.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.GamepadManager;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.core.util.MathUtils;

import java.util.HashMap;

public class XDrive implements Subsystem {

    private  Telemetry telemetry;
    private static XDrive instance;
    private double reset_angle = 0;
    private DcMotor front_left = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private DcMotor front_right = null;
    private IMU imu;
    private float imuRoll = 0;
    private float imuPitch = 0;
    private float imuYaw = 180;

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
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
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
        telemetry.addData("imu", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
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
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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

    @NonNull
    private IMU.Parameters getParameters() {
        ImuOrientationOnRobot orientation = new ImuOrientationOnRobot() {
            @Override
            public Quaternion imuCoordinateSystemOrientationFromPerspectiveOfRobot() {
                return MathUtils.quartenionToEuler(imuRoll,imuPitch,imuYaw);
            }

            @Override
            public Quaternion imuRotationOffset() {return new Quaternion(1,0,0,0,0);}
            @Override
            public Quaternion angularVelocityTransform() {return new Quaternion(1,0,0,0,0);}
        };
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        return parameters;
    }

    public static synchronized XDrive getInstance() {
        if (instance == null) {
            instance = new XDrive();
        }
        return instance;
    }

}
