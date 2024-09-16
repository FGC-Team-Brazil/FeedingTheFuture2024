package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.constants.XDriveConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.GamepadManager;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;

import java.util.concurrent.atomic.AtomicReference;


public class XDrive implements Subsystem {

    private  Telemetry telemetry;
    private SmartGamepad driver;
    private static XDrive instance;
    private double resetAngle = 0;
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
    }

    @Override
    public void stop() {
    }

    @Override
    public void execute(GamepadManager gamepadManager) {
        driver = gamepadManager.getDriver();
        drive(gamepadManager.getDriver());
        resetAngle(gamepadManager.getDriver());

    }

    public void drive(SmartGamepad driver) {
        double rotate = driver.getRightStickX()/4;
        AtomicReference<Double> stick_x = new AtomicReference<>(driver.getLeftStickX() * Math.sqrt(Math.pow(1 - Math.abs(rotate), 2) / 2));
        AtomicReference<Double> stick_y = new AtomicReference<>(driver.getLeftStickY() * Math.sqrt(Math.pow(1 - Math.abs(rotate), 2) / 2));
        double theta;
        double pX;
        double pY;

        double gyroAngle = getHeading() * Math.PI / 180;
        if (gyroAngle <= 0) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
        }
        gyroAngle = -1 * gyroAngle;

        driver.whileButtonDPadUp().run(
                () -> {
                    stick_y.set(-0.5);}
        );
        driver.whileButtonDPadDown().run(
                () -> {stick_y.set(0.5);}
        );
        driver.whileButtonDPadLeft().run(
                () -> {stick_x.set(-0.5);}
        );
        driver.whileButtonDPadRight().run(
                () -> {
                    stick_x.set(0.5);}
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
            resetAngle = getHeading() + resetAngle;
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
        heading = heading - resetAngle;
        return heading;
    }

    public static synchronized XDrive getInstance() {
        if (instance == null) {
            instance = new XDrive();
        }
        return instance;
    }

}
