package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.GamepadManager;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;

public class Intake implements Subsystem {
    private static Intake instance;
    private Servo servo;
    private Telemetry telemetry;

    private Intake() {
    }

    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = hardwareMap.get(Servo.class, "servoIntake");
        servo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void execute(GamepadManager gamepadManager) {
        servo.setPosition(1);
        telemetry.addData("Intake Servo pos", servo.getPosition());

    }
    public static synchronized Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

}
