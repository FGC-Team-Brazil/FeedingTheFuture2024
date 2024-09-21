package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.constants.OuttakeConstants.*;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.GamepadManager;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;

public class Outtake  implements Subsystem {
    private static Outtake instance;
    private Telemetry telemetry;
    private Servo servoRight;
    private Servo servoLeft;
    private ColorSensor colorSensor;

    private Outtake() {}
    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.servoRight = hardwareMap.get(Servo.class, SERVO_RIGHT);
        this.servoLeft = hardwareMap.get(Servo.class, SERVO_LEFT);
        this.colorSensor = hardwareMap.get(ColorSensor.class, COLOR_SENSOR);
        this.telemetry = telemetry;

        this.colorSensor.enableLed(true);

        this.telemetry.addData("Outtake Subsystem", "Initialized");
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void execute(GamepadManager gamepadManager) {
        SmartGamepad operator = gamepadManager.getOperator();

        operator.whileButtonLeftBumper()
                .run(() -> {
                    servoRight.setPosition(-1);
                    servoLeft.setPosition(-1);
                });

        operator.whileButtonRightBumper()
                .run(() -> {
                    servoRight.setPosition(1);
                    servoLeft.setPosition(1);
                });

        if(colorSensor.green() > 100) {
            operator.ledSetColorContinuous(SmartGamepad.Color.RED);
        } else if (colorSensor.blue() > 100) {
            operator.ledSetColorContinuous(SmartGamepad.Color.BLUE);
        } else {
            operator.ledSetColorContinuous(SmartGamepad.Color.WHITE);
        }

        this.telemetry.addData("Outtake Color Sensor", "Red: %d, Green: %d, Blue: %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());

    }

    public static synchronized Outtake getInstance() {
        if (instance == null) {
            instance = new Outtake();
        }
        return instance;
    }
}
