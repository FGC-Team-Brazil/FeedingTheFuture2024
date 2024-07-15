package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.core.lib.pid.PIDController.Mode.ANGLE;
import static org.firstinspires.ftc.teamcode.robot.constants.IntakeArmyConstants.*;
import static org.firstinspires.ftc.teamcode.robot.constants.GlobalConstants.*;

import org.firstinspires.ftc.teamcode.core.lib.gamepad.GamepadManager;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.lib.pid.PIDController;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;

/**
 * Example subsystem that implements the FGCLib.
 * Look at the example to build your own subsystems
 */
public class IntakeArmy implements Subsystem {
    private static IntakeArmy instance;
    private Telemetry telemetry;
    private CRServo intakeMotorLeft;
    private CRServo intakeMotorRight;
    private DcMotor angleMotor;
    private SmartGamepad operator;
    private org.firstinspires.ftc.teamcode.core.lib.pid.PIDController PIDController;
    Map<String ,Double> angles = new HashMap<>();


    private IntakeArmy() {
    }

    /**
     * Initialize method from the Subsystem Interface
     * @param hardwareMap
     * @param telemetry
     */
    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotorLeft = hardwareMap.get(CRServo.class,INTAKE_MOTOR_LEFT );
        intakeMotorRight = hardwareMap.get(CRServo.class, INTAKE_MOTOR_RIGHT);
        angleMotor = hardwareMap.get(DcMotor.class, ANGLE_MOTOR);
        this.telemetry = telemetry;

        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDController = new PIDController(PID.kP, PID.kI, PID.kD, PID.kF, ANGLE);

        angles.put("Posicao 1",1.0);
        angles.put("Posicao 2",10.0);
        angles.put("Posicao 3",100.0);


        telemetry.addData("IntakeArmy Subsystem", "Initialized");
    }

    /**
     * Execute method from the Subsystem Interface
     * @param gamepadManager
     */
    @Override
    public void execute(GamepadManager gamepadManager) {
        operator = gamepadManager.getOperator();

        telemetry.addData("IntakeArmy Subsystem", "Running");

        PIDController.calculate(TARGET_DEGREE, angleMotor.getCurrentPosition());

        operator.whileButtonLeftBumper()
                .run(() -> {
                    setPowerIntake(-1);
                });

        operator.whileButtonRightBumper()
                .run(() -> {
                    setPowerIntake(1);
                });

        operator.whileLeftTriggerPressed()
                .run(() -> {
                    angleMotor.setPower(-0.5);
                });

        operator.whileRightTriggerPressed()
                .run(() -> {
                    angleMotor.setPower(0.5);
                });

        operator.whileButtonX()
                .run(() -> {
                    PIDController.calculate(angles.get("Posicao 1"),angleMotor.getCurrentPosition());
                });

        operator.whileButtonB()
                .run(() -> {
                    PIDController.calculate(angles.get("Posicao 2"),angleMotor.getCurrentPosition());
                });

        operator.whileButtonA()
                .run(() -> {
                    PIDController.calculate(angles.get("Posicao 3"),angleMotor.getCurrentPosition());
                });

        stop();
    }
    /**
     * Start method from the Subsystem Interface
     */

    public void setPowerIntake(double speed){
        intakeMotorLeft.setPower(speed);
        intakeMotorRight.setPower(speed);
    }



    @Override
    public void start() {

    }

    /**
     * Stop method from the Subsystem Interface
     */
    @Override
    public void stop() {
        angleMotor.setPower(0);
        angleMotor.setPower(0);
    }


    private void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * getInstance is a method used to create a instance of the subsystem.
     * It's not good to have many objects of the same subsystem, so every
     * subsystem in FGCLib will have just one instance, that is created
     * with the getInstance method
     * @return IntakeArmy SingleTon
     */
    public static synchronized IntakeArmy getInstance() {
        if (instance == null) {
            instance = new IntakeArmy();
        }
        return instance;
    }
}