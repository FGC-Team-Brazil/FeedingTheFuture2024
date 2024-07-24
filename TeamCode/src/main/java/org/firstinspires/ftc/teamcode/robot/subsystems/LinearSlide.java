package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.core.lib.pid.PIDController.Mode.ANGLE;
import static org.firstinspires.ftc.teamcode.robot.constants.LinearSlideConstants.*;
import static org.firstinspires.ftc.teamcode.robot.constants.GlobalConstants.*;

import org.firstinspires.ftc.teamcode.core.lib.gamepad.GamepadManager;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.lib.pid.PIDController;

import java.util.HashMap;
import java.util.Map;

/**
 * Example subsystem that implements the FGCLib.
 * Look at the example to build your own subsystems
 */
public class LinearSlide implements Subsystem {
    private static LinearSlide instance;
    private Telemetry telemetry;
    private DcMotor linearMotorLeft;
    private DcMotor linearMotorRight;
    Map<String ,Double> linearPidPositions = new HashMap<>();

    private SmartGamepad operator;
    private org.firstinspires.ftc.teamcode.core.lib.pid.PIDController PIDController;

    private LinearSlide() {
    }

    /**
     * Initialize method from the Subsystem Interface
     * @param hardwareMap
     * @param telemetry
     */
    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        linearMotorRight = hardwareMap.get(DcMotor.class, LINEAR_MOTOR_RIGHT);
        linearMotorLeft = hardwareMap.get(DcMotor.class, LINEAR_MOTOR_LEFT);
        this.telemetry = telemetry;

        linearMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDController = new PIDController(PID.kP, PID.kI, PID.kD, PID.kF, ANGLE);

        linearPidPositions.put("Pontua Nexus Baixo",50.0);
        linearPidPositions.put("Pontua Nexus Medio",100.0);
        linearPidPositions.put("Pontua Nexus Alto", 150.0);

        telemetry.addData("LinearSlide Subsystem", "Initialized");
    }

    /**
     * Execute method from the Subsystem Interface
     * @param gamepadManager
     */
    @Override
    public void execute(GamepadManager gamepadManager) {
        operator = gamepadManager.getOperator();

        telemetry.addData("IntakeArm Subsystem", "Running");


        operator.whileButtonLeftBumper()
                .run(() -> {
                    linearSlideMotors(-0.5);
                });

        operator.whileButtonRightBumper()
                .run(() -> {
                    linearSlideMotors(0.5);
                });



        operator.whileRightTriggerPressed()
                .run(() -> {
                    PIDController.calculate(linearPidPositions.get("Pontua Nexus Baixo"),linearMotorLeft.getCurrentPosition());
                }, () -> {
                    PIDController.calculate(linearPidPositions.get("Pontua Nexus Baixo"),linearMotorRight.getCurrentPosition());
                });

        operator.whileRightTriggerPressed()
                .run(() -> {
                    PIDController.calculate(linearPidPositions.get("Pontua Nexus Medio"),linearMotorLeft.getCurrentPosition());
                }, () -> {
                    PIDController.calculate(linearPidPositions.get("Pontua Nexus Medio"),linearMotorRight.getCurrentPosition());
                });

        operator.whileRightTriggerPressed()
                .run(() -> {
                    PIDController.calculate(linearPidPositions.get("Pontua Nexus Alto"),linearMotorLeft.getCurrentPosition());
                }, () -> {
                    PIDController.calculate(linearPidPositions.get("Pontua Nexus Alto"),linearMotorRight.getCurrentPosition());
                });


        stop();
    }

    /**
     * Start method from the Subsystem Interface
     */
    @Override
    public void start() {

    }

    /**
     * Stop method from the Subsystem Interface
     */
    @Override
    public void stop() {
        linearMotorRight.setPower(0);
        linearMotorLeft.setPower(0);
    }

    private void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void linearSlideMotors(double speed){
        linearMotorRight.setPower(speed);
        linearMotorLeft.setPower(speed);
    }


    private void rpmLinearSlide(){

    }


    /**
     * getInstance is a method used to create a instance of the subsystem.
     * It's not good to have many objects of the same subsystem, so every
     * subsystem in FGCLib will have just one instance, that is created
     * with the getInstance method
     * @return LinearSlide SingleTon
     */
    public static synchronized LinearSlide getInstance() {
        if (instance == null) {
            instance = new LinearSlide();
        }
        return instance;
    }
}