package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.core.lib.pid.PIDController.Mode.POSITION;
import static org.firstinspires.ftc.teamcode.robot.constants.LiftConstants.*;

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
        linearMotorRight = hardwareMap.get(DcMotor.class, LIFT_MOTOR_RIGHT);
        linearMotorLeft = hardwareMap.get(DcMotor.class, LIFT_MOTOR_LEFT);
        linearMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        linearMotorRight.setDirection(DcMotor.Direction.FORWARD);
        this.telemetry = telemetry;

        linearMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDController = new PIDController(PID.kP, PID.kI, PID.kD, PID.kF, POSITION);
        PIDController.setTolerance(100);

        linearPidPositions.put("Baixo",5712.0);
        linearPidPositions.put("Medio",7140.0);
        linearPidPositions.put("Alto", 8568.0);

        telemetry.addData("LinearSlide Subsystem", "Initialized");
    }

    /**
     * Execute method from the Subsystem Interface
     * @param gamepadManager
     */
    @Override
    public void execute(GamepadManager gamepadManager) {
        stop();
        operator = gamepadManager.getOperator();

        telemetry.addData("LinearSlide Subsystem", "Running");

        controlLinearSlide();

        //Mantem posição após o operador soltar o botão
        linearSlideMotors(PIDController.calculate(linearMotorLeft.getCurrentPosition(),linearMotorLeft.getCurrentPosition()));

        operator.whileButtonDPadUp()
                .run(() -> {
              linearSlideMotors(PIDController.calculate(linearPidPositions.get("Baixo"), linearMotorLeft.getCurrentPosition()));
                });

        operator.whileButtonDPadDown()
                .run(() -> {
                    linearSlideMotors(PIDController.calculate(linearPidPositions.get("Medio"),linearMotorLeft.getCurrentPosition()));
                });

        operator.whileButtonDPadRight()
                .run(() -> {
                    linearSlideMotors(PIDController.calculate(linearPidPositions.get("Alto"),linearMotorLeft.getCurrentPosition()));
                });

        telemetry.addData("MR power", linearMotorRight.getPower());
        telemetry.addData("ML power", linearMotorLeft.getPower());

        telemetry.addData("MR position", linearMotorLeft.getCurrentPosition());
        telemetry.addData("ML position", linearMotorLeft.getCurrentPosition());

        telemetry.addData("PIDcalculate", PIDController.calculate(-4100, linearMotorLeft.getCurrentPosition()));

        linearSlideMotors(operator.getLeftStickY());
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


    private void linearSlideMotors(double speed){
        linearMotorRight.setPower(speed);
        linearMotorLeft.setPower(speed);
    }


    private void controlLinearSlide(){
            linearSlideMotors(operator.getLeftStickY());
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