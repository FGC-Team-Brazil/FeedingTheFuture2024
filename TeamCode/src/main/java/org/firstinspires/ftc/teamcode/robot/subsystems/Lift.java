package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
public class Lift implements Subsystem {
    private static Lift instance;
    private Telemetry telemetry;
    private DcMotor liftMotorLeft;
    private DcMotor liftMotorRight;
    private TouchSensor limitUp;
    private TouchSensor limitDown;
    private
    Map<String ,Double> liftPidPositions = new HashMap<>();

    private SmartGamepad operator;
    private org.firstinspires.ftc.teamcode.core.lib.pid.PIDController PIDController;

    private Lift() {
    }

    /**
     * Initialize method from the Subsystem Interface
     * @param hardwareMap
     * @param telemetry
     */
    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        liftMotorRight = hardwareMap.get(DcMotor.class, LIFT_MOTOR_RIGHT);
        liftMotorLeft = hardwareMap.get(DcMotor.class, LIFT_MOTOR_LEFT);
        liftMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        liftMotorRight.setDirection(DcMotor.Direction.FORWARD);
        limitUp = hardwareMap.get(TouchSensor.class, LIFT_LIMIT_UP);
        limitDown = hardwareMap.get(TouchSensor.class, LIFT_LIMIT_DOWN);
        this.telemetry = telemetry;

        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDController = new PIDController(PID.kP, PID.kI, PID.kD, PID.kF, POSITION);
        PIDController.setTolerance(100);

        liftPidPositions.put("Baixo",5712.0);
        liftPidPositions.put("Medio",7140.0);
        liftPidPositions.put("Alto", 8568.0);

        telemetry.addData("LIFTSlide Subsystem", "Initialized");
    }

    /**
     * Execute method from the Subsystem Interface
     * @param gamepadManager
     */
    @Override
    public void execute(GamepadManager gamepadManager) {
        operator = gamepadManager.getOperator();

        telemetry.addData("LIFTSlide Subsystem", "Running");

        controlLIFTSlide();

        if(isLimitDown()){liftPidPositions(0);}
        
        if(isLimitUP()){liftPidPositions(0);}

        //Mantem posição após o operador soltar o botão
        liftPidPositions(PIDController.calculate(liftMotorLeft.getCurrentPosition(),liftMotorLeft.getCurrentPosition()));

        operator.whileButtonDPadUp()
                .run(() -> {
              liftPidPositions(PIDController.calculate(liftPidPositions.get("Baixo"), liftMotorLeft.getCurrentPosition()));
                });

        operator.whileButtonDPadDown()
                .run(() -> {
                    liftPidPositions(PIDController.calculate(liftPidPositions.get("Medio"),liftMotorLeft.getCurrentPosition()));
                });

        operator.whileButtonDPadRight()
                .run(() -> {
                    liftPidPositions(PIDController.calculate(liftPidPositions.get("Alto"),liftMotorLeft.getCurrentPosition()));
                });

        

        
        telemetry.addData("MR power", liftMotorRight.getPower());
        telemetry.addData("ML power", liftMotorLeft.getPower());

        telemetry.addData("MR position", liftMotorLeft.getCurrentPosition());
        telemetry.addData("ML position", liftMotorLeft.getCurrentPosition());

        telemetry.addData("PIDcalculate", PIDController.calculate(-4100, liftMotorLeft.getCurrentPosition()));

        
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
        liftMotorRight.setPower(0);
        liftMotorLeft.setPower(0);
    }


    private void liftPidPositions(double speed){
        liftMotorRight.setPower(speed);
        liftMotorLeft.setPower(speed);
    }

    public boolean isLimitUP() {return limitUp.isPressed();}

    public boolean isLimitDown() {
        return limitDown.isPressed();
    }

    private void controlLIFTSlide(){
            liftPidPositions(operator.getLeftStickY());
    }


    /**
     * getInstance is a method used to create a instance of the subsystem.
     * It's not good to have many objects of the same subsystem, so every
     * subsystem in FGCLib will have just one instance, that is created
     * with the getInstance method
     * @return LIFTSlide SingleTon
     */
    public static synchronized Lift getInstance() {
        if (instance == null) {
            instance = new Lift();
        }
        return instance;
    }
}