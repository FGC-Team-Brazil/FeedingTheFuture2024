package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;


import static org.firstinspires.ftc.teamcode.robot.constants.ConveyorConstants.*;

import org.firstinspires.ftc.teamcode.core.lib.gamepad.GamepadManager;
import org.firstinspires.ftc.teamcode.core.lib.gamepad.SmartGamepad;
import org.firstinspires.ftc.teamcode.core.lib.interfaces.Subsystem;

public class ConveyorSubsystem implements Subsystem {
    private static ConveyorSubsystem instance;

    private CRServo boxWheelServo;
    private CRServo leftouttakeServo;
    private CRServo rightouttakeServo;
    /*private ColorSensor outtakeSensor;*/

    private SmartGamepad operator;
    private Telemetry telemetry;

    private int[] foodARGBColor = {0,0,0,0}; //Placeholder
    private int[] energyARGBColor = {0,0,0,0}; //Placeholder
    private int[] waterARGBColor = {0,0,0,0}; //Placeholder


    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        boxWheelServo = hardwareMap.get(CRServo.class, BOX_EXPANSION_SERVO);
        leftouttakeServo = hardwareMap.get(CRServo.class, LEFT_OUTTAKE_SERVO);
        rightouttakeServo = hardwareMap.get(CRServo.class, RIGHT_OUTTAKE_SERVO);
        /*outtakeSensor = hardwareMap.get(ColorSensor.class, OUTTAKE_SENSOR);*/

        boxWheelServo.setDirection(CRServo.Direction.FORWARD);
        leftouttakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightouttakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;

        telemetry.addData("Conveyor Subsystem", "Initialized");
    }

    @Override
    public void start() {
        telemetry.addData("Box Expansion", "Retracted");
    }

    @Override
    public void execute(GamepadManager gamepadManager) {
        operator = gamepadManager.getOperator();

        controlOuttake(operator);

        /*telemetry.addData("Outtake Game Piece", gamePieceColorChecker());*/
    }

    @Override
    public void stop() {

    }

    private void controlOuttake(SmartGamepad operator) {
        if (operator.isRightTriggerPressed()){
            leftouttakeServo.setPower(operator.getRightTrigger());
            rightouttakeServo.setPower(operator.getRightTrigger());
            boxWheelServo.setPower(operator.getRightTrigger());
        } else if(operator.isLeftTriggerPressed()) {
            leftouttakeServo.setPower(-operator.getLeftTrigger());
            rightouttakeServo.setPower(-operator.getLeftTrigger());
        }
    }

    /*private String gamePieceColorChecker() {
        if (outtakeSensor.red() > foodARGBColor[0]-30 && outtakeSensor.green() < foodARGBColor[0]+30
        && outtakeSensor.green() > foodARGBColor[1]-30 && outtakeSensor.green() < foodARGBColor[1]+30
        && outtakeSensor.blue() > foodARGBColor[2]-30 && outtakeSensor.green() < foodARGBColor[2]+30){
            return "Food";
        } else if (outtakeSensor.red() > energyARGBColor[0]-30 && outtakeSensor.green() < energyARGBColor[0]+30
                && outtakeSensor.green() > energyARGBColor[1]-30 && outtakeSensor.green() < energyARGBColor[1]+30
                && outtakeSensor.blue() > energyARGBColor[2]-30 && outtakeSensor.green() < energyARGBColor[2]+30){
            return "Energy";
        } else if (outtakeSensor.red() > waterARGBColor[0]-30 && outtakeSensor.green() < waterARGBColor[0]+30
                && outtakeSensor.green() > waterARGBColor[1]-30 && outtakeSensor.green() < waterARGBColor[1]+30
                && outtakeSensor.blue() > waterARGBColor[2]-30 && outtakeSensor.green() < waterARGBColor[2]+30){
            return "Water";
        } else {
            return "Without GP";
        }
    }*/

    public static synchronized ConveyorSubsystem getInstance() {
        if (instance == null) {
            instance = new ConveyorSubsystem();
        }
        return instance;
    }
}
