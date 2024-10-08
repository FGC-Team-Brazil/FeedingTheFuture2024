package org.firstinspires.ftc.teamcode.core.lib.gamepad;

import static org.checkerframework.checker.units.UnitsTools.g;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * SmartController class is a extension of the GamepadButton class.
 * It contains other functions of the gamepads, as rumble.
 * <br>
 * Always use this class instead of the GamepadButton.
 */
public class SmartGamepad extends GamepadButton {
    public Gamepad gamepad;

    public enum Color{
        RED,
        BLUE,
        GREEN,
        WHITE,

    }

    public SmartGamepad(Gamepad gamepad) {
        super(gamepad);
        this.gamepad = gamepad;
    }

    public void rumbleTimer(int milliseconds) {
        gamepad.rumble(1, 1, milliseconds);
    }

    public void rumbleBlips(int counts) {
        gamepad.rumbleBlips(counts);
    }

    public void rumbleEnable(){
        gamepad.rumble(1, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
    }

    public void rumbleStop() {
        gamepad.stopRumble();
    }
    public void ledSetColorContinuous(Color color ){
        double[] rgb = getRGBValues(color);
        gamepad.setLedColor(rgb[0],rgb[1],rgb[2], Gamepad.LED_DURATION_CONTINUOUS);
    }
    public void ledSetColor(Color color, int durationMS){
        double[] rgb = getRGBValues(color);
        gamepad.setLedColor(rgb[0], rgb[1], rgb[2], durationMS);
    }
    private double[] getRGBValues(Color color) {
        double r = 0;
        double g = 0;
        double b = 0;

        switch (color) {
            case RED:
                r = 1;
                break;
            case GREEN:
                g = 1;
                break;
            case BLUE:
                b = 1;
                break;
            case WHITE:
                r = 1;
                g = 1;
                b = 1;
                break;
        }


        return new double[]{r, g, b};
    }
    public void ledSetRainbow(int durationMs){
        Gamepad.LedEffect rgbEffect = new Gamepad.LedEffect.Builder()
                .addStep(1, 0, 0, durationMs)
                .addStep(0, 1, 0, durationMs)
                .addStep(0, 0, 1, durationMs)
                .addStep(1, 1, 1, durationMs)
                .build();
        gamepad.runLedEffect(rgbEffect);
}}

