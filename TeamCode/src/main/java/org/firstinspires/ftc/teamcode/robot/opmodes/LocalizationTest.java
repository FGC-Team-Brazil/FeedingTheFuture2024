package org.firstinspires.ftc.teamcode.robot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.lib.Robot;
import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.Pose2d;
import org.firstinspires.ftc.teamcode.robot.subsystems.XDrive;

/**
 * The opmodes folder is where all the opmodes
 * will be written. Just the main TeleOp mode is
 * outside this folder
 * <br><br>
 * This class is an example of opmode that we recommend you to use
 * as template
 */
@TeleOp(name = "Localizationtest", group = "Config")
public class LocalizationTest extends OpMode {
    private final Robot robot = new Robot();

    @Override
    public void init() {
        telemetry.addLine("após iniciar este opmode, ande com o robô por uma distância de 2 metros, arrastando as 4 rodas no chão em linha reta para conseguir configurar a constante de ticks/cm");
        robot.configGamepadManager(gamepad1, gamepad2);
        robot.init(hardwareMap, telemetry);

    }

    @Override
    public void start() {

        XDrive.getInstance().setCurrentPose(new Pose2d(0,0,0));
    }

    @Override
    public void loop() {
        XDrive.getInstance().relativeOdometryUpdate();
        Pose2d actualpose = XDrive.getInstance().getCurrentPose();
        telemetry.addData("Posex",actualpose.getX());
        telemetry.addData("Posey",actualpose.getY());
        telemetry.addData("poseh",actualpose.getHeadingDegrees());
        telemetry.update();

    }

    @Override
    public void stop() {
        robot.stop();
    }
}