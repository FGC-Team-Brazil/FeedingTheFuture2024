package org.firstinspires.ftc.teamcode.robot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.lib.Robot;
import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.Pose2d;
import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.TrajectorySequence;
import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.core.lib.autonomousControl.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.core.lib.builders.DrivetrainBuilder;
import org.firstinspires.ftc.teamcode.robot.subsystems.XDrive;


@TeleOp(name = "SampleAuto", group = "SampleAuto")
public class SampleAutoOpMode extends OpMode {
    private final Robot robot = new Robot();
    ElapsedTime timerr = new ElapsedTime();
    TrajectorySequenceRunner runner = new TrajectorySequenceRunner();
    double startTime=0;

    @Override
    public void init() {
        robot.configGamepadManager(gamepad1, gamepad2);
        robot.init(hardwareMap, telemetry); // Don't remove this line

        XDrive.getInstance().setPose2d(new Pose2d(0,0,0));



        TrajectorySequence trajectorySequence= new TrajectorySequenceBuilder()
                .startTrajectoryCourse(new Pose2d(0,10,0),0,90)
                .addBasicCommand(()->{
                    telemetry.addData("boo", "ahh");
                })
                .addCourseSegment(new Pose2d(10,10,90),10)
                .buildCourse()
                .holdPositionForSeconds(2)
                .startTrajectoryCourse(new Pose2d(-10,-10,180),180,90)
                .buildCourse()
                .buildSequence();



        runner.setSequence(trajectorySequence);
    }

    @Override
    public void start() {
        robot.start();
        telemetry.update();
        timerr.reset();
    }

    @Override
    public void loop() {
        runner.execute(timerr.seconds(), startTime);
        //robot.loop();
        Pose2d posicaoDoRobo = XDrive.getInstance().getCurrentPose();
        telemetry.addData("posX", posicaoDoRobo.getX());
        telemetry.addData("posY", posicaoDoRobo.getY());
        telemetry.addData("heading", posicaoDoRobo.getHeadingDegrees());
        telemetry.update();
        startTime = timerr.seconds();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}