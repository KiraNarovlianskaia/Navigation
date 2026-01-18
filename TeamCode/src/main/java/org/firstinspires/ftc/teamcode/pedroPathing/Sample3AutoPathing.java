package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Curve", group = "Tests")
public class Sample3AutoPathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_ENDPOS,
        DONE
    }

    private PathState pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose(58.68787, 84.45328, Math.toRadians(-90));

    // контрольные точки как обычные Pose
    private final Pose control1 = new Pose(85.39761, 44.93837, 0);
    private final Pose control2 = new Pose(9.733598, 38.075547, 0);

    private PathChain driveStartPosShootPos;
    private PathChain driveShootPosEndPos; // конечный путь — задай сам

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        pathTimer = new Timer();
        opModeTimer = new Timer();

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;

        // установка стартовой позиции локации
        follower.setPose(startPose);

        buildPaths();
    }

    private void buildPaths() {
        // Bézier-кривая от startPose через две контрольные точки до shootPose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, control1, control2, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // пример второго пути — прямой путь от shootPose к другой позиции
        // задай правильные финальные координаты
        Pose endPose = new Pose(10, 100, Math.toRadians(90));
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, control1, control2, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        opModeTimer.resetTimer();
    }

    @Override
    public void loop() {
        // обязательно обновляем follower каждый кадр
        follower.update();

        // FSM для перехода по путям
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);  // запускаем первый путь
                pathState = PathState.SHOOT_PRELOAD;
                pathTimer.resetTimer();
                break;

            case SHOOT_PRELOAD:
                // ждем окончания пути + пауза для выстрела
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(driveShootPosEndPos, true);
                    pathState = PathState.DRIVE_SHOOTPOS_ENDPOS;
                    pathTimer.resetTimer();
                }
                break;

            case DRIVE_SHOOTPOS_ENDPOS:
                if (!follower.isBusy()) {
                    pathState = PathState.DONE;
                }
                break;

            case DONE:
                telemetry.addLine("Finished all paths");
                break;
        }

        telemetry.addData("State", pathState.toString());
        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
