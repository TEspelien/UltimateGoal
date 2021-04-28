package org.eastsideprep.ftc.teamcode.null8103;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.eastsideprep.ftc.teamcode.null8103.drive.DriveConstants;
import org.eastsideprep.ftc.teamcode.null8103.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "auto pose test")
public class Auto_pose_test extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();

        telemetry.addData("log", "starting");

        Pose2d start = new Pose2d(-60, -24, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        Trajectory trajectory1 = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(0, -24), Math.toRadians(180))
                .build();

        Trajectory trajectory = drive.trajectoryBuilder(startPose)
                .splineTo(visionPose.vec(), visionPose.getHeading())

//                // 0 ring scenario:
                .splineTo(regionA1Pose.vec(), regionA1Pose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(1000);
//                })
//                .splineTo(TunedConstants.secondWobblePose.vec(), TunedConstants.secondWobblePose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(1000);
//                })
//                .splineTo(TunedConstants.regionA2Pose.vec(), TunedConstants.regionA2Pose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(1000);
//                })
//
//                .splineTo(TunedConstants.powerShootingStartPose.vec(), TunedConstants.powerShootingStartPose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(1000);
//                })
//                .splineTo(TunedConstants.powerShootingEndPose.vec(), TunedConstants.powerShootingEndPose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(1000);
//                })
//
//                .splineTo(TunedConstants.endPose.vec(), TunedConstants.endPose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(2000);
//                })
//
//                // 1 ring scenario
//                .splineTo(TunedConstants.regionB1Pose.vec(), TunedConstants.regionB1Pose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(1000);
//                })
//                .splineTo(TunedConstants.secondWobblePose.vec(), TunedConstants.secondWobblePose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(1000);
//                })
//                .splineTo(TunedConstants.regionB2Pose.vec(), TunedConstants.regionB2Pose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(1000);
//                })
//
//                .splineTo(TunedConstants.intakeStartPose.vec(), TunedConstants.intakeStartPose.getHeading())
//                .splineTo(TunedConstants.intakeEndPose.vec(), TunedConstants.intakeEndPose.getHeading(),
//                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(TunedConstants.highShootingPose.vec(), TunedConstants.highShootingPose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(1000);
//                })
//
//                .splineTo(TunedConstants.endPose.vec(), TunedConstants.endPose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(2000);
//                })
//
//                // 4 ring scenario
//                .splineTo(TunedConstants.regionC1Pose.vec(), TunedConstants.regionC1Pose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(1000);
//                })
//                .splineTo(TunedConstants.secondWobblePose.vec(), TunedConstants.secondWobblePose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(1000);
//                })
//                .splineTo(TunedConstants.regionC2Pose.vec(), TunedConstants.regionC2Pose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(1000);
//                })
//
//                .splineTo(TunedConstants.intakeStartPose.vec(), TunedConstants.intakeStartPose.getHeading())
//                .splineTo(TunedConstants.intakeEndPose.vec(), TunedConstants.intakeEndPose.getHeading(),
//                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(TunedConstants.highShootingPose.vec(), TunedConstants.highShootingPose.getHeading())
//                .addDisplacementMarker(() -> {
//                    sleep(1000);
//                })
//                //TODO: decide what to do about the 4th ring
//
//                .splineTo(TunedConstants.endPose.vec(), TunedConstants.endPose.getHeading())

                .build();


        //drive.followTrajectory(trajectory1);
        drive.followTrajectory(trajectory);

        telemetry.addData("log", "done");
    }

    //TODO: tune all of these values

    //this class saves useful positions and constants that are referenced in multiple opmodes

    static double POWER_SHOT_ANGLE = 10; //degrees to turn between each shot

    //might need to strafe instead of turning to hit the powershots...
    static double POWER_SHOT_STRAFE = 8;

    Pose2d startPose = new Pose2d(-60, -24, Math.toRadians(0));

    Pose2d visionPose = new Pose2d(-36, -24, Math.toRadians(140));

    //the first corresponds to the first wobble goal placement, further away from the starting position
    //the second wobble goal is placed a little closer

    Pose2d regionA1Pose = new Pose2d(18, -66, Math.toRadians(30));
    Pose2d regionA2Pose = new Pose2d(14, -62, Math.toRadians(30));

    Pose2d regionB1Pose = new Pose2d(42, -42, Math.toRadians(-120));
    Pose2d regionB2Pose = new Pose2d(38, -38, Math.toRadians(-120));

    Pose2d regionC1Pose = new Pose2d(54, -66, Math.toRadians(-100));
    Pose2d regionC2Pose = new Pose2d(50, -62, Math.toRadians(-100));

    Pose2d secondWobblePose = new Pose2d(-38, -48, Math.toRadians(-90));

    Pose2d intakeStartPose = new Pose2d(-18, -32.5, Math.toRadians(-150));
    Pose2d intakeEndPose = new Pose2d(-30, -39.5, Math.toRadians(-150));

    Pose2d powerShootingStartPose = new Pose2d(0, -28, Math.toRadians(170));
    Pose2d powerShootingEndPose = powerShootingStartPose.plus(new Pose2d(0, 0, POWER_SHOT_ANGLE * 2)); //3 powershots -> 2 angles between them
    Pose2d highShootingPose = new Pose2d(0, -36, Math.toRadians(160));

    Pose2d endPose = new Pose2d(12, -24, Math.toRadians(90));
}
