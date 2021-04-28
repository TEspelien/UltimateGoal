package org.eastsideprep.ftc.teamcode.null8103;

import com.acmerobotics.dashboard.FtcDashboard;
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

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    Trajectory trajectory = drive.trajectoryBuilder(TunedConstants.startPose)
            .splineTo(TunedConstants.visionPose.vec(), TunedConstants.visionPose.getHeading())

            // 0 ring scenario:
            .splineTo(TunedConstants.regionA1Pose.vec(), TunedConstants.regionA1Pose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(1000);
            })
            .splineTo(TunedConstants.secondWobblePose.vec(), TunedConstants.secondWobblePose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(1000);
            })
            .splineTo(TunedConstants.regionA2Pose.vec(), TunedConstants.regionA2Pose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(1000);
            })

            .splineTo(TunedConstants.powerShootingStartPose.vec(), TunedConstants.powerShootingStartPose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(1000);
            })
            .splineTo(TunedConstants.powerShootingEndPose.vec(), TunedConstants.powerShootingEndPose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(1000);
            })

            .splineTo(TunedConstants.endPose.vec(), TunedConstants.endPose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(2000);
            })

            // 1 ring scenario
            .splineTo(TunedConstants.regionB1Pose.vec(), TunedConstants.regionB1Pose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(1000);
            })
            .splineTo(TunedConstants.secondWobblePose.vec(), TunedConstants.secondWobblePose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(1000);
            })
            .splineTo(TunedConstants.regionB2Pose.vec(), TunedConstants.regionB2Pose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(1000);
            })

            .splineTo(TunedConstants.intakeStartPose.vec(), TunedConstants.intakeStartPose.getHeading())
            .splineTo(TunedConstants.intakeEndPose.vec(), TunedConstants.intakeEndPose.getHeading(),
                    SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .splineTo(TunedConstants.highShootingPose.vec(), TunedConstants.highShootingPose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(1000);
            })

            .splineTo(TunedConstants.endPose.vec(), TunedConstants.endPose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(2000);
            })

            // 4 ring scenario
            .splineTo(TunedConstants.regionC1Pose.vec(), TunedConstants.regionC1Pose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(1000);
            })
            .splineTo(TunedConstants.secondWobblePose.vec(), TunedConstants.secondWobblePose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(1000);
            })
            .splineTo(TunedConstants.regionC2Pose.vec(), TunedConstants.regionC2Pose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(1000);
            })

            .splineTo(TunedConstants.intakeStartPose.vec(), TunedConstants.intakeStartPose.getHeading())
            .splineTo(TunedConstants.intakeEndPose.vec(), TunedConstants.intakeEndPose.getHeading(),
                    SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .splineTo(TunedConstants.highShootingPose.vec(), TunedConstants.highShootingPose.getHeading())
            .addDisplacementMarker(() -> {
                sleep(1000);
            })
            //TODO: decide what to do about the 4th ring

            .splineTo(TunedConstants.endPose.vec(), TunedConstants.endPose.getHeading())

            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //possible solution?
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();

        telemetry.addData("log", "starting");
        drive.followTrajectory(trajectory);
        telemetry.addData("log", "done");
    }
}
