package org.eastsideprep.ftc.teamcode.null8103;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.eastsideprep.ftc.teamcode.null8103.drive.SampleMecanumDrive;

@Autonomous(name = "auto pose test 2")
public class Auto_figure8 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;


        //see poses here https://www.desmos.com/calculator/utqpmqcl4v
        //Pose2d startPose = new Pose2d(0, -24, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)
                //.splineTo(new Vector2d(0,0), Math.toRadians(90))
                .splineTo(new Vector2d(visionPose.component1(), visionPose.component2()), visionPose.component3())
                .splineTo(new Vector2d(40, 0), Math.toRadians(270))
                .splineTo(new Vector2d(24, -18), Math.toRadians(180))
                .splineTo(new Vector2d(0, 0), Math.toRadians(135))
                .splineTo(new Vector2d(-24, 16), Math.toRadians(180))
                .splineTo(new Vector2d(-40, 0), Math.toRadians(270))
                .splineTo(new Vector2d(-24, -18), 0)
                .splineTo(new Vector2d(0,0), Math.toRadians(-45))
                .splineTo(new Vector2d(0, -24), Math.toRadians(-90))
                .build();

        drive.followTrajectory(traj);

    }

    Pose2d startPose = new Pose2d(-60, -24, Math.toRadians(180));

    Pose2d visionPose = new Pose2d(-36, -24, Math.toRadians(140));
}
