package org.eastsideprep.ftc.teamcode.null8103;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class TunedConstants {

    //TODO: tune all of these values

    //this class saves useful positions and constants that are referenced in multiple opmodes

    static double POWER_SHOT_ANGLE = 10; //degrees to turn between each shot

    //might need to strafe instead of turning to hit the powershots...
    static double POWER_SHOT_STRAFE = 8;

    static Pose2d startPose = new Pose2d(-60, -24, Math.toRadians(180));

    static Pose2d visionPose = new Pose2d(-36, -24, Math.toRadians(140));

    //the first corresponds to the first wobble goal placement, further away from the starting position
    //the second wobble goal is placed a little closer

    static Pose2d regionA1Pose = new Pose2d(18, -66, Math.toRadians(30));
    static Pose2d regionA2Pose = new Pose2d(14, -62, Math.toRadians(30));

    static Pose2d regionB1Pose = new Pose2d(42, -42, Math.toRadians(-120));
    static Pose2d regionB2Pose = new Pose2d(38, -38, Math.toRadians(-120));

    static Pose2d regionC1Pose = new Pose2d(54, -66, Math.toRadians(-100));
    static Pose2d regionC2Pose = new Pose2d(50, -62, Math.toRadians(-100));

    static Pose2d secondWobblePose = new Pose2d(-38, -48, Math.toRadians(-90));

    static Pose2d intakeStartPose = new Pose2d(-18, -32.5, Math.toRadians(-150));
    static Pose2d intakeEndPose = new Pose2d(-30, -39.5, Math.toRadians(-150));

    static Pose2d powerShootingStartPose = new Pose2d(0, -28, Math.toRadians(170));
    static Pose2d powerShootingEndPose = powerShootingStartPose.plus(new Pose2d(0, 0, POWER_SHOT_ANGLE * 2)); //3 powershots -> 2 angles between them
    static Pose2d highShootingPose = new Pose2d(0, -36, Math.toRadians(160));

    static Pose2d endPose = new Pose2d(12, -24, Math.toRadians(90));
}
