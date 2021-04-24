package org.eastsideprep.ftc.teamcode.null8103;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeUnit;

public class RobotHardware {

    HardwareMap hwMap = null;

    //drive motors
    public Motor leftFront = null;
    public Motor rightFront = null;
    public Motor rightBack = null;
    public Motor leftBack = null;

    public Motor front_intake = null;
    public Motor top_intake1 = null;
    public Motor top_intake2 = null;

    public Motor shooter = null;

    public SimpleServo RingPushServo = null;

    public SimpleServo wobblePivot = null;
    public SimpleServo wobbleGrabber = null;

    public SimpleServo leftSideArm = null;
    public SimpleServo rightSideArm = null;

    RevIMU revIMU = null;

    public RobotHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        leftFront = new Motor(hwMap, "LF", Motor.GoBILDA.RPM_312);
        rightFront = new Motor(hwMap, "RF", Motor.GoBILDA.RPM_312);
        rightBack = new Motor(hwMap, "RB", Motor.GoBILDA.RPM_312);
        leftBack = new Motor(hwMap, "LB", Motor.GoBILDA.RPM_312);

        front_intake = new Motor(hwMap, "FrontIntakeMotor", Motor.GoBILDA.RPM_1620);
        top_intake1 = new Motor(hwMap, "IntakeMotor1", Motor.GoBILDA.RPM_1150);
        top_intake2 = new Motor(hwMap, "IntakeMotor2", Motor.GoBILDA.RPM_1150);

        shooter = new Motor(hwMap, "ShooterMotor", 28, 5800);

        RingPushServo = new SimpleServo(hwMap, "RingPushServo");

        wobblePivot = new SimpleServo(hwMap, "WobblePivotServo");
        wobbleGrabber = new SimpleServo(hwMap, "WobbleGrabberServo");



        leftFront.setInverted(true);
        rightFront.setInverted(true);
        rightBack.setInverted(true);
        leftBack.setInverted(true);

        front_intake.setInverted(false);
        top_intake1.setInverted(false);
        top_intake2.setInverted(false);
        shooter.setInverted(true);

        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT); //this throws an error bc there is a bug in ftclib

        //revIMU = new RevIMU(hwMap, "imu");
        //revIMU.init();
    }

    //helpful functions for teleop and auto

    public void runIntake(double p) {
        front_intake.set(p);
        top_intake1.set(p);
        top_intake2.set(p);
    }

    public void stopIntake() {
        front_intake.stopMotor();
        top_intake1.stopMotor();
        top_intake2.stopMotor();
    }

    double wobblePivotLow = 0.3;
    double wobblePivotHigh = 1;
    double wobbleGrabberClosed = 1;
    double wobbleGrabberOpen = 0.2;

    public void lowerOpenWobble() {
        wobbleGrabber.setPosition(wobbleGrabberOpen);
        wobblePivot.setPosition(wobblePivotLow);
    }

    public void closeRaiseWobble() {
        Timing.Timer wobbleTimer = new Timing.Timer(250, TimeUnit.MICROSECONDS);
        wobbleGrabber.setPosition(wobbleGrabberClosed);
        wobbleTimer.start();
        if (wobbleTimer.done()) {
            wobblePivot.setPosition(wobblePivotHigh);
        }
    }
}


