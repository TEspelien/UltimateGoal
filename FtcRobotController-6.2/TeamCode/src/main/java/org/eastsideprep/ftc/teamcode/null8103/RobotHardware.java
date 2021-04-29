package org.eastsideprep.ftc.teamcode.null8103;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.R;

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

    public SimpleServo leftBlocker = null;
    public SimpleServo rightBlocker = null;

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

        leftBlocker = new SimpleServo(hwMap, "LeftRingBlockerServo");
        rightBlocker = new SimpleServo(hwMap, "RightRingBlockerServo");

        leftFront.setInverted(true);
        rightFront.setInverted(true);
        rightBack.setInverted(true);
        leftBack.setInverted(true);

        front_intake.setInverted(false);
        top_intake1.setInverted(false);
        top_intake2.setInverted(false);
        shooter.setInverted(false);

        shooter.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //shooter.setRunMode(Motor.RunMode.VelocityControl);
        //shooter.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT); //this throws an error bc there is a bug in ftclib

        //shooter.setVeloCoefficients(1, 0.0, 0.0);
        //start by increasing P
        //then slowly increase D to reduce oscillations
        //dont touch I

        //kV should start at 1/max_velo
        //shooter.setFeedforwardCoefficients(0.0, 0.0001);

        //revIMU = new RevIMU(hwMap, "imu");
        //revIMU.init();
    }

    //helpful functions for teleop and auto

//    public void forwards(int pos, double power){
//        leftFront.setTargetPosition(pos);
//        rightFront.setTargetPosition(-pos);
//        rightBack.setTargetPosition(-pos);
//        leftBack.setTargetPosition(pos);
//
//        leftFront.set(power);
//        rightFront.set(-power);
//        rightBack.set(-power);
//        leftBack.set(power);
//    }
    public void forwards(int pos, double power){
        int LFT = leftFront.getCurrentPosition()+pos;
        int LBT = leftBack.getCurrentPosition()-pos;
        int RFT = rightFront.getCurrentPosition()-pos;
        int RBT = rightBack.getCurrentPosition()+pos;

        leftFront.setTargetPosition(LFT);
        rightFront.setTargetPosition(LBT);
        rightBack.setTargetPosition(RFT);
        leftBack.setTargetPosition(RBT);

        leftFront.set(power);
        rightFront.set(-power);
        rightBack.set(-power);
        leftBack.set(power);

        boolean done = false;
        boolean LF = false;
        boolean LB = false;
        boolean RF = false;
        boolean RB = false;

        while(done == false) {
            if(leftFront.getCurrentPosition() >= LFT) {
                leftFront.stopMotor();
                LF=true;
            }
            if(leftBack.getCurrentPosition() >= LBT) {
                leftBack.stopMotor();
                LB=true;
            }
            if(rightFront.getCurrentPosition() <= RFT) {
                rightFront.stopMotor();
                RF=true;
            }
            if(rightBack.getCurrentPosition() <= RBT) {
                rightBack.stopMotor();
                RB=true;
            }
            if (LF == true && LB == true && RF == true && RB == true) {
                done = true;
            }
        }
    }


    public void turnLeft(int pos, double power){
        leftFront.setTargetPosition(-pos);
        rightFront.setTargetPosition(-pos);
        rightBack.setTargetPosition(-pos);
        leftBack.setTargetPosition(-pos);


        leftFront.set(-power);
        rightFront.set(-power);
        rightBack.set(-power);
        leftBack.set(-power);
    }

    public void turnRight(int pos, double power){
        leftFront.setTargetPosition(pos);
        rightFront.setTargetPosition(pos);
        rightBack.setTargetPosition(pos);
        leftBack.setTargetPosition(pos);

        leftFront.set(power);
        rightFront.set(power);
        rightBack.set(power);
        leftBack.set(power);
    }

    //both of these shoot methods are targeting the high goal
    public void shoot3Rings() {
        Timing.Timer shooterTimer = new Timing.Timer(4000, TimeUnit.MILLISECONDS);
        shooter.set(0.95);
        shooterTimer.start();

        if (shooterTimer.currentTime() > 2000) {
            pushRing();
        } else if (shooterTimer.currentTime() > 3000) {
            pushRing();
        } else if (shooterTimer.done()) {
            pushRing();
        }

        shooter.stopMotor();
    }

    public void shoot1Ring() {
        Timing.Timer shooterTimer = new Timing.Timer(2000, TimeUnit.MILLISECONDS);
        shooter.set(0.95);
        shooterTimer.start();

        if (shooterTimer.done()) {
            pushRing();
        }

        shooter.stopMotor();
    }

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
    double wobbleGrabberClosed = 0.2;
    double wobbleGrabberOpen = 1;

    public void lowerOpenWobble() {
        wobbleGrabber.setPosition(wobbleGrabberOpen);
        wobblePivot.setPosition(wobblePivotLow);
    }

    public void closeRaiseWobble() {
        Timing.Timer wobbleTimer = new Timing.Timer(250, TimeUnit.MILLISECONDS);
        wobbleGrabber.setPosition(wobbleGrabberClosed);
        wobbleTimer.start();
        if (wobbleTimer.done()) {
            wobblePivot.setPosition(wobblePivotHigh);
        }
    }

    double ringPusherLow = 0.52; //not touching ring
    double ringPusherHigh = 0.39; //ring pushed into shooter

    public void pushRing() {
        Timing.Timer ringPusherTimer = new Timing.Timer(1000, TimeUnit.MILLISECONDS);
        RingPushServo.setPosition(ringPusherHigh);
        ringPusherTimer.start();
//        while (!ringPusherTimer.done()) {
//
//        }
        RingPushServo.setPosition(ringPusherLow);
    }
}


