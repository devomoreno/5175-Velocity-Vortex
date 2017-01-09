package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by William Lord on 11/12/2016.
 * Last modified by William Lord on 12/10/2016.
 */
@Autonomous(name = "AutoPushBallCenterParkTimeBlue", group = "LinearOpMode")
@Disabled
public class AutoPushBallCenterParkSensor extends LinearOpMode {
    DcMotor left;
    DcMotor right;
    ColorSensor csGo;

    @Override
    public void runOpMode() throws InterruptedException{
        left = hardwareMap.dcMotor.get("LeftWheel");
        right = hardwareMap.dcMotor.get("RightWheel");
        csGo = hardwareMap.colorSensor.get("ColourSensor");

        right.setDirection(DcMotorSimple.Direction.REVERSE); // For convenience
        waitForStart();

        left.setPower(0);
        right.setPower(0);
        sleep(10000);
        left.setPower(1);
        right.setPower(1);
        sleep(6000);
        left.setPower(-1);
        right.setPower(-1);
        // Park on the center vortex. Hopefully.
        // TODO: Get true color values for base
        /**
         * Infinite loops
         * Infinite loops
         * I hope this doesn't loop forever
         * This is where I make infinite loops
         * I love to make infinite loops
         * The magic and the mystery
         * of infinite loops
         * Little long long ints
         * This machine creates little long long ints
         * It turns constant expression strings
         * Into little long long ints
         * This is where i make unsigned ints
         * I sure love to make unsigned ints
         * I turn little char strings
         * Into unsigned ints
         * I turn little char strings
         * Into unsigned ints
         */
        while(!(180 < csGo.red() && csGo.red() < 200 && 150 < csGo.green() && csGo.green() < 170 && csGo.blue() < 20)){
            sleep(1); // keep going until we see this colour. is there a better way to do this?
        }
        left.setPower(0);
        right.setPower(0);
    }
}