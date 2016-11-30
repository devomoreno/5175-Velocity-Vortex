package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by William Lord on 11/12/2016.
 */
@TeleOp(name = "AutoButtonPusherCenterPark", group = "LinearOpMode")
@Disabled
public class AutoButtonPusherCenterPark extends LinearOpMode {
    DcMotor left;
    DcMotor right;
    Servo lPusher;
    Servo rPusher;

    @Override
    public void runOpMode() throws InterruptedException{
        left = hardwareMap.dcMotor.get("LeftWheel");
        right = hardwareMap.dcMotor.get("RightWheel");
        lPusher = hardwareMap.servo.get("LeftServo");
        rPusher = hardwareMap.servo.get("RightServo");

        right.setDirection(DcMotorSimple.Direction.REVERSE); // For convenience
        waitForStart();

        // TODO: use sensors instead. get actual times for this in the meantime
        left.setPower(1);
        right.setPower(1);
        sleep(2000);
        right.setPower(0); // Turn left.
        sleep(1000);
        right.setPower(1);
        sleep(1000);
        left.setPower(0);
        right.setPower(0);
        sleep(500); // Give us some time to brake
        // I hope we're at the beacon. TODO: Colour sensing. We need a colour sensor for that first.
        lPusher.setPosition(1.00);  // For now assume we're pressing the left button.
        sleep(1000);
        lPusher.setPosition(0.00); // Back it up?
        sleep(1000); //Back. It up.
        left.setPower(-1); //0000000); // BACKING UP!
        right.setPower(-1);
        sleep(1000);
        left.setPower(0);
        right.setPower(0); // Park on the center vortex. Hopefully.


        //while(opModeIsActive()){
            // Do no things. Use this if we need a loop.
        //}

        left.setPower(0);
        right.setPower(0);
    }
}
