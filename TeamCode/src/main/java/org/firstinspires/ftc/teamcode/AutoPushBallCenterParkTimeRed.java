package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * Created by William Lord on 12/10/2016.
 */
@Autonomous(name = "AutoPushBallCenterParkTimeRed", group = "LinearOpMode")
@Disabled
public class AutoPushBallCenterParkTimeRed extends LinearOpMode {
    DcMotor left;
    DcMotor right;

    @Override
    public void runOpMode() throws InterruptedException{
        left = hardwareMap.dcMotor.get("LeftWheel");
        right = hardwareMap.dcMotor.get("RightWheel");

        right.setDirection(DcMotorSimple.Direction.REVERSE); // For convenience
        waitForStart();

        // TODO: use sensors instead. get actual times for this in the meantime
        left.setPower(0);
        right.setPower(0);
        sleep(10000);
        left.setPower(1);
        right.setPower(1);
        sleep(4000);
        left.setPower(-1);
        right.setPower(-1);
        sleep(1000);
        left.setPower(-0.15);
        sleep(2000);
        // Park on the center vortex. Hopefully.
        left.setPower(0);
        right.setPower(0);
    }
}
