package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * Created by William Lord on 12/10/2016.
 */
@Autonomous(name = "Charge to Corner vortex", group = "LinearOpMode")

public class AutoChargeToCornerVortex extends LinearOpMode {
    private DcMotor LeftWheel;
    private DcMotor RightWheel;

    @Override
    public void runOpMode() throws InterruptedException{
        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");

        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE); // For convenience
        waitForStart();

        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        sleep(1000);
        LeftWheel.setPower(-.5);
        RightWheel.setPower(.5);
        sleep(1750);
        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        sleep(1250);
        // Park on the center vortex. Hopefully.
        LeftWheel.setPower(0);
        RightWheel.setPower(0);
    }
}
