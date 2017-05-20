package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * Created by William Lord on 12/10/2016.
 */
@Autonomous(name = "Charge to Center Pole", group = "LinearOpMode")
@Disabled
public class AutoChargeToCenterPole extends LinearOpMode {
    private DcMotor LeftWheel;
    private DcMotor RightWheel;

    @Override
    public void runOpMode() throws InterruptedException{
        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");

        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE); // For convenience
        waitForStart();
        LeftWheel.setPower(0);
        RightWheel.setPower(0);
        sleep(10000);
        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        sleep(3500);
        // Park on the center vortex. Hopefully.
        LeftWheel.setPower(0);
        RightWheel.setPower(0);
    }
}
