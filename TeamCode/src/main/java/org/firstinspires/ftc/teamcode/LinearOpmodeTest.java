package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
/**
 * Created by d3499 on 9/24/2016.
 */

public class LinearOpmodeTest extends LinearOpMode {
    DcMotor left;
    DcMotor right;

    @Override
    public void runOpMode() throws InterruptedException{
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        left.setPower(1);
        right.setPower(1);
        sleep(2000);

        //sleep is wait function it works in milliseconds

        left.setPower(0);
        right.setPower(-1);
        sleep(5000);

        left.setPower(0);
        right.setPower(0);

        while(opModeIsActive()){
            //Do things
        }
    }
}
