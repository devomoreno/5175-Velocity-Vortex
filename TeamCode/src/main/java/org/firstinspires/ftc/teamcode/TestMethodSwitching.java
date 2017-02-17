package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Devin Moreno on 01/18/2017.
 *A simple test to determine if the implimentation of methods can be applied
 *  */

@Autonomous(name = "Test Method Switching" , group = "Sensors")
@Disabled
public class TestMethodSwitching extends LinearOpMode {
    TouchSensor touch;


    @Override
    public void runOpMode() throws InterruptedException {
        touch = hardwareMap.touchSensor.get("touch");


        waitForStart();


        if (touch.isPressed()){
            MethodTest();
        }


    }

    public void MethodTest(){
        telemetry.addLine("Method Change works");
    }
}