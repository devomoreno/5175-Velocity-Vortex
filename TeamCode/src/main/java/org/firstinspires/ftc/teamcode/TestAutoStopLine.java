package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * Created by Devin Moreno on 01/06/2017.
 *
 *  */

@Autonomous(name = "Line Test", group = "Sensors")
@Disabled
public class TestAutoStopLine extends LinearOpMode {
    ColorSensor colorSensor;
    DcMotor LeftWheel;
    DcMotor RightWheel;
    ModernRoboticsI2cRangeSensor rangeSensor;
    Servo buttonPusher;
    ColorSensor FloorSensorRight;
    ColorSensor FloorSensorLeft;


    @Override
    public void runOpMode() throws InterruptedException {
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        LeftWheel.setDirection(DcMotor.Direction.REVERSE);

        buttonPusher = hardwareMap.servo.get("Button Pusher");
        buttonPusher.setDirection(Servo.Direction.REVERSE);

        FloorSensorRight = hardwareMap.colorSensor.get("Right color sensor");
        FloorSensorLeft = hardwareMap.colorSensor.get("Left color sensor");
        colorSensor = hardwareMap.colorSensor.get("color sensor");
        I2cAddr colorAddr = I2cAddr.create8bit(0x3c);
        I2cAddr floorRightAddr = I2cAddr.create8bit(0x3a);
        I2cAddr floorLeftAddr = I2cAddr.create8bit(0x3e);

        colorSensor.setI2cAddress(colorAddr);
        FloorSensorLeft.setI2cAddress(floorLeftAddr);
        FloorSensorRight.setI2cAddress(floorRightAddr);





        waitForStart();


        int didCross;
        int didPress;

        double distance = 0;
        double ACCEPTED_DISTANCE_CLOSE = 11;
        double ACCEPTED_DISTANCE_FAR = 14;
        double FLOOR_ACCEPTED_VAL_MIN = 1;
        double FLOOR_ACCEPTED_VAL_MAX = 30;
        double ALLOWED_FROM_WALL_MIN = 11;
        double ALLOWED_FROM_WALL_MAX = 16;

        while(opModeIsActive()){
            telemetry.addData("Right Clear", FloorSensorRight.alpha());
            telemetry.addData("Right Red  ", FloorSensorRight.red());
            telemetry.addData("Right Green", FloorSensorRight.green());
            telemetry.addData("Right Blue ", FloorSensorRight.blue());
            telemetry.addData("Left Clear", FloorSensorLeft.alpha());
            telemetry.addData("Left Red  ", FloorSensorLeft.red());
            telemetry.addData("Left Green",FloorSensorLeft.green());
            telemetry.addData("Left Blue ", FloorSensorLeft.blue());

            telemetry.update();
        }
        //based on about where we have put the color sensor It cannot see the floor however can see
        //the tape albeit at a low value. Hence the reason I put the threshold origionally at 1
        //If however for some reason it can see the tiling, and it is consistant (i.e. both sensors
        //get the same value for it) then change the minimum to that plus 1.

        if (FloorSensorLeft.alpha() > 0 && FloorSensorRight.alpha() > 0
                && FloorSensorLeft.alpha() == FloorSensorRight.alpha()) {
            FLOOR_ACCEPTED_VAL_MIN = (FloorSensorRight.alpha()) + 1;

        }
//Sets the range sensor active for the rest of the opmode



        //move toward the white line
        LeftWheel.setPower(1);
        RightWheel.setPower(1);
        //When the FloorSensor falls within the set parameters (i.e. Sees the white line), stop. If
        //the right side has passed, then reverse it, if it hasnt, make it come forward to
        //align on the white line
        if (FLOOR_ACCEPTED_VAL_MIN <= FloorSensorRight.alpha()
                && FloorSensorRight.alpha() <= FLOOR_ACCEPTED_VAL_MAX) {
            didCross = 1;
        } else {
            didCross = 0;
        }

        if (FLOOR_ACCEPTED_VAL_MIN <= FloorSensorLeft.alpha()
                && FloorSensorLeft.alpha() <= FLOOR_ACCEPTED_VAL_MAX) {
            LeftWheel.setPower(0);

            distance = rangeSensor.getDistance(DistanceUnit.CM);
//If for some reason before the robot corrects itself, it ends up too close and would push the
            //button or hit the beacon, it will correct itself slowly out of the unnacceptable
            //range
            while (distance < ACCEPTED_DISTANCE_CLOSE) {
                LeftWheel.setPower(-.6);
                RightWheel.setPower(-.5);
                sleep(200);

                LeftWheel.setPower(0);

                RightWheel.setPower(.2);

                if (FLOOR_ACCEPTED_VAL_MIN <= FloorSensorRight.alpha()
                        && FloorSensorRight.alpha() <= FLOOR_ACCEPTED_VAL_MAX) {
                    RightWheel.setPower(0);
                }

            }

            switch (didCross) {
                case 1:
                    RightWheel.setPower(-.5);
                    if (FLOOR_ACCEPTED_VAL_MIN <= FloorSensorRight.alpha()
                            && FloorSensorRight.alpha() <= FLOOR_ACCEPTED_VAL_MAX) {
                        RightWheel.setPower(0);
                        break;
                    }
                case 0:
                    RightWheel.setPower(.5);
                    if (FLOOR_ACCEPTED_VAL_MIN <= FloorSensorRight.alpha()
                            && FloorSensorRight.alpha() <= FLOOR_ACCEPTED_VAL_MAX) {
                        RightWheel.setPower(0);
                        break;
                    }
                default:
                    RightWheel.setPower(-.5);
                    sleep(300);
                    RightWheel.setPower(0);
                    break;

            }
        }


//This assumes starting off from the white line, this first peice is meant to correct its alignment


        //Just resets the commands
        LeftWheel.setPower(0);
        RightWheel.setPower(0);


    }
}