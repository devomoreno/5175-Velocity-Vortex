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




@Autonomous(name = "Test Line Shimmy", group = "Sensors")
@Disabled
public class TestAutoLineShimmy extends LinearOpMode {
    ColorSensor colorSensor;
    DcMotor LeftWheel;
    DcMotor RightWheel;
    ModernRoboticsI2cRangeSensor rangeSensor;
    Servo buttonPusher;
    ColorSensor FloorSensorRight;
    ColorSensor FloorSensorLeft;


    @Override
    public void runOpMode() throws InterruptedException{
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
        RightWheel = hardwareMap.dcMotor.get("RightWheel");
        RightWheel.setDirection(DcMotor.Direction.REVERSE);

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

        //Extent button pusher, as to face the flag to get the date of color
            /*This assumes that the color sensor is attatched to the button pusher servo and that
            * when the button pusher is fully extended, the color senseor is facing the flag*/


        //Method used to select which Team Method to route to

        //If Team (isBlue) is true, then route to method BlueTeam



        /*This Method contains all of the slight modifications needed to accomplish the goal of
         * this autonomous when we are on the blue team */

        //Mostly variables carried over from topside of program, others are added in for the same
        //reason that the values above were all variables. Good for logic Checking though



        double distance = 0;
        double ACCEPTED_DISTANCE_CLOSE = 11 ;
        double ACCEPTED_DISTANCE_FAR = 14 ;
        double FLOOR_ACCEPTED_VAL_MIN = 1;
        double FLOOR_ACCEPTED_VAL_MAX = 30;
        double ALLOWED_FROM_WALL_MIN = 11;
        double ALLOWED_FROM_WALL_MAX = 16;


        //based on about where we have put the color sensor It cannot see the floor however can see
        //the tape albeit at a low value. Hence the reason I put the threshold origionally at 1
        //If however for some reason it can see the tiling, and it is consistant (i.e. both sensors
        //get the same value for it) then change the minimum to that plus 1.
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
        if( FloorSensorLeft.alpha() > 0 && FloorSensorRight.alpha() > 0
                && FloorSensorLeft.alpha() == FloorSensorRight.alpha()){
            FLOOR_ACCEPTED_VAL_MIN = (FloorSensorRight.alpha()) +1;

        }
//Sets the range sensor active for the rest of the opmode



        //gets the robot within an accepted distance bu inching it toward the wall. This is done by
        //bakcing up, and sending the left wheel foward frst, then matching the right side to the
        //white line
        distance = rangeSensor.getDistance(DistanceUnit.CM);
        while(ACCEPTED_DISTANCE_FAR < distance){
            LeftWheel.setPower(-.5);
            RightWheel.setPower(-.5);
            sleep(300);
            RightWheel.setPower(0);
            if(!(FLOOR_ACCEPTED_VAL_MIN <= FloorSensorLeft.alpha()
                    && FloorSensorLeft.alpha()<= FLOOR_ACCEPTED_VAL_MAX)){
                LeftWheel.setPower(.2);

            }


            LeftWheel.setPower(0);
            if(!(FLOOR_ACCEPTED_VAL_MIN <= FloorSensorRight.alpha())){
                RightWheel.setPower(.2);
            }
            RightWheel.setPower(0);

            distance = rangeSensor.getDistance(DistanceUnit.CM);

        }

        //When it is within the accepted distance, the robot will stop
        LeftWheel.setPower(0);
        RightWheel.setPower(0);


    }

}