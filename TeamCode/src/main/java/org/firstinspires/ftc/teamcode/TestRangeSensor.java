package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Range Sensor Only", group="Test")

//import all hardware going to be used
public class TestRangeSensor extends OpMode {
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg
    ModernRoboticsI2cRangeSensor rangeSensor;
    public TestRangeSensor(){}

    @Override
            public void init(){
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        //map items here and set rules ( reference any vector baseline or basic programs)

    }
    @Override
            public void loop(){
    //set all the driver and gamepad options. this is where the program goes.

        telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", rangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
    }
    @Override
        public void stop(){
        //this is, to my knowledge all that is needed for this public void
    }
    //This is for the driving scale as far as this point it is ok without modification

}
