package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="Button Pusher", group="Module")
@Disabled

//import all hardware going to be used
public class ModuleButtonPusher2 extends OpMode {
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg

    Servo buttonPusher;
    double PUSHER_MIN = 0;
    double PUSHER_MAX = 1;
    double servoDelta = .005;
    double ServoPosition;

    public ModuleButtonPusher2(){}

    @Override
            public void init(){
        buttonPusher = hardwareMap.servo.get("Button Pusher");




        //map items here and set rules ( reference any vector baseline or basic programs)

    }
    @Override
            public void loop(){



    //set all the driver and gamepad options. this is where the program goes.
        if(gamepad1.right_bumper){
            if(ServoPosition != 0){
                ServoPosition -= servoDelta;
            }

        }


        if(gamepad1.right_trigger>0){
            if(ServoPosition != PUSHER_MAX){
                ServoPosition += servoDelta;
            }
        }

        ServoPosition = Range.clip(ServoPosition, PUSHER_MIN, PUSHER_MAX);

        buttonPusher.setPosition(ServoPosition);
    }
    @Override
        public void stop(){
        //this is, to my knowledge all that is needed for this public void
    }
    //This is for the driving scale as far as this point it is ok without modification

}
