package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "LLAMA then Park")
@Disabled
/**
 * Created by d3499 on 9/24/2016.
 */

public class AutoSpinnerLLAMA extends LinearOpMode {

    private DcMotor LLAMA;
    private DcMotor Spinner;



    @Override
    public void runOpMode() throws InterruptedException{

        Spinner =hardwareMap.dcMotor.get("Spinner");

        LLAMA = hardwareMap.dcMotor.get("LLAMA");

        LLAMA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LLAMA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LLAMA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();
        int encoderShoot = 0;
        int forwardMove = 0;
        int turnLeast = 0;
        int turnMost = 0;
        int ontoRamp= 0;


        //FIXME Get actual values
        int maxPosition = 12;



        Spinner.setPower(1);

        sleep(5000);
        if(LLAMA.getCurrentPosition() > maxPosition+1){
            LLAMA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        LLAMA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        LLAMA.setTargetPosition(encoderShoot);
        LLAMA.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        LLAMA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turn to align to corner vortex

        LLAMA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }

