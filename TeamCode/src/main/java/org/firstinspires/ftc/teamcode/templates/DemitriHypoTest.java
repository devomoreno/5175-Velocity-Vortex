package org.firstinspires.ftc.teamcode.templates;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

//import all hardware going to be used
public class DemitriHypoTest extends OpMode {
    //name Dcmotors and for purpose of the program
    //ex:  Dcmotor Greg
    DcMotor Treadfrontleft;
    DcMotor Treadfrontright;
    DcMotor Treadbackleft;
    DcMotor Treadbackright;
    CRServo  ArcScrew;
    public DemitriHypoTest(){}

    @Override
            public void init(){
        Treadfrontleft=hardwareMap.dcMotor.get("fLeftTread");
        Treadfrontright=hardwareMap.dcMotor.get("fRightTread");
        Treadbackleft=hardwareMap.dcMotor.get("bLeftTread");
        Treadbackright=hardwareMap.dcMotor.get("bRightTread");
        Treadfrontleft.setDirection(DcMotor.Direction.REVERSE);
        Treadbackleft.setDirection(DcMotor.Direction.REVERSE);
        ArcScrew=hardwareMap.crservo.get("arcservo");
        //map itemshere and set rules ( reference any vector baseline or basic programs)

    }
    @Override
            public void loop(){
        float fleft=gamepad1.left_stick_y;
        float fright=gamepad1.right_stick_y;
        float bleft=gamepad1.left_stick_y;
        float bright=gamepad1.right_stick_y;

        fright= Range.clip(fright, -1, 1);
        fleft = Range.clip(fleft, -1, 1);
        bright= Range.clip(bright, -1, 1);
        bleft = Range.clip(bleft, -1, 1);

        fright = (float)scaleInput(fright);
        fleft =  (float)scaleInput(fleft);
        bright = (float)scaleInput(bright);
        bleft =  (float)scaleInput(bleft);

        Treadfrontleft.setPower(fleft);
        Treadfrontright.setPower(fright);
        Treadbackleft.setPower(bleft);
        Treadbackright.setPower(bright);
    //set all the driver and gamepad options. this is where the program goes.
    }
    @Override
        public void stop(){
        //this is, to my knowledge all that is needed for this public void
    }
    //This is for the driving scale as far as this point it is ok without modification
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.00, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
                index = -index;
            } else if (index > 16) {
                index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
