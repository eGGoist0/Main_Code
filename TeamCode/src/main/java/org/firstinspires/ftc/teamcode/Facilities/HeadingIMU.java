package org.firstinspires.ftc.teamcode.Facilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp 
public class HeadingIMU extends OpMode {

    TestIMU test = new TestIMU();

    @Override
    public void init() {
        test.init(hardwareMap);
    }

    @Override
    public void loop(){
        telemetry.addData("Heading", test.getHeading(AngleUnit.DEGREES));
    }
}
