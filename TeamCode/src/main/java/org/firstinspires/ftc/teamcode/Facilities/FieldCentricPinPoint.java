package org.firstinspires.ftc.teamcode.Facilities;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp (name = "Field Centric PinPoint", group = "Linear")
public class FieldCentricPinPoint extends OpMode {

    GoBildaPinpointDriver pinpoint;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;



    @Override
    public void init(){
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //GoBilda PinPoint Configuration

        //CHANGE position of pinpoints relative to robot
        pinpoint.setOffsets(-84.0,-168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //GoBilda PinPoint Starting Position
        pinpoint.resetPosAndIMU();

        //CHANGE position of robot relative to field
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -900.925, 1601.25, AngleUnit.RADIANS,0);

        telemetry.addData("Status","Initialized");
        telemetry.addData("X offset", pinpoint.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y Offset", pinpoint.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:" , pinpoint.getDeviceVersion());
        telemetry.addData("Device Scalar", pinpoint.getYawScalar());
        telemetry.update();
    }

    //Move robot based on gamepad input using CENTRIC DRIVE
    public void moveRobot(){

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        Pose2D pose = pinpoint.getPosition();
        double heading = pose.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2)-heading);
        double sinAngle = Math.sin((Math.PI / 2)-heading);

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * cosAngle + strafe * sinAngle;

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        frontLeftMotor.setPower(newWheelSpeeds[0]);
        frontRightMotor.setPower(newWheelSpeeds[1]);
        backLeftMotor.setPower(newWheelSpeeds[2]);
        backRightMotor.setPower(newWheelSpeeds[3]);

        telemetry.addData("Robot XPose:", pose.getX(DistanceUnit.MM));
        telemetry.addData("Robot YPose:", pose.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading:", pose.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Forward Speed:", globalForward);
        telemetry.addData("Strafe Speed:", globalStrafe);
        telemetry.addData("Robot Rotate:", rotate);

    }

    @Override
    public void loop(){
        moveRobot();

        Pose2D pose = pinpoint.getPosition();

        telemetry.addData("Robot X:", pinpoint.getPosX(DistanceUnit.MM));
        telemetry.addData("Robot Y:", pinpoint.getPosY(DistanceUnit.MM));
        telemetry.addData("Robot Heading Radians:", pinpoint.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Robot Heading Degrees:", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.update();

        pinpoint.update();
    }

}
