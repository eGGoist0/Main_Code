package org.firstinspires.ftc.teamcode.Facilities;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveSet {
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    IMU imu;

    public void init(HardwareMap hwMap){

        frontLeftMotor = hwMap.get(DcMotor.class,"frontLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hwMap.get(DcMotor.class,"backLeftMotor");
        backRightMotor = hwMap.get(DcMotor.class,"backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class,"imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void driveFieldRelative(double forward, double right, double rotate) {
        // Converting to Polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Rotation to pointing of robot
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Converting to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        this.drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        //preserving proportions of motors as they should be, and confining it <1.0
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        //maxSpeed* - CAN CHANGE - decreasing Power and Speed of motors for tests
        frontLeftMotor.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightMotor.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftMotor.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightMotor.setPower(maxSpeed * (backRightPower / maxPower));
    }
}