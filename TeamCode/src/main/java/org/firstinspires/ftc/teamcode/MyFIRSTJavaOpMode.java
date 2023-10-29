package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Jason AS Test: MyFIRSTJavaOpMode", group="JAS")
public class MyFIRSTJavaOpMode extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor rearLeftMotor = hardwareMap.dcMotor.get("rearLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor rearRightMotor = hardwareMap.dcMotor.get("rearRightMotor");

        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);



        waitForStart();

        if (isStopRequested()) return;

        double robotOriented = 1;
        double slowingDown = 4;

        while (opModeIsActive()) {


            if(gamepad1.b){
                slowingDown = 4;
            }
            else{
                slowingDown = 1;
            }
            double y = (-gamepad1.left_stick_y / slowingDown) + (gamepad1.right_stick_y / -2);
            double x = (gamepad1.left_stick_x / slowingDown) + (gamepad1.right_stick_x / 2);
            double rx = (gamepad1.left_trigger - gamepad1.right_trigger) / -2;


            //if(gamepad1.b){
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // }
            // else if(gamepad1.y){
            // frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            // frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            // rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            // }

            if (gamepad1.back){
                imu.resetYaw();
            }


            if(gamepad1.y){
                robotOriented = 1;
            }
            else if(gamepad1.x){
                robotOriented = 0;
            }

            double botHeading;
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) * robotOriented;


            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double rearLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double rearRightPower = (rotY + rotX - rx) / denominator;

            double setPoint;
            double feedback = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error;
            double rotationSpeed;

            if(gamepad1.dpad_left){
                setPoint = 90;
            }
            else if(gamepad1.dpad_up){
                setPoint = 0;
            }
            else if(gamepad1.dpad_right){
                setPoint = -90;
            }
            else if(gamepad1.dpad_down){
                setPoint = 180;
            }
            else{
                setPoint = 0;

            }

            if(gamepad1.dpad_left & gamepad1.dpad_up & gamepad1.dpad_right & gamepad1.dpad_down){
                error = setPoint - feedback;

                if(error > 180){
                    error = error - 360;
                }
                else if(error < -180){
                    error = error + 360;
                }
                rotationSpeed = 0.2 * error;
                if(rotationSpeed > 0.4){
                    rotationSpeed = 0.4;
                }
                else if(rotationSpeed < -0.4){
                    rotationSpeed = -0.4;
                }
            }
            else{
                rotationSpeed = 0;
            }

            if((Math.abs(frontLeftPower) < 0.05) & (Math.abs(rearLeftPower) < 0.05) & (Math.abs(frontRightPower) < 0.05) & (Math.abs(rearRightPower) < 0.05))
            {
                frontLeftMotor.setPower(0);
                rearLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                rearRightMotor.setPower(0);
            }
            else
            {
                frontLeftMotor.setPower(frontLeftPower + rotationSpeed);
                rearLeftMotor.setPower(rearLeftPower + rotationSpeed);
                frontRightMotor.setPower(frontRightPower - rotationSpeed);
                rearRightMotor.setPower(rearRightPower - rotationSpeed);
            }

            if((botHeading == 0) & (robotOriented == 1)){
                telemetry.addLine("Gyro broken, please reset robot");
            }

            telemetry.addData("Robot current yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}

