package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test: Drive System Test", group = "Test")
public class MotorTestOpMode extends LinearOpMode {

    private DcMotor frontLeft, backLeft, backRight, frontRight;

    @Override
    public void runOpMode() {
        // Map motors – names must match your Robot Configuration
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRight = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightDrive");

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("This will run a sequence of drive tests:");
        telemetry.addLine("1) Forward  2) Backward");
        telemetry.addLine("3) Strafe Right  (90°)");
        telemetry.addLine("4) Strafe Left   (90°)");
        telemetry.addLine("5) Diagonal (45°) Forward-Right");
        telemetry.addLine("6) Diagonal (45°) Forward-Left");
        telemetry.addLine("7) Turn CW  8) Turn CCW");
        telemetry.addLine("Press PLAY to start.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Forward
            driveAll(0.5, 0, 0, "Forward");
            // Backward
            driveAll(-0.5, 0, 0, "Backward");

            // Strafe right (pure X)
            strafe(0.5, "Strafe Right (90°)");
            // Strafe left (pure X)
            strafe(-0.5, "Strafe Left (90°)");

            // Diagonal forward-right (approx 45°)
            diagonal(0.5, true, "Diagonal Forward-Right (45°)");
            // Diagonal forward-left (approx 45°)
            diagonal(0.5, false, "Diagonal Forward-Left (45°)");

            // Turn in place clockwise
            turn(0.5, "Turn CW");
            // Turn in place counter-clockwise
            turn(-0.5, "Turn CCW");

            stopAll();
        }

        telemetry.addData("Status", "Test complete");
        telemetry.update();
    }

    private void driveAll(double yPower, double xPower, double rotPower, String label) {
        if (!opModeIsActive()) return;

        telemetry.addData("Mode", label);
        telemetry.update();

        // Standard mecanum mixing:
        double fl = yPower + xPower + rotPower;
        double bl = yPower - xPower + rotPower;
        double br = yPower + xPower - rotPower;
        double fr = yPower - xPower - rotPower;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        backRight.setPower(-br);
        frontRight.setPower(-fr);

        sleep(5000);
        stopAll();
        sleep(500);
    }

    private void strafe(double xPower, String label) {
        // yPower = 0, rotPower = 0, only xPower
        driveAll(0, xPower, 0, label);
    }

    private void diagonal(double power, boolean right, String label) {
        if (!opModeIsActive()) return;

        telemetry.addData("Mode", label);
        telemetry.update();

        // For a simple diagonal test, run only two opposite wheels
        if (right) {
            // forward-right: frontLeft + backRight
            frontLeft.setPower(power);
            backRight.setPower(-power);
            backLeft.setPower(0);
            frontRight.setPower(0);
        } else {
            // forward-left: frontRight + backLeft
            frontRight.setPower(power);
            backLeft.setPower(-power);
            frontLeft.setPower(0);
            backRight.setPower(0);
        }

        sleep(5000);
        stopAll();
        sleep(500);
    }

    private void turn(double rotPower, String label) {
        // pure rotation about center
        driveAll(0, 0, rotPower, label);
    }

    private void stopAll() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
}