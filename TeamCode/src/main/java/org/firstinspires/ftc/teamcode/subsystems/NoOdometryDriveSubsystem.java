//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.geometry.Vector2d;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
///**
// * Drive subsystem that uses mecanum motors and IMU heading only.
// * No odometry wheel position is used; pose (x,y) is always (0,0).
// * This version directly manipulates the motors without Road Runner dependencies.
// *
// * <p>Controls four mecanum drive motors and supports field-centric driving based on the IMU
// * heading. Speeds are commanded in a unitless (-1..1) range where `x` is strafe right, `y` is
// * forward, and `rot` is CCW rotation. This subsystem does not track pose; it only issues motor
// * outputs and exposes a best-effort field-relative velocity estimate computed from commanded
// * speeds.
// */
//public class NoOdometryDriveSubsystem extends SubsystemBase {
//
//   private final Telemetry m_telemetry;
//
//   private final DcMotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;
//   private final IMU m_imu;
//
//    /**
//     * Robot width in inches — distance between left and right wheels. Used for kinematic
//     * calculations when needed.
//     */
//    public static final double WIDTH = 18;
//
//    /**
//     * Robot depth in inches — distance between front and back wheels.
//     */
//    public static final double DEPTH = 18;
//
//   // commanded chassis speeds (unitless, -1..1 style)
//   private double m_xSpeed = 0, m_ySpeed = 0, m_rotSpeed = 0;
//   private boolean m_fieldCentric = false;
//
//   // field-relative velocity estimate (computed from commanded speeds)
//   private Vector2d m_fieldVelocity = new Vector2d(0, 0);
//
//   /**
//    * Constructs a new NoOdometryDriveSubsystem.
//    *
//    * @param hardwareMap the opmode HardwareMap used to obtain motor and sensor instances. The
//    *                    map must contain motors with the names "frontLeftDrive", "backLeftDrive",
//    *                    "backRightDrive" and "frontRightDrive".
//    * @param telemetry   telemetry for sending runtime information to the driver station.
//    */
//   public NoOdometryDriveSubsystem(final HardwareMap hardwareMap, final Telemetry telemetry) {
//       m_telemetry = telemetry;
//
//       m_frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
//       m_backLeft = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
//       m_backRight = hardwareMap.get(DcMotorEx.class, "backRightDrive");
//       m_frontRight = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
//
//       m_frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       m_backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       m_backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       m_frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//       // Standard motor directions for mecanum
//       m_frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//       m_backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//   }
//
//   /**
//    * Periodic update called by the scheduler. This method:
//    * - Reads the current IMU heading
//    * - Applies an optional field-centric transform to the commanded x/y
//    * - Computes and writes motor outputs for mecanum drive
//    * - Updates the exposed field-relative velocity estimate and telemetry
//    */
//   @Override
//   public void periodic() {
//       Rotation2d heading = getHeading();
//
//
//       double x = m_xSpeed;
//       double y = m_ySpeed;
//       double rot = m_rotSpeed;
//
//
//
//       // Mecanum kinematics
//       // y is forward, x is strafe right, rot is CCW rotation
//       double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1.0);
//
//       double frontLeftPower = (y + x + rot) / denominator;
//       double backLeftPower = (y - x + rot) / denominator;
//       double frontRightPower = (y - x - rot) / denominator;
//       double backRightPower = (y + x - rot) / denominator;
//
//       m_frontLeft.setPower(frontLeftPower);
//       m_backLeft.setPower(backLeftPower);
//       m_frontRight.setPower(frontRightPower);
//       m_backRight.setPower(backRightPower);
//
//       // Update the exposed field velocity (for telemetry/other use)
//       m_fieldVelocity = new Vector2d(x, y);
//
//       m_telemetry.update();
//   }
//
//   /**
//    * Set drivetrain speeds (unitless, -1..1 style).
//    *
//    * @param xSpeed      strafe velocity: positive moves the robot to the right (-1..1)
//    * @param ySpeed      forward velocity: positive moves the robot forward (-1..1)
//    * @param rotSpeed    rotational velocity: positive rotates CCW (-1..1)
//    * @param fieldCentric if true, interprets `xSpeed`/`ySpeed` as field-relative velocities using
//    *                     the IMU heading; otherwise they are robot-relative.
//    */
//   public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldCentric) {
//       m_xSpeed = xSpeed;
//       m_ySpeed = ySpeed;
//       m_rotSpeed = rotSpeed;
//       m_fieldCentric = fieldCentric;
//   }
//
//   /**
//    * Returns the last-computed field-relative velocity estimate derived from the commanded
//    * chassis speeds. The components are in the same unitless scale (-1..1) as the commanded speeds.
//    *
//    * @return a Vector2d where `x` is strafe right and `y` is forward (field-relative).
//    */
//   public Vector2d getFieldVelocity() {
//       return m_fieldVelocity;
//   }
//
//
//}
