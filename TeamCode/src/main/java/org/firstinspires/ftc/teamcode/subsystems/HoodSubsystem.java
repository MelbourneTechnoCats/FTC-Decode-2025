import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HoodSubsystem {
	private DcMotor hoodMotor;

	// How many encoder ticks is "one part" (tune this)
	private static final int STEP_TICKS = 50;

	// Default power when moving to a position (tune this)
	private static final double POSITION_POWER = 0.4;

	public HoodSubsystem(HardwareMap hardwareMap) {
		// Change "hoodMotor" to your actual motor name in the configuration
		hoodMotor = hardwareMap.get(DcMotor.class, "hoodMotor");
		hoodMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		// Reverse direction if needed
		// hoodMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		// Encoder + position mode setup
		hoodMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		hoodMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void setPower(double power) {
		hoodMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		hoodMotor.setPower(power);
	}

	public void stop() {
		hoodMotor.setPower(0);
	}

	public DcMotor getMotor() {
		return hoodMotor;
	}

	// ---- NEW COMMAND-LIKE METHODS ----

	/** Move the hood up by one step (STEP_TICKS). */
	public void incrementUp() {
		int currentPos = hoodMotor.getCurrentPosition();
		int targetPos = currentPos + STEP_TICKS;
		moveToPosition(targetPos);
	}

	/** Move the hood down by one step (STEP_TICKS). */
	public void incrementDown() {
		int currentPos = hoodMotor.getCurrentPosition();
		int targetPos = currentPos - STEP_TICKS;
		moveToPosition(targetPos);
	}

	/** Set an absolute position in encoder ticks. */
	public void setPosition(int targetTicks) {
		moveToPosition(targetTicks);
	}

	/** Internal helper for encoder-based movement. */
	private void moveToPosition(int targetPos) {
		hoodMotor.setTargetPosition(targetPos);
		hoodMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		hoodMotor.setPower(POSITION_POWER);
	}
}