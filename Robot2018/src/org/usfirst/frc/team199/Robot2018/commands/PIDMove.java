package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.SmartDashboardInterface;
import org.usfirst.frc.team199.Robot2018.autonomous.AutoUtils;
import org.usfirst.frc.team199.Robot2018.subsystems.DrivetrainInterface;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Drives the robot a certain target distance using PID. Implements PIDOutput in
 * order to keep move PID centralized in this command.
 */
public class PIDMove extends Command implements PIDOutput {

	private double target;
	private DrivetrainInterface dt;
	private PIDController moveController;
	private PIDSource avg;
	private SmartDashboardInterface sd;
	private double[] point;
	private boolean absolute;

	/**
	 * Constructs this command with a new PIDController. Sets all of the
	 * controller's PID constants based on SD prefs. Sets the controller's PIDSource
	 * to the encoder average object and sets its PIDOutput to this command which
	 * implements PIDOutput's pidWrite() method. Uses either a relative target
	 * specified with `target` or an absolute point with `point`.
	 * 
	 * @param target
	 *            the target distance (in inches) to move to
	 * @param point
	 *            the target point in inches, absolute distance from the starting
	 *            point
	 * @param dt
	 *            the Drivetrain (for actual code) or a DrivetrainInterface (for
	 *            testing)
	 * @param sd
	 *            the SmartDashboard
	 * @param avg
	 *            the PIDSourceAverage of the DT's two Encoders
	 * @param absolute
	 *            whether to use the absolute variable `point` or the relative
	 *            variable `target`
	 */
	public PIDMove(double target, double[] point, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource avg,
			boolean absolute) {
		this.point = point;
		this.target = target;
		this.absolute = absolute;
		this.dt = dt;
		this.avg = avg;
		this.sd = sd;
		if (Robot.dt != null) {
			requires(Robot.dt);
		}

		double maxSpeed = dt.getCurrentMaxSpeed();

		System.out.println("proposed p = " + (dt.getPIDMoveConstant() / maxSpeed));

		double r = Robot.getConst("DistancePidR", 3.0);
		double kP = r / Robot.rmap.getDrivetrainTimeConstant() / maxSpeed;
		double kI = 0;
		double kD = r / maxSpeed;
		double kF = 1 / (maxSpeed * sd.getConst("Default PID Update Time", 0.05)) / maxSpeed;

		moveController = new PIDController(kP, kI, kD, kF, avg, this);
		// {
		/**
		 * Move Velocity: V = sqrt(8TGd) / (R*m) where T = max torque of wheels G = gear
		 * ratio d = distance remaining R = radius of wheels m = mass
		 */
		// @Override
		// protected double calculateFeedForward() {
		// double originalFF = super.calculateFeedForward();
		// double feedForwardConst = dt.getPIDMoveConstant();
		// double error = getError();
		// return (Math.signum(error) * feedForwardConst * Math.sqrt(Math.abs(error)) +
		// originalFF)
		// / dt.getCurrentMaxSpeed();
		// }

		// @Override
		// protected double getContinuousError(double error) {
		// return Math.signum(error) *
		// Math.sqrt(Math.abs(super.getContinuousError(error)));
		// }
		// };
		sd.putData("Move PID", moveController);
	}

	/**
	 * Constructs this command with a new PIDController. Sets all of the
	 * controller's PID constants based on SD prefs. Sets the controller's PIDSource
	 * to the encoder average object and sets its PIDOutput to this command which
	 * implements PIDOutput's pidWrite() method.
	 * 
	 * @param target
	 *            the target distance (in inches) to move to
	 * @param dt
	 *            the Drivetrain (for actual code) or a DrivetrainInterface (for
	 *            testing)
	 * @param sd
	 *            the SmartDashboard
	 * @param avg
	 *            the PIDSourceAverage of the DT's two Encoders
	 */
	public PIDMove(double target, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource avg) {
		this(target, null, dt, sd, avg, false);
	}

	/**
	 * Constructs this command with a new PIDController. Sets all of the
	 * controller's PID constants based on SD prefs. Sets the controller's PIDSource
	 * to the encoder average object and sets its PIDOutput to this command which
	 * implements PIDOutput's pidWrite() method.
	 * 
	 * @param point
	 *            the target point in inches, absolute distance from the starting
	 *            point
	 * @param dt
	 *            the Drivetrain (for actual code) or a DrivetrainInterface (for
	 *            testing)
	 * @param sd
	 *            the SmartDashboard
	 * @param avg
	 *            the PIDSorceAverage of the DT's two Encoders
	 */
	public PIDMove(double[] point, DrivetrainInterface dt, SmartDashboardInterface sd, PIDSource avg) {
		this(0, point, dt, sd, avg, true);
	}

	/**
	 * Called just before this Command runs the first time. Resets the distance
	 * encoders, sets the moveController's settings, and then enables it.
	 */
	@Override
	public void initialize() {
		if (absolute) {
			double dx = point[0] - AutoUtils.state.getX();
			double dy = point[1] - AutoUtils.state.getY();

			double dist = Math.sqrt(dx * dx + dy * dy); // pythagorean theorem to find distance
			this.target = dist;
		}

		dt.resetDistEncs();
		moveController.disable();
		// input is in inches
		moveController.setInputRange(-dt.getCurrentMaxSpeed(), dt.getCurrentMaxSpeed());
		// output in "motor units" (arcade and tank only accept values [-1, 1]
		moveController.setOutputRange(-1.0, 1.0);
		moveController.setContinuous(false);
		moveController.setAbsoluteTolerance(Robot.getConst("MoveTolerance", 0.1));
		System.out.println("move target = " + target);
		moveController.setSetpoint(target);

		moveController.enable();
		// dt.enableVelocityPIDs();
	}

	/**
	 * Called repeatedly when this Command is scheduled to run. This method is empty
	 * bc the moveController runs on a different thread as soon as it is enabled in
	 * initialize().
	 */
	@Override
	protected void execute() {
		System.out.println("Enc Avg Dist: " + avg.pidGet());
		sd.putNumber("Move PID Result", moveController.get());
		sd.putNumber("Move PID Error", moveController.getError());
	}

	/**
	 * Tells this command to terminate when the moveController has reached its
	 * target.
	 * 
	 * @return true if the robot has moved the correct distance, false if not
	 */
	@Override
	protected boolean isFinished() {
		return moveController.onTarget()
				&& Math.abs(dt.getLeftEncRate()) <= Robot.getConst("Maximum Velocity When Stop", 1)
				&& Math.abs(dt.getRightEncRate()) <= Robot.getConst("Maximum Velocity When Stop", 1);
	}

	/**
	 * Called once after isFinished returns true. Disables and frees the
	 * moveController, essentially turning it "off" and "deleting" the thread it was
	 * running on.
	 */
	@Override
	protected void end() {
		moveController.disable();
		System.out.println("End");
		// moveController.free();

		double angle = Math.toRadians(AutoUtils.state.getRot());
		double dist = avg.pidGet();
		// x and y are switched because we are using bearings
		double y = Math.cos(angle) * dist;
		double x = Math.sin(angle) * dist;
		AutoUtils.state.changeX(x);
		AutoUtils.state.changeY(y);

		AutoUtils.state.setRot(dt.getAHRSAngle());

		dt.disableVelocityPIDs();
	}

	/**
	 * Called when another command which requires one or more of the same subsystems
	 * is scheduled to run. Disables and frees the moveController, essentially
	 * turning it "off" and "deleting" the thread it was running on.
	 */
	@Override
	protected void interrupted() {
		end();
	}

	/**
	 * Implementation of PIDOutput's pidWrite method. Sends the moveController's
	 * output speed to the motors as the drive/forward speed in DifferentialDrive's
	 * arcadeDrive method.
	 * 
	 * @param output
	 *            the output drive/forward speed [-1, 1], calculated by the move
	 *            PIDController, to pass in as the drive/forward speed in
	 *            arcadeDrive()
	 */
	@Override
	public void pidWrite(double output) {
		dt.arcadeDrive(output, 0);
		sd.putNumber("Move PID Output", output);
	}
}
