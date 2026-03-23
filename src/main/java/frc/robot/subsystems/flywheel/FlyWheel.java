package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.joysticks.SmartJoystick;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.constants.flywheel.FlywheelConstants;
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

public class FlyWheel extends GBSubsystem {

	private final IRequest<Rotation2d> velocityVoltageRequest;
	private final IRequest<Rotation2d> velocityBangBangRequest;
	private final IRequest<Double> voltageRequest;

	private final InputSignal<Rotation2d> velocitySignal;
	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Double> currentSignal;

	private final ControllableMotor motor;

	private final FlyWheelCommandBuilder flyWheelCommandBuilder;

	private final SysIdCalibrator sysIdCalibrator;

	private Rotation2d targetVelocity;

	private Rotation2d previousVelocity;
	private Rotation2d acceleration;

	public FlyWheel(
		String logPath,
		IRequest<Rotation2d> velocityVoltageRequest,
		IRequest<Rotation2d> velocityBangBangRequest,
		IRequest<Double> voltageRequest,
		InputSignal<Rotation2d> velocitySignal,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		ControllableMotor motor
	) {
		super(logPath);
		this.velocityVoltageRequest = velocityVoltageRequest;
		this.velocityBangBangRequest = velocityBangBangRequest;
		this.voltageRequest = voltageRequest;
		this.velocitySignal = velocitySignal;
		this.voltageSignal = voltageSignal;
		this.currentSignal = currentSignal;
		this.motor = motor;
		this.flyWheelCommandBuilder = new FlyWheelCommandBuilder(this);
		this.sysIdCalibrator = new SysIdCalibrator(motor.getSysidConfigInfo(), this, this::setVoltage);
		this.targetVelocity = Rotation2d.kZero;
		this.previousVelocity = Rotation2d.kZero;
		this.acceleration = Rotation2d.kZero;
		setDefaultCommand(getCommandBuilder().stop());
	}

	public FlyWheelCommandBuilder getCommandBuilder() {
		return flyWheelCommandBuilder;
	}

	public void setTargetVelocity(Rotation2d velocity) {
		targetVelocity = velocity;
		if (
			Math.abs(velocity.getDegrees() - velocitySignal.getLatestValue().getDegrees())
				> FlywheelConstants.MIN_ERROR_TO_APPLY_BANG_BANG_CONTROL_RPS.getDegrees()
		) {
			motor.applyRequest(velocityBangBangRequest.withSetPoint(velocity));
			Logger.recordOutput(getLogPath() + "/usedControl", "Bang Bang");
		} else {
			motor.applyRequest(velocityVoltageRequest.withSetPoint(velocity));
			Logger.recordOutput(getLogPath() + "/usedControl", "PID");
		}
	}

	public void setVoltage(double voltage) {
		motor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	public Rotation2d getVelocity() {
		return velocitySignal.getLatestValue();
	}

	public double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public double getCurrent() {
		return currentSignal.getLatestValue();
	}

	public boolean isAtVelocity(Rotation2d targetVelocity, Rotation2d tolerance) {
		return velocitySignal.isNear(targetVelocity, tolerance);
	}

	public void stop() {
		motor.stop();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public Rotation2d getAcceleration() {
		return acceleration;
	}

	public void update() {
		motor.updateSimulation();
		motor.updateInputs(velocitySignal, voltageSignal, currentSignal);

		double dt = TimeUtil.getLatestCycleTimeSeconds();

		double currentVelocityRotations = velocitySignal.getLatestValue().getRotations();
		double previousVelocityRotations = previousVelocity.getRotations();
		Rotation2d dv = Rotation2d.fromRotations(previousVelocityRotations - currentVelocityRotations);

		Logger.recordOutput(getLogPath() + "/dt", dt);
		Logger.recordOutput(getLogPath() + "/dv", dv);
		if (dt > 0) {
			acceleration = Rotation2d.fromRotations(dv.getRotations() / dt);
		}
		previousVelocity = velocitySignal.getLatestValue();

		Logger.recordOutput(getLogPath() + "/targetVelocity", targetVelocity);
		Logger.recordOutput(getLogPath() + "/acceleration", acceleration.getRotations());
	}

	public void applyCalibrationsBindings(SmartJoystick joystick) {
		joystick.POV_LEFT.onTrue(getCommandBuilder().setTargetVelocity(Rotation2d.fromRotations(1)));
		joystick.POV_UP.onTrue(getCommandBuilder().setTargetVelocity(Rotation2d.fromRotations(2)));
		joystick.POV_DOWN.onTrue(getCommandBuilder().stop());
		sysIdCalibrator.setAllButtonsForCalibration(joystick);
	}

}
