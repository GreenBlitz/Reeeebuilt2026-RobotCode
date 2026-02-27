package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.joysticks.SmartJoystick;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.constants.flywheel.FlywheelConstants;
import frc.utils.calibration.sysid.SysIdCalibrator;
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
			Logger.recordOutput(getLogPath() + "/velocityControl", "BangBang");
			motor.applyRequest(velocityBangBangRequest.withSetPoint(velocity));
		} else {
			Logger.recordOutput(getLogPath() + "/velocityControl", "VelocityVoltage");
			motor.applyRequest(velocityVoltageRequest.withSetPoint(velocity));
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

	public void update() {
		motor.updateSimulation();
		motor.updateInputs(velocitySignal, voltageSignal, currentSignal);
		Logger.recordOutput(getLogPath() + "/targetVelocity", targetVelocity);
	}

	public void applyCalibrationsBindings(SmartJoystick joystick) {
		joystick.POV_LEFT.onTrue(getCommandBuilder().setTargetVelocity(Rotation2d.fromRotations(1)));
		joystick.POV_UP.onTrue(getCommandBuilder().setTargetVelocity(Rotation2d.fromRotations(2)));
		joystick.POV_DOWN.onTrue(getCommandBuilder().stop());
		sysIdCalibrator.setAllButtonsForCalibration(joystick);
	}

}
