package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Roller extends GBSubsystem {

	protected final ControllableMotor roller;

	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Double> currentSignal;
	private final InputSignal<Rotation2d> positionSignal;
	private final InputSignal<Rotation2d> velocitySignal;
	private final IRequest<Double> voltageRequest;
	private final IRequest<Rotation2d> velocityRequest;
	private final RollerCommandsBuilder commandsBuilder;
	private Rotation2d targetPosition;
	private Rotation2d targetVelocity;

	public Roller(
		String logPath,
		ControllableMotor roller,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Rotation2d> velocitySignal,
		IRequest<Double> voltageRequest,
		IRequest<Rotation2d> velocityRequest
	) {
		super(logPath);
		this.roller = roller;
		this.voltageSignal = voltageSignal;
		this.currentSignal = currentSignal;
		this.positionSignal = positionSignal;
		this.velocitySignal = velocitySignal;
		this.voltageRequest = voltageRequest;
		this.commandsBuilder = new RollerCommandsBuilder(this);
		this.roller.resetPosition(Rotation2d.fromRotations(0));
		this.targetPosition = Rotation2d.fromRotations(0);
		this.velocityRequest = velocityRequest;
		this.targetVelocity = Rotation2d.fromRotations(0);
	}

	public RollerCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public void updateTargetPosition(Rotation2d targetPosition) {
		this.targetPosition = targetPosition;
	}

	public void updateTargetVelocity(Rotation2d velocity) {
		this.targetVelocity = velocity;
	}

	public void setVoltage(double voltage) {
		roller.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	public void setPower(double power) {
		roller.setPower(power);
	}

	public void setVelocity(Rotation2d velocity) {
		roller.applyRequest(velocityRequest.withSetPoint(velocity));
	}

	public void stop() {
		roller.stop();
	}

	public void setBrake(boolean brake) {
		roller.setBrake(brake);
	}

	public Double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public Double getCurrent() {
		return currentSignal.getLatestValue();
	}

	public Rotation2d getPosition() {
		return positionSignal.getLatestValue();
	}

	public boolean isAtPosition(Rotation2d position, Rotation2d tolerance) {
		return positionSignal.isNear(position, tolerance);
	}

	public boolean isBehindPosition(Rotation2d position) {
		return positionSignal.isLess(position);
	}

	public boolean isPastPosition(Rotation2d position) {
		return positionSignal.isGreater(position);
	}

	public boolean isPastTargetPosition() {
		return isPastPosition(targetPosition);
	}

	public boolean isBehindTargetPosition() {
		return isBehindPosition(targetPosition);
	}

	public void update() {
		roller.updateSimulation();
		roller.updateInputs(voltageSignal, currentSignal, positionSignal, velocitySignal);
		Logger.recordOutput(getLogPath() + "/PositionTarget", targetPosition);
		Logger.recordOutput(getLogPath() + "/VelocityTarget", targetVelocity);
	}

}
