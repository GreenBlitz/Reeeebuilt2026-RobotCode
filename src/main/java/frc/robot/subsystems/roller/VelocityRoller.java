package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.interfaces.ControllableMotor;
import org.littletonrobotics.junction.Logger;

public class VelocityRoller extends Roller {

	private final InputSignal<Rotation2d> velocitySignal;
	private final IRequest<Rotation2d> velocityRequest;
	private final VelocityRollerCommandBuilder commandsBuilder;

	public VelocityRoller(
		String logPath,
		ControllableMotor roller,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Rotation2d> velocitySignal,
		IRequest<Double> voltageRequest,
		IRequest<Rotation2d> velocityRequest
	) {
		super(logPath, roller, voltageSignal, currentSignal, positionSignal, voltageRequest);
		this.velocitySignal = velocitySignal;
		this.velocityRequest = velocityRequest;
		this.commandsBuilder = new VelocityRollerCommandBuilder(this);
		setDefaultCommand(commandsBuilder.stop());
	}

	@Override
	public VelocityRollerCommandBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public Rotation2d getVelocity() {
		return velocitySignal.getLatestValue();
	}

	public void setVelocity(Rotation2d targetVelocity) {
		velocityRequest.withSetPoint(targetVelocity);
		motor.applyRequest(velocityRequest);
	}

	@Override
	public void stop() {
		super.stop();
		velocityRequest.withSetPoint(Rotation2d.kZero);
	}

	@Override
	public void update() {
		super.update();
		Logger.recordOutput(getLogPath() + "/TargetVelocity", getVelocity());
		motor.updateInputs(velocitySignal);
	}

}
