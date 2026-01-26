package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.*;
import org.littletonrobotics.junction.Logger;

public class VelocityRoller extends Roller {
	InputSignal<Rotation2d> velocitySignal;
	private IRequest<Rotation2d> velocityRequest;
	private VelocityRollerCommandBuilder commandsBuilder;
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

	public Rotation2d getVelocity(){
		return velocityRequest.getSetPoint();
	}

	public void setVelocity(Rotation2d targetVelocity) {
		velocityRequest.withSetPoint(targetVelocity);
		roller.applyRequest(velocityRequest);
	}

	@Override
	public VelocityRollerCommandBuilder getCommandsBuilder(){
		return commandsBuilder;
	}

	@Override
	public void stop(){
		super.stop();
		velocityRequest.withSetPoint(Rotation2d.kZero);
	}

	@Override
	public void update() {
		super.update();
		Logger.recordOutput(getLogPath() + "/TargetVelocity", getVelocity());
		roller.updateInputs(velocitySignal);
	}

}
