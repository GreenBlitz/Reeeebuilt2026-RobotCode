package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.interfaces.ControllableMotor;
import org.littletonrobotics.junction.Logger;

public class VelocityRoller extends Roller {

	private final InputSignal<Rotation2d> velocitySignal;
	private final IRequest<Rotation2d> velocityVoltageRequest;
	private final IRequest<Rotation2d> velocityBangBangRequest;
	private final VelocityRollerCommandBuilder commandsBuilder;

	Rotation2d targetVelocity;

	public VelocityRoller(
		String logPath,
		ControllableMotor roller,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Rotation2d> velocitySignal,
		IRequest<Double> voltageRequest,
		IRequest<Rotation2d> velocityVoltageRequest,
		IRequest<Rotation2d> velocityBangBangRequest
	) {
		super(logPath, roller, voltageSignal, currentSignal, positionSignal, voltageRequest);
		this.velocitySignal = velocitySignal;
		this.velocityVoltageRequest = velocityVoltageRequest;
		this.velocityBangBangRequest = velocityBangBangRequest;
		this.commandsBuilder = new VelocityRollerCommandBuilder(this);
		this.targetVelocity = Rotation2d.kZero;
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
		this.targetVelocity = targetVelocity;
//		if (
//				velocitySignal.isLess(targetVelocity)
//		) {
		motor.applyRequest(velocityBangBangRequest.withSetPoint(targetVelocity));
		Logger.recordOutput(getLogPath() + "/usedControl", "Bang Bang");
//		} else {
//			motor.applyRequest(velocityVoltageRequest.withSetPoint(targetVelocity));
//			Logger.recordOutput(getLogPath() + "/usedControl", "PID");
//		}
	}

	@Override
	public void stop() {
		super.stop();
		velocityVoltageRequest.withSetPoint(Rotation2d.kZero);
	}

	@Override
	public void update() {
		super.update();
		Logger.recordOutput(getLogPath() + "/TargetVelocity", targetVelocity);
		motor.updateInputs(velocitySignal);
	}

}
