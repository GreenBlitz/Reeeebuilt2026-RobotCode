package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.*;
import org.littletonrobotics.junction.Logger;

public class VelocityRoller extends Roller {

	private VelocityRequest velocityRequest;

	public VelocityRoller(
		String logPath,
		ControllableMotor roller,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		InputSignal<Rotation2d> positionSignal,
		IRequest<Double> voltageRequest
	) {
		super(logPath, roller, voltageSignal, currentSignal, positionSignal, voltageRequest);
	}


	public void setTargetVelocity(Rotation2d targetVelocity) {
		velocityRequest.setVelocity(targetVelocity);
		roller.applyRequest(velocityRequest);
	}

	public void log() {
		Logger.recordOutput(getLogPath() + "/VelocityRollerTargetVelocity", velocityRequest.getVelocityRPS());
	}

}
