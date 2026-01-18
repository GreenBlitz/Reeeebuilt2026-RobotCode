package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IFeedForwardRequest;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.VelocityPositionRequest;

public class VelocityPositionArm extends Arm {

	VelocityPositionRequest velocityPositionRequest;
	IFeedForwardRequest positionRequest;
	IRequest<Double> voltageRequest;

	public VelocityPositionArm(
		String logPath,
		ControllableMotor motor,
		ArmSignals signals,
		IRequest<Double> voltageRequest,
		VelocityPositionRequest velocityPositionRequest,
		IFeedForwardRequest positionRequest,
		double kG
	) {
		super(logPath, motor, signals, voltageRequest, positionRequest, kG);
		this.velocityPositionRequest = velocityPositionRequest;
	}

	public void setPositionVelocity(Rotation2d position, Rotation2d velocity) {
		velocityPositionRequest.withSetPoint(position);
		velocityPositionRequest.withSetVelocity(velocity);
		motor.applyRequest(velocityPositionRequest);
	}

}
