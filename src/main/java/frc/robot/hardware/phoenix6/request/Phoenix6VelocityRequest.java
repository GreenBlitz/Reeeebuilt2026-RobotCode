package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.VelocityRequest;

import java.util.function.Consumer;

public class Phoenix6VelocityRequest extends Phoenix6FeedForwardRequest implements VelocityRequest {

	private Consumer<Rotation2d> setVelocity;
	private Rotation2d velocity;

	Phoenix6VelocityRequest(
		Rotation2d defaultSetPoint,
		ControlRequest controlRequest,
		Consumer<Rotation2d> setSetPoint,
		Consumer<Double> setFeedForward,
		double defaultArbitraryFeedForward
	) {
		super(defaultSetPoint, controlRequest, setSetPoint, setFeedForward, defaultArbitraryFeedForward);
	}

	@Override
	public VelocityRequest setVelocity(Rotation2d targetVelocityRPS) {
		this.velocity = targetVelocityRPS;
		setVelocity.accept(targetVelocityRPS);
		return this;
	}

	@Override
	public Rotation2d getVelocityRPS() {
		return velocity;
	}

}
