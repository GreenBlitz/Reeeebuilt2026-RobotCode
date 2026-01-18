package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.VelocityPositionRequest;

import java.util.function.Consumer;

public class Phoenix6VelocityPositionRequest extends Phoenix6FeedForwardRequest implements VelocityPositionRequest {

	private final Consumer<Rotation2d> setVelocity;
	private Rotation2d velocity;

	public Phoenix6VelocityPositionRequest(
		Consumer<Rotation2d> setVelocity,
		Rotation2d position,
		ControlRequest controlRequest,
		Consumer<Double> setFeedForward,
		double defaultArbitraryFeedForward,
		Rotation2d velocity

	) {
		super(position, controlRequest, setVelocity, setFeedForward, defaultArbitraryFeedForward);
		this.setVelocity = setVelocity;
		this.velocity = velocity;
	}

	@Override
	public void withSetVelocity(Rotation2d velocity) {
		this.velocity = velocity;
		setVelocity.accept(velocity);
	}

	@Override
	public double getVelocity() {
		return velocity.getRotations();
	}

}
