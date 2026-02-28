package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.VelocityPositionRequest;

import java.util.function.Consumer;

public class Phoenix6VelocityPositionRequest extends Phoenix6FeedForwardRequest implements VelocityPositionRequest {

	private final Consumer<Rotation2d> setVelocity;
	private Rotation2d targetVelocity;
	private Consumer<Rotation2d> setPosition;

	public Phoenix6VelocityPositionRequest(
		Consumer<Rotation2d> setVelocity,
		Consumer<Rotation2d> setPosition,
		Rotation2d position,
		ControlRequest controlRequest,
		Consumer<Double> setFeedForward,
		double defaultArbitraryFeedForward,
		Rotation2d defaultTargetVelocity

	) {
		super(position, controlRequest, setPosition, setFeedForward, defaultArbitraryFeedForward);
		this.setVelocity = setVelocity;
		this.targetVelocity = defaultTargetVelocity;
	}

	@Override
	public VelocityPositionRequest setVelocity(Rotation2d targetVelocityRPS) {
		this.targetVelocity = targetVelocityRPS;
		setVelocity.accept(targetVelocityRPS);
		return this;
	}

	@Override
	public Rotation2d getTargetVelocityRPS() {
		return targetVelocity;
	}

}
