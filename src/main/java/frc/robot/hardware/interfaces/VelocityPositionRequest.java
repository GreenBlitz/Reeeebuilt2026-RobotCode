package frc.robot.hardware.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface VelocityPositionRequest extends IFeedForwardRequest {

	void withSetVelocity(Rotation2d setPoint);

	double getVelocity();

}
