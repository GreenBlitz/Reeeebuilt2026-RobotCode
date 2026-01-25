package frc.robot.hardware.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface VelocityRequest extends IFeedForwardRequest {

	VelocityRequest setVelocity(Rotation2d targetVelocityRPS);

	Rotation2d getVelocityRPS();

}
