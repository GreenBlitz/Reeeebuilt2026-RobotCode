package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface IOdometryEstimator {

	void updateOdometry(OdometryData[] odometryData);

	void updateOdometry(OdometryData odometryData);

	void resetPose(
		double timestampSeconds,
		Rotation3d imuOrientation,
		Translation2d imuXYAccelerationG,
		SwerveModulePosition[] wheelPositions,
		SwerveModuleState[] wheelStates,
		Pose2d poseMeters
	);

	Pose2d getOdometryPose();

	void setHeading(Rotation2d newHeading);

}
