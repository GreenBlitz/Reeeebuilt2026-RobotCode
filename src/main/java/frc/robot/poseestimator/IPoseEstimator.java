package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.RobotPoseObservation;

import java.util.Optional;


public interface IPoseEstimator extends IVisionEstimator, IOdometryEstimator {

	void resetPose(Pose2d poseMeters);

	Pose2d getEstimatedPose();

	Optional<Pose2d> getEstimatedPoseAtTimestamp(double timestampSeconds);

	void updateIMUOffsetCalibration(RobotPoseObservation... visionRobotPoseObservations);

	boolean isIMUOffsetCalibrated();

	void log();

}
