package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.cameras.limelight.Limelight;

import java.util.List;
import java.util.Optional;


public interface IPoseEstimator extends IVisionEstimator, IOdometryEstimator {

	void resetPose(Pose2d poseMeters);

	Pose2d getEstimatedPose();

	Optional<Pose2d> getEstimatedPoseAtTimestamp(double timestampSeconds);

	boolean isIMUOffsetCalibrated();

	void log();

	void updateVisionPoses();

	void updateByBestPose(List<Limelight> lls);

}
