package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotPoseEstimation {

	private double timestampSeconds = 0;
	private Pose2d estimatedPose;

	public RobotPoseEstimation(double initialTimestampSeconds, Pose2d estimatedPose) {
		this.timestampSeconds = initialTimestampSeconds;
		this.estimatedPose = estimatedPose;
	}

	public double getTimestampSeconds() {
		return timestampSeconds;
	}

	public Pose2d getEstimatedPose() {
		return estimatedPose;
	}

	public void setTimestampSeconds(double timestampSeconds) {
		this.timestampSeconds = timestampSeconds;
	}

	public void setEstimatedPose(Pose2d estimatedPose) {
		this.estimatedPose = estimatedPose;
	}

}
