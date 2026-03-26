package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotPoseEstimation {

	private double timestampSeconds = 0;
	private Pose2d estimatedPose;

	public RobotPoseEstimation(double initialTimestampSeconds, Pose2d estimatedPose) {
		this.timestampSeconds = initialTimestampSeconds;
		this.estimatedPose = estimatedPose;
	}

	public RobotPoseEstimation(double timestampSeconds){
		this.timestampSeconds = timestampSeconds;
		this.estimatedPose = new Pose2d();
	}

	public double getTimestampSeconds() {
		return timestampSeconds;
	}

	public Pose2d getEstimatedPose() {
		return estimatedPose;
	}

	public double getX(){ return estimatedPose.getX(); }

	public double getY(){ return estimatedPose.getY();}

	public Rotation2d getRotation(){ return estimatedPose.getRotation(); }

	public void setTimestampSeconds(double timestampSeconds) {
		this.timestampSeconds = timestampSeconds;
	}

	public void setEstimatedPose(Pose2d estimatedPose) {
		this.estimatedPose = estimatedPose;
	}

}
