package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.RobotPoseObservation;
import frc.robot.vision.cameras.limelight.Limelight;

public interface IVisionEstimator {

	void updateVision(Limelight ll, RobotPoseObservation... robotPoseVisionData);

	Pose2d getEstimatedPose();

}
