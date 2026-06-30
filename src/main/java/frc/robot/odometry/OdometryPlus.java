package frc.robot.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import java.util.function.Supplier;

public class OdometryPlus<T> extends Odometry<T> {

	private final Supplier<Rotation3d> robotOrientationSupplier;

	/**
	 * Constructs a 2d odometry object that takes into consideration the robot's 3d orientation.
	 *
	 * @param kinematics        The kinematics of the chassis.
	 * @param gyroAngle         The given robot's yaw.
	 * @param wheelPositions    The current wheel positions.
	 * @param initialPoseMeters The starting position of the robot on the field.
	 */
	public OdometryPlus(Kinematics<?, T> kinematics, Rotation2d gyroAngle, T wheelPositions, Pose2d initialPoseMeters) {
		super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
		this.robotOrientationSupplier = () -> new Rotation3d(0, 0, this.getHeading().getRadians());
	}

	public OdometryPlus(
		Kinematics<?, T> kinematics,
		Rotation2d gyroAngle,
		T wheelPositions,
		Pose2d initialPoseMeters,
		Supplier<Rotation3d> robotOrientationSupplier
	) {
		super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
		this.robotOrientationSupplier = robotOrientationSupplier;
	}

	@Override
	public Pose2d update(Rotation2d gyroAngle, T wheelPositions) {
		Pose2d lastPose = getPoseMeters();
		Pose2d unCompensatedPose = super.update(gyroAngle, wheelPositions);

		if (robotOrientationSupplier == null)
			return unCompensatedPose;
		Rotation3d robotOrientation = robotOrientationSupplier.get();
		double scale = Math.cos(robotOrientation.rotateBy(new Rotation3d(0, 0, -robotOrientation.getZ())).getY());
		Transform2d unCompensatedDistance = new Transform2d(lastPose, unCompensatedPose);

		Pose2d compensatedByPitch = lastPose
			.plus(new Transform2d(unCompensatedDistance.getTranslation().times(scale), unCompensatedDistance.getRotation()));

		resetTranslation(compensatedByPitch.getTranslation());

		return compensatedByPitch;
	}

	public Rotation2d getHeading() {
		return this.getPoseMeters().getRotation();
	}

}
