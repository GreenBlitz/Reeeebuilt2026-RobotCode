package frc.robot.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;

import java.util.function.Supplier;

public class OdometryPlus<T> extends Odometry<T> {

	private final Supplier<Rotation3d> robotAnglesSupplier;

	/**
	 * Constructs an Odometry object.
	 *
	 * @param kinematics        The kinematics of the drivebase.
	 * @param gyroAngle         The angle reported by the gyroscope.
	 * @param wheelPositions    The current encoder readings.
	 * @param initialPoseMeters The starting position of the robot on the field.
	 */
	public OdometryPlus(Kinematics<?, T> kinematics, Rotation2d gyroAngle, T wheelPositions, Pose2d initialPoseMeters) {
		super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
		robotAnglesSupplier = null;
	}

	public OdometryPlus(
		Kinematics<?, T> kinematics,
		Rotation2d gyroAngle,
		T wheelPositions,
		Pose2d initialPoseMeters,
		Supplier<Rotation3d> robotAnglesSupplier
	) {
		super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
		this.robotAnglesSupplier = robotAnglesSupplier;
	}

	@Override
	public Pose2d update(Rotation2d gyroAngle, T wheelPositions) {
		Pose2d lastPose = getPoseMeters();
		Pose2d rawPose = super.update(gyroAngle, wheelPositions);

		if (robotAnglesSupplier == null)
			return rawPose;

		Rotation3d angles = robotAnglesSupplier.get();
		// in case angle above 90 so wont minus , i dont think pose is relevant when robot flips
		double scale = Math.max(0, Math.cos(angles.rotateBy(new Rotation3d(0, 0, -angles.getZ())).getY()));

		Transform2d delta = new Transform2d(lastPose, rawPose);

		Pose2d corrected = lastPose.plus(new Transform2d(delta.getTranslation().times(scale), delta.getRotation()));

		resetTranslation(corrected.getTranslation());

		return corrected;
	}

}
