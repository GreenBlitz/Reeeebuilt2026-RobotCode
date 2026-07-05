package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.vision.RobotPoseObservation;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.OdometryData;
import frc.utils.buffers.RingBuffer.RingBuffer;
import frc.utils.math.StandardDeviations2D;
import frc.utils.math.StatisticsMath;
import frc.utils.pose.PoseUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class WPILibPoseEstimatorWrapper implements IPoseEstimator {

	private final String logPath;
	private final SwerveDriveKinematics kinematics;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private final RingBuffer<Rotation2d> poseToIMUYawDifferenceBuffer;
	private final TimeInterpolatableBuffer<Rotation2d> imuYawBuffer;
	private final TimeInterpolatableBuffer<Translation2d> imuXYAccelerationGBuffer;
	private OdometryData lastOdometryData;
	private double lastVisionUpdateTimestamp;
	private boolean isIMUOffsetCalibrated;

	public WPILibPoseEstimatorWrapper(
		String logPath,
		SwerveDriveKinematics kinematics,
		SwerveModulePosition[] initialModulePositions,
		SwerveModuleState[] initialModuleStates,
		Rotation3d initialIMUOrientation,
		Translation2d initialIMUXYAccelerationG,
		double initialTimestampSeconds
	) {
		this.logPath = logPath;
		this.kinematics = kinematics;
		this.odometryEstimator = new Odometry<>(
			kinematics,
			Rotation2d.fromRadians(initialIMUOrientation.getZ()),
			initialModulePositions,
			WPILibPoseEstimatorConstants.STARTING_ODOMETRY_POSE
		);
		this.poseEstimator = new PoseEstimator<>(
			kinematics,
			odometryEstimator,
			WPILibPoseEstimatorConstants.DEFAULT_ODOMETRY_STD_DEV.asColumnVector(),
			WPILibPoseEstimatorConstants.DEFAULT_VISION_STD_DEV.asColumnVector()
		);
		this.lastOdometryData = new OdometryData(
			initialTimestampSeconds,
			initialModulePositions,
			initialModuleStates,
			Optional.of(initialIMUOrientation),
			Optional.of(initialIMUXYAccelerationG)
		);
		this.isIMUOffsetCalibrated = false;
		this.poseToIMUYawDifferenceBuffer = new RingBuffer<>(WPILibPoseEstimatorConstants.POSE_TO_IMU_YAW_DIFFERENCE_BUFFER_SIZE);
		this.imuYawBuffer = TimeInterpolatableBuffer.createBuffer(WPILibPoseEstimatorConstants.IMU_YAW_BUFFER_SIZE_SECONDS);
		this.imuXYAccelerationGBuffer = TimeInterpolatableBuffer
			.createBuffer(WPILibPoseEstimatorConstants.IMU_XY_ACCELERATION_G_BUFFER_SIZE_SECONDS);
	}


	@Override
	public Pose2d getEstimatedPose() {
		return poseEstimator.getEstimatedPosition();
	}

	@Override
	public Optional<Pose2d> getEstimatedPoseAtTimestamp(double timestampSeconds) {
		return poseEstimator.sampleAt(timestampSeconds);
	}

	@Override
	public Pose2d getOdometryPose() {
		return odometryEstimator.getPoseMeters();
	}

	@Override
	public void updateOdometry(OdometryData[] odometryData) {
		for (OdometryData data : odometryData) {
			updateOdometry(data);
		}
	}

	@Override
	public void updateOdometry(OdometryData data) {
		Twist2d changeInPose = kinematics.toTwist2d(lastOdometryData.getWheelPositions(), data.getWheelPositions());
		if (data.getIMUOrientation().isEmpty()) {
			data.setIMUOrientation(
				new Rotation3d(
					lastOdometryData.getIMUOrientation().get().getX(),
					lastOdometryData.getIMUOrientation().get().getY(),
					Rotation2d.fromRadians(lastOdometryData.getIMUOrientation().get().getZ())
						.plus(Rotation2d.fromRadians(changeInPose.dtheta))
						.getRadians()
				)
			);
		}

		poseEstimator
			.updateWithTime(data.getTimestampSeconds(), Rotation2d.fromRadians(data.getIMUOrientation().get().getZ()), data.getWheelPositions());
		imuYawBuffer.addSample(data.getTimestampSeconds(), Rotation2d.fromRadians(data.getIMUOrientation().get().getZ()));
		data.getIMUXYAccelerationG().ifPresent((acceleration) -> imuXYAccelerationGBuffer.addSample(data.getTimestampSeconds(), acceleration));

		lastOdometryData.setWheelPositions(data.getWheelPositions());
		lastOdometryData.setWheelStates(data.getWheelStates());
		lastOdometryData.setIMUOrientation(data.getIMUOrientation());
		lastOdometryData.setIMUXYAcceleration(data.getIMUXYAccelerationG());
		lastOdometryData.setTimestamp(data.getTimestampSeconds());
	}

	@Override
	public void updateVision(RobotPoseObservation[]... visionRobotPoseObservations) {
		for (RobotPoseObservation[] visionRobotPoseObservation : visionRobotPoseObservations) {
			updateVision(visionRobotPoseObservation);
		}
	}

	@Override
	public void resetPose(OdometryData odometryData, Pose2d poseMeters) {
		Logger.recordOutput(logPath + "/lastPoseResetTo", poseMeters);

		poseEstimator.resetPosition(
			Rotation2d.fromRadians(odometryData.getIMUOrientation().orElse(Rotation3d.kZero).getZ()),
			odometryData.getWheelPositions(),
			poseMeters
		);
		this.lastOdometryData = odometryData;
		poseToIMUYawDifferenceBuffer.clear();
		imuYawBuffer.addSample(
			odometryData.getTimestampSeconds(),
			Rotation2d.fromRadians(odometryData.getIMUOrientation().orElse(Rotation3d.kZero).getZ())
		);
		imuXYAccelerationGBuffer.addSample(odometryData.getTimestampSeconds(), odometryData.getIMUXYAccelerationG().orElse(Translation2d.kZero));
	}

	@Override
	public void resetPose(Pose2d poseMeters) {
		resetPose(lastOdometryData, poseMeters);
	}

	@Override
	public void setHeading(Rotation2d newHeading) {
		poseEstimator.resetRotation(newHeading);
		poseToIMUYawDifferenceBuffer.clear();
	}

	@Override
	public boolean isIMUOffsetCalibrated() {
		return isIMUOffsetCalibrated;
	}

	@Override
	public void log() {
		Logger.recordOutput(logPath + "/estimatedPose", getEstimatedPose());
		Logger.recordOutput(logPath + "/odometryPose", getOdometryPose());
		Logger.recordOutput(logPath + "/predictedOdometryPose", getPredictedOdometryPose());
		if (lastVisionUpdateTimestamp != 0) {
			Logger.recordOutput(logPath + "/lastVisionUpdate", lastVisionUpdateTimestamp);
		}
		Logger.recordOutput(logPath + "/lastOdometryUpdate", lastOdometryData.getTimestampSeconds());
		Logger.recordOutput(logPath + "/isIMUOffsetCalibrated", isIMUOffsetCalibrated);

		lastOdometryData.getIMUXYAccelerationG()
			.ifPresent(
				(imuXYAcceleration) -> Logger.recordOutput(
					logPath + "/isColliding",
					PoseUtil.getIsColliding(imuXYAcceleration, WPILibPoseEstimatorConstants.MINIMUM_COLLISION_IMU_ACCELERATION_G)
				)
			);

		lastOdometryData.getIMUOrientation()
			.ifPresent(
				(imuOrientation) -> Logger.recordOutput(
					logPath + "/isTilted",
					PoseUtil.getIsTilted(
						Rotation2d.fromRadians(imuOrientation.getX()),
						Rotation2d.fromRadians(imuOrientation.getY()),
						WPILibPoseEstimatorConstants.MINIMUM_TILT_IMU_ROLL,
						WPILibPoseEstimatorConstants.MINIMUM_TILT_IMU_PITCH
					)
				)
			);

		Logger.recordOutput(
			logPath + "/isSkidding",
			PoseUtil.getIsSkidding(
				kinematics,
				lastOdometryData.getWheelStates(),
				WPILibPoseEstimatorConstants.MINIMUM_SKID_ROBOT_TO_MODULE_VELOCITY_DIFFERENCE_METERS_PER_SECOND
			)
		);
	}

	private void updateVision(RobotPoseObservation[] visionRobotPoseObservations) {
		List<RobotPoseObservation> similarVisionRobotPoseObservations = Arrays.stream(visionRobotPoseObservations)
			.filter(observation -> Arrays.stream(visionRobotPoseObservations).anyMatch(other -> getAreObservationsSimilar(observation, other)))
			.toList();
		Arrays.stream(visionRobotPoseObservations)
			.forEach(observation -> addVisionMeasurement(observation, similarVisionRobotPoseObservations.contains(observation)));

		Arrays.stream(visionRobotPoseObservations).forEach(this::updateIMUOffset);

		lastVisionUpdateTimestamp = Arrays.stream(visionRobotPoseObservations)
			.max(Comparator.comparingDouble(RobotPoseObservation::timestampSeconds))
			.orElse(new RobotPoseObservation())
			.timestampSeconds();
	}

	public void updateIMUOffset(RobotPoseObservation visionRobotPoseObservation) {
		getEstimatedPoseToIMUYawDifference(
			imuYawBuffer.getSample(visionRobotPoseObservation.timestampSeconds()),
			visionRobotPoseObservation.timestampSeconds()
		).ifPresent(yawDifference -> {
			poseToIMUYawDifferenceBuffer.insert(yawDifference);

			if (!isIMUOffsetCalibrated) {
				updateIsIMUOffsetCalibrated();
			}
		});
	}

	private void updateIsIMUOffsetCalibrated() {
		double poseToIMUYawDifferenceStdDev = StatisticsMath.calculateStandardDeviations(poseToIMUYawDifferenceBuffer, Rotation2d::getRadians);
		isIMUOffsetCalibrated = poseToIMUYawDifferenceStdDev < WPILibPoseEstimatorConstants.MAX_POSE_TO_IMU_YAW_DIFFERENCE_STD_DEV
			&& poseToIMUYawDifferenceBuffer.isFull();
		Logger.recordOutput(logPath + "/poseToIMUOffsetStdDev", poseToIMUYawDifferenceStdDev);
	}

	public void resetIsIMUOffsetCalibrated() {
		poseToIMUYawDifferenceBuffer.clear();
		isIMUOffsetCalibrated = false;
	}

	private void addVisionMeasurement(RobotPoseObservation visionObservation, boolean isObservationSimilar) {
		poseEstimator.addVisionMeasurement(
			visionObservation.robotPose(),
			visionObservation.timestampSeconds(),
			getSimilarityAccountedVisionStdDevs(getCollisionCompensatedVisionStdDevs(visionObservation), isObservationSimilar).asColumnVector()
		);
	}

	private StandardDeviations2D getCollisionCompensatedVisionStdDevs(RobotPoseObservation visionObservation) {
		boolean isColliding = imuXYAccelerationGBuffer.getSample(visionObservation.timestampSeconds())
			.map(
				(imuAccelerationG) -> PoseUtil
					.getIsColliding(imuAccelerationG, WPILibPoseEstimatorConstants.MINIMUM_COLLISION_IMU_ACCELERATION_G)
			)
			.orElse(false);

		return isColliding
			? new StandardDeviations2D(
				visionObservation.stdDevs()
					.asColumnVector()
					.minus(WPILibPoseEstimatorConstants.VISION_STD_DEV_COLLISION_REDUCTION.asColumnVector())
			)
			: visionObservation.stdDevs();
	}

	private StandardDeviations2D getSimilarityAccountedVisionStdDevs(
		StandardDeviations2D visionStandardDeviations,
		boolean isObservationSimilar
	) {
		return isObservationSimilar
			? new StandardDeviations2D(
				visionStandardDeviations.asColumnVector()
					.minus(WPILibPoseEstimatorConstants.VISION_STD_DEV_SIMILARITY_REDUCTION.asColumnVector())
			)
			: visionStandardDeviations;
	}

	private Optional<Rotation2d> getEstimatedPoseToIMUYawDifference(Optional<Rotation2d> gyroYaw, double timestampSeconds) {
		return getEstimatedPoseAtTimestamp(timestampSeconds)
			.flatMap(estimatedPose -> gyroYaw.map(yaw -> estimatedPose.getRotation().minus(yaw)));
	}

	private Pose2d getPredictedOdometryPose() {
		return poseEstimator.getEstimatedPosition()
			.exp(
				kinematics.toChassisSpeeds(lastOdometryData.getWheelStates())
					.toTwist2d(WPILibPoseEstimatorConstants.ODOMETRY_POSE_PREDICTION_TIME_SECONDS)
			);
	}

	private boolean getAreObservationsSimilar(RobotPoseObservation observation, RobotPoseObservation other) {
		return !observation.equals(other)
			&& Math.abs(observation.timestampSeconds() - other.timestampSeconds())
				< WPILibPoseEstimatorConstants.SIMILAR_POSE_TIMESTAMP_TOLERANCE_SECONDS
			&& getArePosesSimilar(observation.robotPose(), other.robotPose());
	}

	private boolean getArePosesSimilar(Pose2d pose, Pose2d other) {
		return PoseUtil.getDifference(pose.getTranslation(), other.getTranslation())
			< WPILibPoseEstimatorConstants.SIMILAR_POSE_TRANSLATION_NORM_TOLERANCE_METERS
			&& PoseUtil.getDifferenceRadians(pose.getRotation(), other.getRotation())
				< WPILibPoseEstimatorConstants.SIMILAR_POSE_ROTATION_TOLERANCE.getRadians();
	}

}
