package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.Odometry;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.vision.RobotPoseObservation;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.OdometryData;
import frc.utils.buffers.RingBuffer.RingBuffer;
import frc.utils.math.StandardDeviations2D;
import frc.utils.math.StatisticsMath;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class WPILibPoseEstimatorWrapper implements IPoseEstimator {

	private final String logPath;
	private final SwerveDriveKinematics kinematics;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private final RingBuffer<Rotation2d> poseToIMUYawDifferenceBuffer;
	private final TimeInterpolatableBuffer<Rotation2d> imuYawBuffer;
	private final TimeInterpolatableBuffer<Double> imuAccelerationBuffer;
	private RobotPoseObservation lastVisionObservation;
	private OdometryData lastOdometryData;
	private boolean isIMUOffsetCalibrated;
	private double odometryCausedEstimatedPoseError;

	public WPILibPoseEstimatorWrapper(
		String logPath,
		SwerveDriveKinematics kinematics,
		SwerveModulePosition[] initialModulePositions,
		Rotation3d initialIMUOriantation,
		double initialIMUAccelerationMagnitudeG,
		double initialTimestampSeconds
	) {
		this.logPath = logPath;
		this.kinematics = kinematics;
		this.odometryEstimator = new Odometry<>(
			kinematics,
			Rotation2d.fromRadians(initialIMUOriantation.getZ()),
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
			Optional.of(initialIMUOriantation),
			Optional.of(initialIMUAccelerationMagnitudeG)
		);
		this.isIMUOffsetCalibrated = false;
		this.poseToIMUYawDifferenceBuffer = new RingBuffer<>(WPILibPoseEstimatorConstants.POSE_TO_IMU_YAW_DIFFERENCE_BUFFER_SIZE);
		this.imuYawBuffer = TimeInterpolatableBuffer.createBuffer(WPILibPoseEstimatorConstants.IMU_YAW_BUFFER_SIZE_SECONDS);
		this.imuAccelerationBuffer = TimeInterpolatableBuffer
			.createDoubleBuffer(WPILibPoseEstimatorConstants.IMU_ACCELERATION_BUFFER_SIZE_SECONDS);
		this.odometryCausedEstimatedPoseError = 0;
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
		if (data.getImuOrientation().isPresent() || lastOdometryData.getImuOrientation().isPresent()) {
			data.getImuOrientation()
				.ifPresentOrElse(
					(imuOrientation) -> data.setIMUYaw(Rotation2d.fromRadians(imuOrientation.getZ())),
					() -> data.setIMUYaw(
						Rotation2d.fromRadians(lastOdometryData.getImuOrientation().get().getZ())
							.plus(Rotation2d.fromRadians(changeInPose.dtheta))
					)
				);
		}
		poseEstimator
			.updateWithTime(data.getTimestampSeconds(), Rotation2d.fromRadians(data.getImuOrientation().get().getZ()), data.getWheelPositions());
		imuYawBuffer.addSample(data.getTimestampSeconds(), Rotation2d.fromRadians(data.getImuOrientation().get().getZ()));

		updateCausedErrorForOdometryData(data);

		lastOdometryData.setWheelPositions(data.getWheelPositions());
		lastOdometryData.setIMUOrientation(data.getImuOrientation());
		lastOdometryData.setTimestamp(data.getTimestampSeconds());
		lastOdometryData.setIMUAcceleration(data.getImuAccelerationMagnitudeG());

		data.getImuAccelerationMagnitudeG()
			.ifPresent((acceleration) -> imuAccelerationBuffer.addSample(lastOdometryData.getTimestampSeconds(), acceleration));
	}

	public void updateCausedErrorForOdometryData(OdometryData data) {
		Twist2d changeInPose = kinematics.toTwist2d(lastOdometryData.getWheelPositions(), data.getWheelPositions());
		double changeInPoseNorm = Math.hypot(changeInPose.dx, changeInPose.dy);
		odometryCausedEstimatedPoseError += isColliding(data)
			? WPILibPoseEstimatorConstants.COLLISION_CAUSED_ODOMETRY_ERROR_COUNTER_ADDITION * changeInPoseNorm
			: 0;
		odometryCausedEstimatedPoseError += isTilted(data)
			? WPILibPoseEstimatorConstants.TILT_CAUSED_ODOMETRY_ERROR_COUNTER_ADDITION * changeInPoseNorm
			: 0;
	}

	public boolean isColliding(OdometryData data) {
		return data.getImuAccelerationMagnitudeG()
			.map((acceleration) -> acceleration >= SwerveConstants.MIN_COLLISION_G_FORCE)
			.orElseGet(() -> false);
	}

	public boolean isTilted(OdometryData data) {
		return data.getImuOrientation()
			.map(
				(imuOrientation) -> imuOrientation.getX() >= SwerveConstants.TILTED_ROBOT_ROLL_TOLERANCE.getRadians()
					|| imuOrientation.getY() >= SwerveConstants.TILTED_ROBOT_PITCH_TOLERANCE.getRadians()
			)
			.orElseGet(() -> false);
	}

	@Override
	public void updateVision(RobotPoseObservation... visionRobotPoseObservations) {
		for (RobotPoseObservation visionRobotPoseObservation : visionRobotPoseObservations) {
			updateVision(visionRobotPoseObservation);
		}
	}

	@Override
	public void resetPose(
		double timestampSeconds,
		Rotation3d imuOrientation,
		double imuAccelerationMagnitudeG,
		SwerveModulePosition[] wheelPositions,
		Pose2d poseMeters
	) {
		Logger.recordOutput(logPath + "/lastPoseResetTo", poseMeters);
		poseEstimator.resetPosition(Rotation2d.fromRadians(imuOrientation.getZ()), wheelPositions, poseMeters);
		this.lastOdometryData = new OdometryData(
			timestampSeconds,
			wheelPositions,
			Optional.of(imuOrientation),
			Optional.of(imuAccelerationMagnitudeG)
		);
		poseToIMUYawDifferenceBuffer.clear();
		imuYawBuffer.addSample(timestampSeconds, Rotation2d.fromRadians(imuOrientation.getZ()));
		imuAccelerationBuffer.addSample(timestampSeconds, imuAccelerationMagnitudeG);
	}

	@Override
	public void resetPose(Pose2d poseMeters) {
		resetPose(
			lastOdometryData.getTimestampSeconds(),
			lastOdometryData.getImuOrientation().get(),
			lastOdometryData.getImuAccelerationMagnitudeG().get(),
			lastOdometryData.getWheelPositions(),
			poseMeters
		);
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
		Logger.recordOutput(logPath + "/lastOdometryUpdate", lastOdometryData.getTimestampSeconds());
		if (lastVisionObservation != null) {
			Logger.recordOutput(logPath + "/lastVisionUpdate", lastVisionObservation.timestampSeconds());
		}
		Logger.recordOutput(logPath + "/isIMUOffsetCalibrated", isIMUOffsetCalibrated);
		Logger.recordOutput(logPath + "/odometryCausedEstimatedPoseError", odometryCausedEstimatedPoseError);
	}

	public void resetIsIMUOffsetCalibrated() {
		poseToIMUYawDifferenceBuffer.clear();
		isIMUOffsetCalibrated = false;
	}

	private void updateVision(RobotPoseObservation visionRobotPoseObservation) {
		addVisionMeasurement(visionRobotPoseObservation);

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

	private void addVisionMeasurement(RobotPoseObservation visionObservation) {
		poseEstimator.addVisionMeasurement(
			visionObservation.robotPose(),
			visionObservation.timestampSeconds(),
			getOdometryErrorCompensatedVisionStdDevs(visionObservation.stdDevs()).asColumnVector()
		);
		this.lastVisionObservation = visionObservation;
	}

	private StandardDeviations2D getOdometryErrorCompensatedVisionStdDevs(StandardDeviations2D visionStd) {
		StandardDeviations2D compensatedStdDevs = new StandardDeviations2D(
			(visionStd.xStandardDeviations() * WPILibPoseEstimatorConstants.CONSTANT_TO_CALC_ERROR_REDUCTION) / odometryCausedEstimatedPoseError,
			(visionStd.yStandardDeviations() * WPILibPoseEstimatorConstants.CONSTANT_TO_CALC_ERROR_REDUCTION) / odometryCausedEstimatedPoseError,
			(visionStd.angleStandardDeviations() * WPILibPoseEstimatorConstants.CONSTANT_TO_CALC_ERROR_REDUCTION)
				/ odometryCausedEstimatedPoseError
		);
		double avgStdXnY = (compensatedStdDevs.xStandardDeviations() + compensatedStdDevs.yStandardDeviations()) / 2.0;
		this.odometryCausedEstimatedPoseError /= WPILibPoseEstimatorConstants.CONSTANT_TO_CALC_STD;

		return compensatedStdDevs;
	}

	private void updateIsIMUOffsetCalibrated() {
		double poseToIMUYawDifferenceStdDev = StatisticsMath.calculateStandardDeviations(poseToIMUYawDifferenceBuffer, Rotation2d::getRadians);
		isIMUOffsetCalibrated = poseToIMUYawDifferenceStdDev < WPILibPoseEstimatorConstants.MAX_POSE_TO_IMU_YAW_DIFFERENCE_STD_DEV
			&& poseToIMUYawDifferenceBuffer.isFull();
		Logger.recordOutput(logPath + "/poseToIMUOffsetStdDev", poseToIMUYawDifferenceStdDev);
	}

	private Optional<Rotation2d> getEstimatedPoseToIMUYawDifference(Optional<Rotation2d> gyroYaw, double timestampSeconds) {
		return getEstimatedPoseAtTimestamp(timestampSeconds)
			.flatMap(estimatedPose -> gyroYaw.map(yaw -> estimatedPose.getRotation().minus(yaw)));
	}

}
