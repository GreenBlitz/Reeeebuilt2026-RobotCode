package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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

import java.util.Optional;

public class WPILibPoseEstimatorWrapper implements IPoseEstimator {

	private final String logPath;
	private final SwerveDriveKinematics kinematics;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private final RingBuffer<Rotation2d> poseToIMUYawDifferenceBuffer;
	private final TimeInterpolatableBuffer<Rotation2d> imuYawBuffer;
	private RobotPoseObservation lastVisionObservation;
	private OdometryData lastOdometryData;
	private boolean isIMUOffsetCalibrated;

	private boolean isColliding;
	private boolean isTilted;
	private boolean isSkidding;
	private double odometryAccuracy;

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

		this.isColliding = false;
		this.isTilted = false;
		this.isSkidding = false;
		this.odometryAccuracy = 1;
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

		updateOdometryProblemsStatus(data);
		updateOdometryAccuracy(data);
		poseEstimator
			.updateWithTime(data.getTimestampSeconds(), Rotation2d.fromRadians(data.getIMUOrientation().get().getZ()), data.getWheelPositions());
		imuYawBuffer.addSample(data.getTimestampSeconds(), Rotation2d.fromRadians(data.getIMUOrientation().get().getZ()));
		lastOdometryData.setWheelPositions(data.getWheelPositions());
		lastOdometryData.setWheelStates(data.getWheelStates());
		lastOdometryData.setIMUOrientation(data.getIMUOrientation());
		lastOdometryData.setIMUXYAcceleration(data.getIMUXYAccelerationG());
		lastOdometryData.setTimestamp(data.getTimestampSeconds());
	}

	@Override
	public void updateVision(RobotPoseObservation... visionRobotPoseObservations) {
		for (RobotPoseObservation visionRobotPoseObservation : visionRobotPoseObservations) {
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
		Logger.recordOutput(logPath + "/lastOdometryUpdate", lastOdometryData.getTimestampSeconds());
		if (lastVisionObservation != null) {
			Logger.recordOutput(logPath + "/lastVisionUpdate", lastVisionObservation.timestampSeconds());
		}
		Logger.recordOutput(logPath + "/isIMUOffsetCalibrated", isIMUOffsetCalibrated);

		Logger.recordOutput("/isColliding", isColliding);
		Logger.recordOutput("/isTilted", isTilted);
		Logger.recordOutput("/isSkidding", isSkidding);
		Logger.recordOutput("/odometryAccuracy", odometryAccuracy);
	}

	public void resetIsIMUOffsetCalibrated() {
		poseToIMUYawDifferenceBuffer.clear();
		isIMUOffsetCalibrated = false;
	}

	private void updateOdometryProblemsStatus(OdometryData data) {
		isColliding = data.getIMUXYAccelerationG()
			.map(
				imuXYAcceleration -> PoseUtil
					.getIsColliding(imuXYAcceleration, WPILibPoseEstimatorConstants.MINIMUM_COLLISION_IMU_ACCELERATION_G)
			)
			.orElse(false);

		isTilted = data.getIMUOrientation()
			.map(
				imuOrientation -> PoseUtil.getIsTilted(
					Rotation2d.fromRadians(imuOrientation.getX()),
					Rotation2d.fromRadians(imuOrientation.getY()),
					WPILibPoseEstimatorConstants.MINIMUM_TILT_IMU_ROLL,
					WPILibPoseEstimatorConstants.MINIMUM_TILT_IMU_PITCH
				)
			)
			.orElse(false);

		isSkidding = PoseUtil.getIsSkidding(
			kinematics,
			data.getWheelStates(),
			WPILibPoseEstimatorConstants.MINIMUM_SKID_ROBOT_TO_MODULE_VELOCITY_DIFFERENCE_METERS_PER_SECOND
		);
	}

	private void updateOdometryAccuracy(OdometryData data) {
		Twist2d changeInPose = kinematics.toTwist2d(lastOdometryData.getWheelPositions(), data.getWheelPositions());
		double changeInPoseNorm = Math.hypot(changeInPose.dx, changeInPose.dy);

		odometryAccuracy -= isColliding ? WPILibPoseEstimatorConstants.COLLISION_ODOMETRY_ACCURACY_REDUCTION_FACTOR * changeInPoseNorm : 0;

		odometryAccuracy -= isTilted ? WPILibPoseEstimatorConstants.TILT_ODOMETRY_ACCURACY_REDUCTION_FACTOR * changeInPoseNorm : 0;

		odometryAccuracy -= isSkidding ? WPILibPoseEstimatorConstants.SKID_ODOMETRY_ACCURACY_REDUCTION_FACTOR * changeInPoseNorm : 0;

		odometryAccuracy = Math.max(odometryAccuracy, 0);
	}

	private void updateVision(RobotPoseObservation visionRobotPoseObservation) {
		RobotPoseObservation compensatedVisionObservation = new RobotPoseObservation(
			visionRobotPoseObservation.timestampSeconds(),
			visionRobotPoseObservation.robotPose(),
			compensateByPoseEstimatorConditions(visionRobotPoseObservation.stdDevs())
		);
		Logger.recordOutput(logPath + "/lastCompensatedVisionObservationStdDevs", compensatedVisionObservation.stdDevs());
		addVisionMeasurement(compensatedVisionObservation);

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
			visionObservation.stdDevs().asColumnVector()
		);
		this.lastVisionObservation = visionObservation;
	}

	private StandardDeviations2D compensateByPoseEstimatorConditions(StandardDeviations2D visionStdDevs) {
		return compensateByIsIMUOffsetCalibrated(compensateByOdometryAccuracy(visionStdDevs));
	}

	private StandardDeviations2D compensateByIsIMUOffsetCalibrated(StandardDeviations2D visionStdDevs) {
		return new StandardDeviations2D(
			visionStdDevs.xStandardDeviations(),
			visionStdDevs.yStandardDeviations(),
			isIMUOffsetCalibrated
				? visionStdDevs.angleStandardDeviations() + WPILibPoseEstimatorConstants.CALIBRATED_IMU_OFFSET_VISION_ANGLE_STD_DEVS_ADDITION
				: visionStdDevs.angleStandardDeviations()
		);
	}

	private StandardDeviations2D compensateByOdometryAccuracy(StandardDeviations2D visionStdDevs) {
		StandardDeviations2D compensatedStdDevs = new StandardDeviations2D(
			visionStdDevs.xStandardDeviations() * odometryAccuracy,
			visionStdDevs.yStandardDeviations() * odometryAccuracy,
			visionStdDevs.angleStandardDeviations()
		);
		updateOdometryAccuracy(compensatedStdDevs);
		return compensatedStdDevs;
	}

	private void updateOdometryAccuracy(StandardDeviations2D compensatedVisionStdDevs) {
		double averageCompensatedTranslationalStdDevs = (compensatedVisionStdDevs.xStandardDeviations()
			+ compensatedVisionStdDevs.yStandardDeviations()) / 2.0;
		odometryAccuracy += Math.pow(WPILibPoseEstimatorConstants.ODOMETRY_ACCURACY_ADDITION_POWER_BASE, averageCompensatedTranslationalStdDevs);
		odometryAccuracy = Math.min(odometryAccuracy, 1);
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
