package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.Matrix;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.vision.RobotPoseObservation;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.OdometryData;
import frc.robot.vision.cameras.limelight.Limelight;
import frc.utils.buffers.RingBuffer.RingBuffer;
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
	private final TimeInterpolatableBuffer<Translation2d> imuXYAccelerationGBuffer;
	private RobotPoseObservation lastVisionObservation;
	private OdometryData lastOdometryData;
	private boolean isIMUOffsetCalibrated;

	public boolean[] areSame = new boolean[3];
	public RobotPoseObservation[] goodPoses = new RobotPoseObservation[3];
	public Pose2d bestPose = new Pose2d();

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
	public void updateVision(Limelight ll, RobotPoseObservation... visionRobotPoseObservations) {
		for (RobotPoseObservation visionRobotPoseObservation : visionRobotPoseObservations) {
			updateVision(visionRobotPoseObservation, ll);
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
		if (lastVisionObservation != null) {
			Logger.recordOutput(logPath + "/lastVisionUpdate", lastVisionObservation.timestampSeconds());
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

	private void updateVision(RobotPoseObservation visionRobotPoseObservation, Limelight ll) {
		addVisionMeasurement(visionRobotPoseObservation);

		double xdiff = Math.abs(lastVisionObservation.robotPose().getX() - getEstimatedPose().getX());
		double ydiff = Math.abs(lastVisionObservation.robotPose().getY() - getEstimatedPose().getY());
		double angdiff = Math.abs(lastVisionObservation.robotPose().getRotation().getDegrees() - getEstimatedPose().getRotation().getDegrees());

		if (ll.getName().equals("limelight-front")) {
			areSame[0] = xdiff + ydiff < 1;
			if (areSame[0]) {
				goodPoses[0] = lastVisionObservation;
			} else {
				goodPoses[0] = null;
			}
		}
		if (ll.getName().equals("limelight-right")) {
			areSame[1] = xdiff + ydiff < 1;
			if (areSame[1]) {
				goodPoses[1] = lastVisionObservation;
			} else {
				goodPoses[1] = null;
			}
		}
		if (ll.getName().equals("limelight-left")) {
			areSame[2] = xdiff + ydiff < 1;
			if (areSame[2]) {
				goodPoses[2] = lastVisionObservation;
			} else {
				goodPoses[2] = null;
			}
		}
		Logger.recordOutput("areSame", areSame);

		int c = 0;
		Pose2d maybeBest = new Pose2d();
		for (int i = 0; i < goodPoses.length; i++) {
			if (goodPoses[i] != null) {
				c++;
				maybeBest = new Pose2d(
					maybeBest.getX() + goodPoses[i].robotPose().getX(),
					maybeBest.getY() + goodPoses[i].robotPose().getY(),
					maybeBest.getRotation().plus(goodPoses[i].robotPose().getRotation())
				);
			}
		}
		if (c > 1) {
			bestPose = new Pose2d(maybeBest.getX() / c, maybeBest.getY() / c, maybeBest.getRotation().div(c));
		}
		Logger.recordOutput("bestPose", bestPose);

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

	private void addVisionMeasurement(RobotPoseObservation visionObservation) {
		poseEstimator.addVisionMeasurement(
			visionObservation.robotPose(),
			visionObservation.timestampSeconds(),
			getCollisionCompensatedVisionStdDevs(visionObservation)
		);
		this.lastVisionObservation = visionObservation;
	}

	private Matrix<N3, N1> getCollisionCompensatedVisionStdDevs(RobotPoseObservation visionObservation) {
		boolean isColliding = imuXYAccelerationGBuffer.getSample(visionObservation.timestampSeconds())
			.map(
				(imuAccelerationG) -> PoseUtil
					.getIsColliding(imuAccelerationG, WPILibPoseEstimatorConstants.MINIMUM_COLLISION_IMU_ACCELERATION_G)
			)
			.orElse(false);

		return isColliding
			? visionObservation.stdDevs()
				.asColumnVector()
				.minus(WPILibPoseEstimatorConstants.VISION_STD_DEV_COLLISION_REDUCTION.asColumnVector())
			: visionObservation.stdDevs().asColumnVector();
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

}
