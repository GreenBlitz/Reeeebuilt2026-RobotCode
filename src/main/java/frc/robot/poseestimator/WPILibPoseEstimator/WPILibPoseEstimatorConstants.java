package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.math.StandardDeviations2D;


public class WPILibPoseEstimatorConstants {

	public static final String WPILIB_POSEESTIMATOR_LOGPATH = "WPILibPoseEstimator";

	public static final StandardDeviations2D DEFAULT_ODOMETRY_STD_DEV = new StandardDeviations2D(0.003, 0.003, 0.003);

	public static final StandardDeviations2D DEFAULT_VISION_STD_DEV = new StandardDeviations2D(0.0003, 0.0003, 0.003);

	public static final double MINIMUM_COLLISION_IMU_ACCELERATION_G = 2;

	public static final Rotation2d MINIMUM_TILT_IMU_ROLL = Rotation2d.fromDegrees(4);

	public static final Rotation2d MINIMUM_TILT_IMU_PITCH = Rotation2d.fromDegrees(4);

	public static final double MINIMUM_SKID_ROBOT_TO_MODULE_VELOCITY_DIFFERENCE_METERS_PER_SECOND = 0.5;

	public static final double COLLISION_ODOMETRY_ACCURACY_REDUCTION_FACTOR = 0;

	public static final double TILT_ODOMETRY_ACCURACY_REDUCTION_FACTOR = 0;

	public static final double SKID_ODOMETRY_ACCURACY_REDUCTION_FACTOR = 0;

	public static final double CALIBRATED_IMU_OFFSET_VISION_STD_DEVS_ADDITION = 0;

	/**
	 * constant represents the addition to odometryAccuracy when the compensated vision X and Y StdDevs average is equal to one
	 */
	public static double ODOMETRY_ACCURACY_ADDITION_POWER_BASE = 0;

	public static final Rotation2d INITIAL_IMU_YAW = new Rotation2d();

	public static final Pose2d STARTING_ODOMETRY_POSE = new Pose2d();

	public static final int POSE_TO_IMU_YAW_DIFFERENCE_BUFFER_SIZE = 50;

	public static double MAX_POSE_TO_IMU_YAW_DIFFERENCE_STD_DEV = 0.0025;

	public static double IMU_YAW_BUFFER_SIZE_SECONDS = 2;

}
