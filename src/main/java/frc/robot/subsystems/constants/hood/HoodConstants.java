package frc.robot.subsystems.constants.hood;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class HoodConstants {

	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();
	public static final Slot0Configs REAL_SLOT = new Slot0Configs();
	public static final Slot0Configs SIMULATION_SLOT = new Slot0Configs();

	static {
		REAL_SLOT.kP = 350;
		REAL_SLOT.kI = 0;
		REAL_SLOT.kD = 10;
		REAL_SLOT.kS = 0.4;
		REAL_SLOT.kG = 0;
		REAL_SLOT.kV = 0;
		REAL_SLOT.kA = 0;
		REAL_SLOT.GravityType = GravityTypeValue.Elevator_Static;

		SIMULATION_SLOT.kP = 150;
		SIMULATION_SLOT.kI = 0;
		SIMULATION_SLOT.kD = 0;
		SIMULATION_SLOT.kG = 0;
		SIMULATION_SLOT.kS = 0;
		SIMULATION_SLOT.GravityType = GravityTypeValue.Elevator_Static;

		FEEDBACK_CONFIGS.RotorToSensorRatio = 1;
		FEEDBACK_CONFIGS.SensorToMechanismRatio = 32.95 * 4;
	}

	public static final boolean IS_INVERTED = true;
	public static final double ARBITRARY_FEEDFORWARD = 0;
	public static final double CURRENT_LIMIT = 20;
	public static final double MOMENT_OF_INERTIA = 0.001;
	public static final double HOOD_LENGTH_METERS = 0.3;
	public static final Rotation2d FORWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(60);
	public static final Rotation2d BACKWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(30);
	public static final Rotation2d MINIMUM_POSITION = Rotation2d.fromDegrees(25.4);
	public static final Rotation2d MAXIMUM_POSITION = Rotation2d.fromDegrees(60.0);
	public static final Rotation2d DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE = Rotation2d.fromRotations(3);
	public static final Rotation2d DEFAULT_MAX_VELOCITY_PER_SECOND = Rotation2d.fromRotations(3);
	public static final Rotation2d HOOD_POSITION_TOLERANCE = Rotation2d.fromDegrees(1);
	public static final SysIdRoutine.Config SYSIDROUTINE_CONFIG = new SysIdRoutine.Config();
	public static final boolean IS_CONTINUOUS_WRAP = false;

}

