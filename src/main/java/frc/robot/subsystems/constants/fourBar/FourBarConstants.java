package frc.robot.subsystems.constants.fourBar;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.arm.ArmSimulationConstants;
import frc.robot.subsystems.arm.CurrentControlArm;
import frc.robot.subsystems.arm.TalonFXArmBuilder;

public class FourBarConstants {

	public static final String LOG_PATH = RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/FourBar";

	public static final boolean IS_INVERTED = false;
	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();
	public static final boolean IS_CONTINUOUS_WRAP = false;

	public static final int CURRENT_LIMIT = 15;

	public static final Slot0Configs REAL_SLOT = new Slot0Configs();
	public static final Slot0Configs SIMULATION_SLOT = new Slot0Configs();
	public static final TalonFXFollowerConfig TALON_FX_FOLLOWER_CONFIG = new TalonFXFollowerConfig();

	static {
		FEEDBACK_CONFIGS.RotorToSensorRatio = 1;
		FEEDBACK_CONFIGS.SensorToMechanismRatio = 140;

		REAL_SLOT.kP = 0;
		REAL_SLOT.kI = 0;
		REAL_SLOT.kD = 0;
		REAL_SLOT.kS = 0;
		REAL_SLOT.kG = 0;
		REAL_SLOT.kV = 0;
		REAL_SLOT.kA = 0;
		REAL_SLOT.GravityType = GravityTypeValue.Arm_Cosine;

		SIMULATION_SLOT.kP = 50;
		SIMULATION_SLOT.kI = 0;
		SIMULATION_SLOT.kD = 0;
		SIMULATION_SLOT.kG = 0;
		SIMULATION_SLOT.kS = 0;
		SIMULATION_SLOT.GravityType = GravityTypeValue.Arm_Cosine;
	}

	public static final Rotation2d MAXIMUM_POSITION = Rotation2d.fromDegrees(80.15);
	public static final Rotation2d MINIMUM_POSITION = Rotation2d.fromDegrees(20.85);

	public static final Rotation2d FORWARD_SOFTWARE_LIMITS = MAXIMUM_POSITION.minus(Rotation2d.fromDegrees(1.5));
	public static final Rotation2d BACKWARD_SOFTWARE_LIMITS = MINIMUM_POSITION.plus(Rotation2d.fromDegrees(1.5));

	public static final double CLOSE_VOLTAGE = 7;
	public static final double CLOSE_STALL_CURRENT_AMP = 13;
	public static final double CURRENT_TO_HOLD_INTAKE_CLOSED = 3.0;

	public static final double INTAKE_OPEN_VOLTAGE = -5.0;
	public static final double RELAXED_CURRENT_AMP = -3.5;
	public static final double HOLD_CURRENT_AMP = -6.0;

	public static final double FOUR_BAR_RESET_VOLTAGE = 2;
	public static final double CURRENT_THRESHOLD_TO_RESET_POSITION = 15;

	public static final double FOUR_BAR_LENGTH = 0.3;
	public static final double MOMENT_OF_INERTIA = 0.001;
	public static final SysIdRoutine.Config SYS_ID_ROUTINE = new SysIdRoutine.Config();


	public static CurrentControlArm createFourBar() {
		ArmSimulationConstants fourBarSimConstant = new ArmSimulationConstants(
			MAXIMUM_POSITION,
			MINIMUM_POSITION,
			MAXIMUM_POSITION,
			MOMENT_OF_INERTIA,
			FOUR_BAR_LENGTH
		);
		return TalonFXArmBuilder.buildCurrentControlArm(
			LOG_PATH,
			IDs.TalonFXIDs.FOUR_BAR,
			IS_INVERTED,
			IS_CONTINUOUS_WRAP,
			TALON_FX_FOLLOWER_CONFIG,
			SYS_ID_ROUTINE,
			FEEDBACK_CONFIGS,
			REAL_SLOT,
			SIMULATION_SLOT,
			CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			FORWARD_SOFTWARE_LIMITS,
			BACKWARD_SOFTWARE_LIMITS,
			fourBarSimConstant
		);
	}

}
