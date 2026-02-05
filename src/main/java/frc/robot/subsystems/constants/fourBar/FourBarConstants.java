package frc.robot.subsystems.constants.fourBar;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.digitalinput.chooser.ChooserDigitalInput;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSimulationConstants;
import frc.robot.subsystems.arm.TalonFXArmBuilder;

public class FourBarConstants {

	public static final String LOG_PATH = RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/FourBar";
	public static final int CURRENT_LIMIT = 40;
	public static final double MOMENT_OF_INERTIA = 0.001;
	public static final double FOUR_BAR_LENGTH = 0.3;
	public static final double ARBITRARY_FEED_FORWARD = 0.0;
	public static final boolean IS_INVERTED = false;
	public static final TalonFXFollowerConfig TALON_FX_FOLLOWER_CONFIG = new TalonFXFollowerConfig();
	public static final SysIdRoutine.Config SYS_ID_ROUTINE = new SysIdRoutine.Config();
	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();
	public static final Slot0Configs REAL_SLOT = new Slot0Configs();
	public static final Slot0Configs SIMULATION_SLOT = new Slot0Configs();

	static {
		REAL_SLOT.kP = 28;
		REAL_SLOT.kI = 0;
		REAL_SLOT.kD = 0;
		REAL_SLOT.kS = 0.065;
		REAL_SLOT.kG = 0.37;
		REAL_SLOT.kV = 9.0000095367432;
		REAL_SLOT.kA = 0.5209;
		REAL_SLOT.GravityType = GravityTypeValue.Arm_Cosine;

		SIMULATION_SLOT.kP = 50;
		SIMULATION_SLOT.kI = 0;
		SIMULATION_SLOT.kD = 0;
		SIMULATION_SLOT.kG = 0;
		SIMULATION_SLOT.kS = 0;
		SIMULATION_SLOT.GravityType = GravityTypeValue.Arm_Cosine;

		FEEDBACK_CONFIGS.RotorToSensorRatio = 1;
		FEEDBACK_CONFIGS.SensorToMechanismRatio = 450 / 7.0;
	}

	public static final Rotation2d FORWARD_SOFTWARE_LIMITS = Rotation2d.fromDegrees(91);
	public static final Rotation2d BACKWARD_SOFTWARE_LIMITS = Rotation2d.fromDegrees(10);
	public static final Rotation2d MAXIMUM_POSITION = Rotation2d.fromDegrees(100);
	public static final Rotation2d MINIMUM_POSITION = Rotation2d.fromDegrees(0);
	public static final Rotation2d MAX_ACCELERATION_RPS_SQUARE = Rotation2d.fromRotations(3);
	public static final Rotation2d MAX_VELOCITY_RPS = Rotation2d.fromRotations(3);
	public static final boolean IS_CONTINUOUS_WRAP = false;
	public final static double RESET_CHECK_SENSOR_DEBOUNCE_TIME = 0.15;
	public final static boolean IS_RESET_CHECK_SENSOR_INVERTED = false;
	public static final double FOUR_BAR_RESET_VOLTAGE = -1;

	public static Arm createFourBar() {
		ArmSimulationConstants fourBarSimConstant = new ArmSimulationConstants(
			MAXIMUM_POSITION,
			MINIMUM_POSITION,
			MAXIMUM_POSITION,
			MOMENT_OF_INERTIA,
			FOUR_BAR_LENGTH
		);
		return TalonFXArmBuilder.buildDynamicMotionMagicArm(
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
			ARBITRARY_FEED_FORWARD,
			FORWARD_SOFTWARE_LIMITS,
			BACKWARD_SOFTWARE_LIMITS,
			fourBarSimConstant,
			MAX_ACCELERATION_RPS_SQUARE,
			MAX_VELOCITY_RPS
		);
	}

	public static IDigitalInput createFourBarSensorResetCheck() {
		return Robot.ROBOT_TYPE.isReal()
			? new ChanneledDigitalInput(
				new DigitalInput(IDs.DigitalInputsIDs.FOUR_BAR_RESET_SENSOR),
				new Debouncer(RESET_CHECK_SENSOR_DEBOUNCE_TIME),
				IS_RESET_CHECK_SENSOR_INVERTED
			)
			: new ChooserDigitalInput("intakeResetCheck");
	}

}
