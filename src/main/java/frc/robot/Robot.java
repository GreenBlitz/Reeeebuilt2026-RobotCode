// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.RobotManager;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.digitalinput.chooser.ChooserDigitalInput;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.statemachine.RobotCommander;
import frc.robot.statemachine.ShootingCalculations;
import frc.robot.statemachine.intakestatehandler.IntakeState;
import frc.robot.statemachine.shooterstatehandler.ShooterState;
import frc.robot.subsystems.arm.ArmSimulationConstants;
import frc.robot.subsystems.arm.VelocityPositionArm;
import frc.robot.subsystems.constants.belly.BellyConstants;
import frc.robot.subsystems.constants.intakeRollers.IntakeRollerConstants;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.TalonFXArmBuilder;
import frc.robot.subsystems.constants.fourBar.FourBarConstants;
import frc.robot.subsystems.constants.hood.HoodConstants;
import frc.robot.subsystems.constants.train.TrainConstant;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.KrakenX60FlyWheelBuilder;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.SparkMaxRollerBuilder;
import frc.robot.subsystems.roller.TalonFXRollerBuilder;
import frc.robot.subsystems.roller.VelocityRoller;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.imu.IMUFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.statemachine.shooterstatehandler.TurretCalculations;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;
import frc.utils.brakestate.BrakeStateManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);
	private final VelocityPositionArm turret;
	private final FlyWheel flyWheel;
	private final Roller intakeRoller;
	private final Arm fourBar;
	private final Arm hood;
	private final IDigitalInput turretResetCheckSensor;
	private final IDigitalInput fourBarResetCheckSensor;
	private final IDigitalInput hoodResetCheckSensor;
	private final VelocityRoller train;
	private final SimulationManager simulationManager;
	private final Roller belly;

	private final RobotCommander robotCommander;

	private final Swerve swerve;
	private final IPoseEstimator poseEstimator;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		this.turret = createTurret();
		this.turretResetCheckSensor = createTurretResetCheckSensor();
		turret.setPosition(TurretConstants.MIN_POSITION);
		BrakeStateManager.add(() -> turret.setBrake(true), () -> turret.setBrake(false));

		this.flyWheel = KrakenX60FlyWheelBuilder.build("Subsystems/FlyWheel", IDs.TalonFXIDs.FLYWHEEL);

		this.fourBar = createFourBar();
		this.fourBarResetCheckSensor = createFourBarSensorResetCheck();
		fourBar.setPosition(FourBarConstants.MAXIMUM_POSITION);
		BrakeStateManager.add(() -> fourBar.setBrake(true), () -> fourBar.setBrake(false));

		this.hood = createHood();
		this.hoodResetCheckSensor = createHoodResetCheckSensor();
		hood.setPosition(HoodConstants.MINIMUM_POSITION);
		BrakeStateManager.add(() -> hood.setBrake(true), () -> hood.setBrake(false));

		this.intakeRoller = createIntakeRollers();
		BrakeStateManager.add(() -> intakeRoller.setBrake(true), () -> intakeRoller.setBrake(false));

		this.train = createTrain();
		BrakeStateManager.add(() -> train.setBrake(true), () -> train.setBrake(false));

		this.belly = createBelly();
		BrakeStateManager.add(() -> belly.setBrake(true), () -> belly.setBrake(false));

		IIMU imu = IMUFactory.createIMU(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			imu,
			IMUFactory.createSignals(imu)
		);

		this.poseEstimator = new WPILibPoseEstimatorWrapper(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
			swerve.getKinematics(),
			swerve.getModules().getWheelPositions(0),
			swerve.getModules().getCurrentStates(),
			swerve.getIMUOrientation(),
			swerve.getIMUAccelerationG().toTranslation2d(),
			swerve.getIMUAbsoluteYaw().getTimestamp()
		);

		robotCommander = new RobotCommander("StateMachine", this);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.getStateHandler().setIsTurretMoveLegalSupplier(() -> isTurretMoveLegal());
		swerve.getStateHandler().setRobotPoseSupplier(() -> poseEstimator.getEstimatedPose());
		swerve.getStateHandler().setTurretAngleSupplier(() -> turret.getPosition());

		simulationManager = new SimulationManager("SimulationManager", this);

		new Trigger(() -> DriverStation.isEnabled()).onTrue(
			(new ParallelCommandGroup(
				robotCommander.getShooterStateHandler().setState(ShooterState.RESET_SUBSYSTEMS),
				robotCommander.getIntakeStateHandler().setState(IntakeState.RESET_FOUR_BAR)
			).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
		);
	}

	public void resetSubsystems() {
		if (DriverStation.isEnabled()) {
			return;
		}

		if (HoodConstants.MINIMUM_POSITION.getRadians() > hood.getPosition().getRadians()) {
			hood.setPosition(HoodConstants.MINIMUM_POSITION);
		}
		if (TurretConstants.MIN_POSITION.getRadians() > turret.getPosition().getRadians()) {
			turret.setPosition(TurretConstants.MIN_POSITION);
		}
		if (FourBarConstants.MAXIMUM_POSITION.getRadians() < fourBar.getPosition().getRadians()) {
			fourBar.setPosition(FourBarConstants.MAXIMUM_POSITION);
		}
	}

	private void updateAllSubsystems() {
		swerve.update();
		fourBar.update();
		intakeRoller.update();
		belly.update();
		train.update();
		turret.update();
		hood.update();
		flyWheel.update();
	}

	public boolean isTurretMoveLegal() {
		return TurretCalculations.isTurretMoveLegal(ShootingCalculations.getShootingParams().targetTurretPosition(), turret.getPosition());
	}

	public void periodic() {
		BusChain.refreshAll();
		updateAllSubsystems();
		resetSubsystems();

		poseEstimator.updateOdometry(swerve.getAllOdometryData());
		poseEstimator.log();
		ShootingCalculations
			.updateShootingParams(poseEstimator.getEstimatedPose(), swerve.getFieldRelativeVelocity(), swerve.getIMUAngularVelocityRPS()[2]);

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	private Roller createIntakeRollers() {
		return SparkMaxRollerBuilder.build(
			RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/IntakeRollers",
			IDs.SparkMAXIDs.INTAKE_ROLLERS,
			IntakeRollerConstants.IS_INVERTED,
			IntakeRollerConstants.GEAR_RATIO,
			IntakeRollerConstants.CURRENT_LIMIT,
			IntakeRollerConstants.MOMENT_OF_INERTIA
		);
	}

	private VelocityPositionArm createTurret() {
		ArmSimulationConstants turretSimulationConstants = new ArmSimulationConstants(
			TurretConstants.MAX_POSITION,
			TurretConstants.MIN_POSITION,
			TurretConstants.MIN_POSITION,
			TurretConstants.MOMENT_OF_INERTIA,
			TurretConstants.TURRET_RADIUS
		);
		return TalonFXArmBuilder.buildVelocityPositionArm(
			TurretConstants.LOG_PATH,
			IDs.TalonFXIDs.TURRET,
			TurretConstants.IS_INVERTED,
			TurretConstants.IS_CONTINUOUS_WRAP,
			TurretConstants.TALON_FX_FOLLOWER_CONFIG,
			TurretConstants.SYS_ID_ROUTINE_CONFIG,
			TurretConstants.FEEDBACK_CONFIGS,
			TurretConstants.REAL_SLOTS_CONFIG,
			TurretConstants.SIMULATION_SLOTS_CONFIG,
			TurretConstants.CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			TurretConstants.ARBITRARY_FEED_FORWARD,
			TurretConstants.FORWARD_SOFTWARE_LIMIT,
			TurretConstants.BACKWARDS_SOFTWARE_LIMIT,
			turretSimulationConstants
		);
	}

	private Arm createFourBar() {
		ArmSimulationConstants fourBarSimConstant = new ArmSimulationConstants(
			FourBarConstants.MAXIMUM_POSITION,
			FourBarConstants.MINIMUM_POSITION,
			FourBarConstants.MAXIMUM_POSITION,
			FourBarConstants.MOMENT_OF_INERTIA,
			FourBarConstants.FOUR_BAR_LENGTH
		);
		return TalonFXArmBuilder.buildDynamicMotionMagicArm(
			FourBarConstants.LOG_PATH,
			IDs.TalonFXIDs.FOUR_BAR,
			FourBarConstants.IS_INVERTED,
			FourBarConstants.IS_CONTINUOUS_WRAP,
			FourBarConstants.TALON_FX_FOLLOWER_CONFIG,
			FourBarConstants.SYS_ID_ROUTINE,
			FourBarConstants.FEEDBACK_CONFIGS,
			FourBarConstants.REAL_SLOT,
			FourBarConstants.SIMULATION_SLOT,
			FourBarConstants.CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			FourBarConstants.ARBITRARY_FEED_FORWARD,
			FourBarConstants.FORWARD_SOFTWARE_LIMITS,
			FourBarConstants.BACKWARD_SOFTWARE_LIMITS,
			fourBarSimConstant,
			FourBarConstants.MAX_ACCELERATION_RPS_SQUARE,
			FourBarConstants.MAX_VELOCITY_RPS
		);
	}

	private Arm createHood() {
		ArmSimulationConstants hoodSimulationConstants = new ArmSimulationConstants(
			HoodConstants.MAXIMUM_POSITION,
			HoodConstants.MINIMUM_POSITION,
			HoodConstants.MINIMUM_POSITION,
			HoodConstants.MOMENT_OF_INERTIA,
			HoodConstants.HOOD_LENGTH_METERS
		);
		return TalonFXArmBuilder.buildDynamicMotionMagicArm(
			RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Hood",
			IDs.TalonFXIDs.HOOD,
			HoodConstants.IS_INVERTED,
			HoodConstants.IS_CONTINUOUS_WRAP,
			new TalonFXFollowerConfig(),
			HoodConstants.SYSIDROUTINE_CONFIG,
			HoodConstants.FEEDBACK_CONFIGS,
			HoodConstants.REAL_SLOT,
			HoodConstants.SIMULATION_SLOT,
			HoodConstants.CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			HoodConstants.ARBITRARY_FEEDFORWARD,
			HoodConstants.FORWARD_SOFTWARE_LIMIT,
			HoodConstants.BACKWARD_SOFTWARE_LIMIT,
			hoodSimulationConstants,
			HoodConstants.DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE,
			HoodConstants.DEFAULT_MAX_VELOCITY_PER_SECOND
		);
	}

	private VelocityRoller createTrain() {
		return TalonFXRollerBuilder.buildVelocityRoller(
			TrainConstant.LOG_PATH,
			IDs.TalonFXIDs.TRAIN,
			TrainConstant.REAL_SLOTS_CONFIG,
			TrainConstant.SIMULATION_SLOTS_CONFIG,
			TrainConstant.CURRENT_LIMIT,
			TrainConstant.FEEDBACK_CONFIGS,
			TrainConstant.MOMENT_OF_INERTIA,
			TrainConstant.IS_INVERTED
		);
	}

	private Roller createBelly() {
		return SparkMaxRollerBuilder.build(
			BellyConstants.LOG_PATH,
			IDs.SparkMAXIDs.BELLY,
			BellyConstants.IS_INVERTED,
			BellyConstants.GEAR_RATIO,
			BellyConstants.CURRENT_LIMIT,
			BellyConstants.MOMENT_OF_INERTIA
		);
	}

	private IDigitalInput createFourBarSensorResetCheck() {
		return ROBOT_TYPE.isReal()
			? new ChanneledDigitalInput(
				new DigitalInput(IDs.DigitalInputsIDs.FOUR_BAR_RESET_SENSOR),
				new Debouncer(FourBarConstants.RESET_CHECK_SENSOR_DEBOUNCE_TIME),
				FourBarConstants.IS_RESET_CHECK_SENSOR_INVERTED
			)
			: new ChooserDigitalInput("intakeResetCheck");
	}

	private IDigitalInput createTurretResetCheckSensor() {
		return ROBOT_TYPE.isReal()
			? new ChanneledDigitalInput(
				new DigitalInput(IDs.DigitalInputsIDs.TURRET_RESET_SENSOR),
				new Debouncer(TurretConstants.RESET_CHECK_SENSOR_DEBOUNCE_TIME),
				TurretConstants.IS_RESET_CHECK_SENSOR_INVERTED
			)
			: new ChooserDigitalInput("turretResetCheck");
	}

	private IDigitalInput createHoodResetCheckSensor() {
		return ROBOT_TYPE.isReal()
			? new ChanneledDigitalInput(
				new DigitalInput(IDs.DigitalInputsIDs.HOOD_RESET_SENSOR),
				new Debouncer(HoodConstants.RESET_CHECK_SENSOR_DEBOUNCE_TIME),
				HoodConstants.IS_RESET_CHECK_SENSOR_INVERTED
			)
			: new ChooserDigitalInput("hoodResetCheck");
	}

	public Roller getIntakeRoller() {
		return intakeRoller;
	}

	public VelocityPositionArm getTurret() {
		return turret;
	}

	public FlyWheel getFlyWheel() {
		return flyWheel;
	}

	public Arm getFourBar() {
		return fourBar;
	}

	public VelocityRoller getTrain() {
		return train;
	}

	public Roller getBelly() {
		return belly;
	}

	public Arm getHood() {
		return hood;
	}

	public IDigitalInput getTurretResetCheckSensor() {
		return turretResetCheckSensor;
	}

	public IDigitalInput getHoodResetCheckSensor() {
		return hoodResetCheckSensor;
	}

	public IDigitalInput getFourBarResetCheckSensor() {
		return fourBarResetCheckSensor;
	}

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public RobotCommander getRobotCommander() {
		return robotCommander;
	}

	public SimulationManager getSimulationManager() {
		return simulationManager;
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

}
