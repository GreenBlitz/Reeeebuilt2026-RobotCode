// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.digitalinput.chooser.ChooserDigitalInput;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.statemachine.RobotCommander;
import frc.robot.statemachine.ShootingCalculations;
import frc.robot.subsystems.arm.ArmSimulationConstants;
import frc.robot.subsystems.arm.VelocityPositionArm;
import frc.robot.subsystems.constants.belly.BellyConstants;
import frc.robot.subsystems.constants.FunnelSensorConstants;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.TalonFXArmBuilder;
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
import frc.robot.vision.cameras.limelight.Limelight;
import frc.robot.vision.cameras.limelight.LimelightFilters;
import frc.robot.vision.cameras.limelight.LimelightPipeline;
import frc.robot.vision.cameras.limelight.LimelightStdDevCalculations;
import frc.robot.statemachine.shooterstatehandler.TurretCalculations;
import frc.robot.subsystems.swerve.factories.modules.drive.KrakenX60DriveBuilder;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.math.StandardDeviations2D;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);
	private final VelocityPositionArm turret;
	private final FlyWheel flyWheel;
	private final Arm hood;
	private final IDigitalInput turretResetCheckSensor;
	private final IDigitalInput hoodResetCheckSensor;
	private final VelocityRoller train;
	private final SimulationManager simulationManager;
	private final Roller belly;

	private final RobotCommander robotCommander;

	private final Swerve swerve;
	private final Limelight limelight;
	private final IPoseEstimator poseEstimator;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		this.turret = createTurret();
		this.turretResetCheckSensor = createTurretResetCheckSensor();
		turret.setPosition(TurretConstants.MIN_POSITION);
		BrakeStateManager.add(() -> turret.setBrake(true), () -> turret.setBrake(false));

		this.flyWheel = KrakenX60FlyWheelBuilder.build("Subsystems/FlyWheel", IDs.TalonFXIDs.FLYWHEEL);

		this.hood = createHood();
		this.hoodResetCheckSensor = createHoodResetCheckSensor();
		hood.setPosition(HoodConstants.MINIMUM_POSITION);
		BrakeStateManager.add(() -> hood.setBrake(true), () -> hood.setBrake(false));

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

		this.limelight = new Limelight(
			"limelight-new",
			"Vision",
			new Pose3d(
				new Translation3d(-0.017, 0.357, 0.287),
				new Rotation3d(Math.toRadians(2.44), Math.toRadians(29.15), Math.toRadians(-92.69))
			),
			LimelightPipeline.APRIL_TAG
		);

		limelight.setMT1StdDevsCalculation(
			LimelightStdDevCalculations.getMT1StdDevsCalculation(
				limelight,
				new StandardDeviations2D(0.5),
				new StandardDeviations2D(0.05),
				new StandardDeviations2D(0.5),
				new StandardDeviations2D(-0.02)
			)
		);
		limelight.setMT1PoseFilter(
			LimelightFilters.megaTag1Filter(
				limelight,
				timestamp -> poseEstimator.getEstimatedPoseAtTimestamp(timestamp).map(Pose2d::getRotation),
				poseEstimator::isIMUOffsetCalibrated,
				new Translation2d(0.1, 0.1),
				Rotation2d.fromDegrees(10)
			)
		);

		robotCommander = new RobotCommander("StateMachine", this);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.getStateHandler().setIsTurretMoveLegalSupplier(() -> isTurretMoveLegal());
		swerve.getStateHandler().setRobotPoseSupplier(() -> poseEstimator.getEstimatedPose());
		swerve.getStateHandler().setTurretAngleSupplier(() -> turret.getPosition());

		simulationManager = new SimulationManager("SimulationManager", this);

//		new Trigger(() -> DriverStation.isEnabled()).onTrue(
//			(new ParallelCommandGroup(
//				robotCommander.getShooterStateHandler().setState(ShooterState.RESET_SUBSYSTEMS)).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
//		);

		swerve.configPathPlanner(() -> poseEstimator.getEstimatedPose(), (pose) -> {}, getRobotConfig());
	}

	private void updateAllSubsystems() {
		swerve.update();
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
		robotCommander.update();

		poseEstimator.updateOdometry(swerve.getAllOdometryData());
		limelight.updateIsConnected();
		limelight.updateMT1();
		limelight.getIndependentRobotPose().ifPresent(poseEstimator::updateVision);
		poseEstimator.log();
		ShootingCalculations
			.updateShootingParams(poseEstimator.getEstimatedPose(), swerve.getFieldRelativeVelocity(), swerve.getIMUAngularVelocityRPS()[2]);

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public RobotConfig getRobotConfig() {
		return new RobotConfig(
			60,
			6.325,
			new ModuleConfig(
				swerve.getModules().getModule(ModuleUtil.ModulePosition.FRONT_LEFT).getConstants().wheelDiameterMeters() / 2,
				swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
				0.96,
				DCMotor.getKrakenX60Foc(1),
				KrakenX60DriveBuilder.GEAR_RATIO,
				KrakenX60DriveBuilder.SLIP_CURRENT,
				1
			),
			swerve.getModules().getModulePositionsFromCenterMeters()
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

	private IDigitalInput createFunnelDI() {
		return ROBOT_TYPE.isSimulation()
			? new ChooserDigitalInput("funnelSensor")
			: new ChanneledDigitalInput(
				new DigitalInput(FunnelSensorConstants.CHANNEL),
				FunnelSensorConstants.DEBOUNCER,
				FunnelSensorConstants.INVERTED
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
		return TalonFXArmBuilder.buildVelocityPositionArm(
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
			hoodSimulationConstants
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

	public VelocityPositionArm getTurret() {
		return turret;
	}

	public FlyWheel getFlyWheel() {
		return flyWheel;
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
