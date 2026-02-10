// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.RobotManager;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.statemachine.RobotCommander;
import frc.robot.statemachine.ShootingCalculations;
import frc.robot.statemachine.intakestatehandler.IntakeState;
import frc.robot.statemachine.shooterstatehandler.ShooterState;
import frc.robot.subsystems.arm.VelocityPositionArm;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.constants.belly.BellyConstants;
import frc.robot.subsystems.constants.flywheel.FlywheelConstants;
import frc.robot.subsystems.constants.fourBar.FourBarConstants;
import frc.robot.subsystems.constants.hood.HoodConstants;
import frc.robot.subsystems.constants.intakeRollers.IntakeRollerConstants;
import frc.robot.subsystems.constants.train.TrainConstant;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.SparkMaxRollerBuilder;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.imu.IMUFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.vision.cameras.limelight.Limelight;
import frc.robot.vision.cameras.limelight.LimelightFilters;
import frc.robot.vision.cameras.limelight.LimelightPipeline;
import frc.robot.vision.cameras.limelight.LimelightStdDevCalculations;
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
	private final Arm turret;
	private final FlyWheel flyWheel;
	private final Roller intakeRoller;
	private final Arm fourBar;
	private final Arm hood;
	private final IDigitalInput intakeRollerSensor;
	private final Roller train;
	private final IDigitalInput funnelDigitalInput;
	private final SimulationManager simulationManager;
	private final Roller belly;

	private final RobotCommander robotCommander;

	private final Swerve swerve;
	private final IPoseEstimator poseEstimator;

	private final Limelight limelight;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		this.turret = createTurret();
		turret.setPosition(TurretConstants.MIN_POSITION);
		BrakeStateManager.add(() -> turret.setBrake(true), () -> turret.setBrake(false));

		this.flyWheel = KrakenX60FlyWheelBuilder.build("Subsystems/FlyWheel", IDs.TalonFXIDs.FLYWHEEL);

		this.fourBar = createFourBar();
		fourBar.setPosition(FourBarConstants.MAXIMUM_POSITION);
		BrakeStateManager.add(() -> fourBar.setBrake(true), () -> fourBar.setBrake(false));

		this.hood = createHood();
		hood.setPosition(HoodConstants.MINIMUM_POSITION);
		BrakeStateManager.add(() -> hood.setBrake(true), () -> hood.setBrake(false));

		Pair<Roller, IDigitalInput> intakeRollerAndDigitalInput = createIntakeRollers();
		this.intakeRoller = intakeRollerAndDigitalInput.getFirst();
		this.intakeRollerSensor = intakeRollerAndDigitalInput.getSecond();
		BrakeStateManager.add(() -> intakeRoller.setBrake(true), () -> intakeRoller.setBrake(false));

		Pair<Roller, IDigitalInput> trainAndDigitalInput = createTrainAndSignal();
		this.train = trainAndDigitalInput.getFirst();
		this.funnelDigitalInput = trainAndDigitalInput.getSecond();
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
			"limelight-left",
			"Vision",
			new Pose3d(0.268, 0.003, 0.244, new Rotation3d(Math.toRadians(0.13), Math.toRadians(27.68), Math.toRadians(1.37))),
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

		robotCommander = new RobotCommander("/RobotCommander", this);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.getStateHandler().setRobotPoseSupplier(() -> poseEstimator.getEstimatedPose());
		swerve.getStateHandler().setTurretAngleSupplier(() -> turret.getPosition());

		simulationManager = new SimulationManager("SimulationManager", this);
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

	public void periodic() {
		BusChain.refreshAll();
		updateAllSubsystems();
		resetSubsystems();
		simulationManager.logPoses();

		swerve.update();
		poseEstimator.updateOdometry(swerve.getAllOdometryData());
		limelight.updateMT1();
		limelight.getIndependentRobotPose().ifPresent(poseEstimator::updateVision);
		poseEstimator.log();
		ShootingCalculations.updateShootingParams(poseEstimator.getEstimatedPose());

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public Roller getIntakeRoller() {
		return intakeRoller;
	}

	public Arm getTurret() {
		return turret;
	}

	public FlyWheel getFlyWheel() {
		return flyWheel;
	}

	public Arm getFourBar() {
		return fourBar;
	}

	public Roller getTrain() {
		return train;
	}

	public Roller getBelly() {
		return belly;
	}

	public IDigitalInput getFunnelDigitalInput() {
		return funnelDigitalInput;
	}

	public Arm getHood() {
		return hood;
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

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

}
