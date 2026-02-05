// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

		this.turret = TurretConstants.createTurret();
		this.turretResetCheckSensor = TurretConstants.createTurretResetCheckSensor();
		turret.setPosition(TurretConstants.MIN_POSITION);
		BrakeStateManager.add(() -> turret.setBrake(true), () -> turret.setBrake(false));

		this.flyWheel = FlywheelConstants.createFlyWheel();

		this.fourBar = FourBarConstants.createFourBar();
		this.fourBarResetCheckSensor = FourBarConstants.createFourBarSensorResetCheck();
		fourBar.setPosition(FourBarConstants.MAXIMUM_POSITION);
		BrakeStateManager.add(() -> fourBar.setBrake(true), () -> fourBar.setBrake(false));

		this.hood = HoodConstants.createHood();
		this.hoodResetCheckSensor = HoodConstants.createHoodResetCheckSensor();
		hood.setPosition(HoodConstants.MINIMUM_POSITION);
		BrakeStateManager.add(() -> hood.setBrake(true), () -> hood.setBrake(false));

		this.intakeRoller = IntakeRollerConstants.createIntakeRollers();
		BrakeStateManager.add(() -> intakeRoller.setBrake(true), () -> intakeRoller.setBrake(false));

		this.train = TrainConstant.createTrain();
		BrakeStateManager.add(() -> train.setBrake(true), () -> train.setBrake(false));

		this.belly = BellyConstants.createBelly();
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
		robotCommander.update();

		poseEstimator.updateOdometry(swerve.getAllOdometryData());
		poseEstimator.log();
		ShootingCalculations
			.updateShootingParams(poseEstimator.getEstimatedPose(), swerve.getFieldRelativeVelocity(), swerve.getIMUAngularVelocityRPS()[2]);

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
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
