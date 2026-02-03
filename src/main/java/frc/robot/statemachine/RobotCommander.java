package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.funnelstatehandler.FunnelState;
import frc.robot.statemachine.funnelstatehandler.FunnelStateHandler;
import frc.robot.statemachine.intakestatehandler.IntakeStateHandler;
import frc.robot.statemachine.shooterstatehandler.ShooterState;
import frc.robot.statemachine.shooterstatehandler.ShooterStateHandler;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

public class RobotCommander extends GBSubsystem {

	private final Robot robot;
	private final Swerve swerve;
	private final IntakeStateHandler intakeStateHandler;
	private final FunnelStateHandler funnelStateHandler;
	private final ShooterStateHandler shooterStateHandler;

	private RobotState currentState;
	private final String logPath;

	public RobotCommander(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();

		this.logPath = logPath;

		this.intakeStateHandler = new IntakeStateHandler(
			robot.getFourBar(),
			robot.getIntakeRoller(),
			robot.getFourBarResetCheckSensor(),
			logPath
		);
		this.funnelStateHandler = new FunnelStateHandler(robot.getTrain(), robot.getBelly(), logPath);
		this.shooterStateHandler = new ShooterStateHandler(
			robot.getTurret(),
			robot.getHood(),
			robot.getFlyWheel(),
			ShootingCalculations::getShootingParams,
			robot.getTurretResetCheckSensor(),
			robot.getHoodResetCheckSensor(),
			logPath
		);

		this.currentState = RobotState.STAY_IN_PLACE;
		Logger.recordOutput(logPath + "/CurrentState", RobotState.STAY_IN_PLACE);

		setDefaultCommand(
			new ConditionalCommand(
				asSubsystemCommand(Commands.none(), "Disabled"),
				new InstantCommand(
					() -> CommandScheduler.getInstance()
						.schedule(
							new DeferredCommand(
								() -> endState(currentState),
								Set.of(this, swerve, robot.getTurret(), robot.getHood(), robot.getTrain(), robot.getBelly(), robot.getFlyWheel())
							)
						)
				),
				this::isRunningIndependently
			)
		);
	}

	public FunnelStateHandler getFunnelStateHandler() {
		return funnelStateHandler;
	}

	public ShooterStateHandler getShooterStateHandler() {
		return shooterStateHandler;
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public boolean isRunningIndependently() {
		return swerve.isRunningIndependently()
			|| robot.getFlyWheel().isRunningIndependently()
			|| robot.getHood().isRunningIndependently()
			|| robot.getTrain().isRunningIndependently()
			|| robot.getBelly().isRunningIndependently()
			|| robot.getTurret().isRunningIndependently();
	}

	@Override
	protected void subsystemPeriodic() {
		intakeStateHandler.periodic();
		funnelStateHandler.periodic();
		shooterStateHandler.periodic();
		Logger.recordOutput(logPath + "/isRunningIndependently", isRunningIndependently());
	}

	private Command setState(RobotState robotState) {
		return asSubsystemCommand(switch (robotState) {
			case STAY_IN_PLACE -> stayInPlace();
			case NEUTRAL -> neutral();
			case PRE_SHOOT -> preShoot();
			case SHOOT -> shoot();
			case RESET_SUBSYSTEMS -> resetSubsystems();
			case CALIBRATION_PRE_SHOOT -> calibrationPreShoot();
			case CALIBRATION_SHOOT -> calibrationShoot();
		}, robotState);
	}

	public Command driveWith(RobotState state, Command command) {
		Command swerveDriveCommand = swerve.getCommandsBuilder().driveByDriversInputs(state.getSwerveState());
		Command wantedCommand = command.deadlineFor(swerveDriveCommand);
		return asSubsystemCommand(wantedCommand, state);
	}

	public Command driveWith(RobotState state) {
		return driveWith(state, setState(state));
	}

	private Command stayInPlace() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.STAY_IN_PLACE), funnelStateHandler.setState(FunnelState.STOP));
	}

	private Command neutral() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.NEUTRAL), funnelStateHandler.setState(FunnelState.NEUTRAL));
	}

	private Command preShoot() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.SHOOT), funnelStateHandler.setState(FunnelState.NEUTRAL));
	}

	private Command shoot() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.SHOOT), funnelStateHandler.setState(FunnelState.SHOOT));
	}

	private Command resetSubsystems() {
		return new ParallelDeadlineGroup(
			shooterStateHandler.setState(ShooterState.RESET_SUBSYSTEMS),
			funnelStateHandler.setState(FunnelState.NEUTRAL)
		);
	}

	private Command calibrationPreShoot() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.CALIBRATION),
			funnelStateHandler.setState(FunnelState.NEUTRAL)
		);
	}

	private Command calibrationShoot() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.CALIBRATION), funnelStateHandler.setState(FunnelState.SHOOT));
	}

	private boolean isReadyToShoot() {
		return ShootingChecks.isReadyToShoot(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_SHOOTING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_START_SHOOTING,
			StateMachineConstants.TURRET_LOOK_AT_HUB_TOLERANCE_TO_START_SHOOTING,
			StateMachineConstants.MAX_ANGLE_FROM_GOAL_CENTER,
			StateMachineConstants.MAX_DISTANCE_TO_SHOOT_METERS
		);
	}

	private boolean canContinueShooting() {
		return ShootingChecks.canContinueShooting(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_SHOOTING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_CONTINUE_SHOOTING,
			StateMachineConstants.TURRET_LOOK_AT_HUB_TOLERANCE_TO_CONTINUE_SHOOTING,
			StateMachineConstants.MAX_ANGLE_FROM_GOAL_CENTER,
			StateMachineConstants.MAX_DISTANCE_TO_SHOOT_METERS
		);
	}

	private boolean calibrationIsReadyToShoot() {
		return ShootingChecks.calibrationIsReadyToShoot(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_SHOOTING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_START_SHOOTING
		);
	}

	private boolean calibrationCanContinueShooting() {
		return ShootingChecks.calibrationCanContinueShooting(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_SHOOTING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_CONTINUE_SHOOTING

		);
	}

	public Command shootSequence() {
		return new ParallelCommandGroup(
			swerve.getCommandsBuilder().driveByDriversInputs(RobotState.SHOOT.getSwerveState()),
			shooterStateHandler.setState(ShooterState.SHOOT),
			new RepeatCommand(
				new SequentialCommandGroup(
					new ParallelCommandGroup(
						asSubsystemCommand(funnelStateHandler.setState(FunnelState.NEUTRAL).until(this::isReadyToShoot), RobotState.PRE_SHOOT)
					),
					new ParallelCommandGroup(
						asSubsystemCommand(funnelStateHandler.setState(FunnelState.SHOOT).until(() -> !canContinueShooting()), RobotState.SHOOT)
					)
				)
			)
		);
	}

	public Command calibrationShootSequence() {
		return new ParallelCommandGroup(
			swerve.getCommandsBuilder().driveByDriversInputs(RobotState.SHOOT.getSwerveState()),
			shooterStateHandler.setState(ShooterState.CALIBRATION),
			new RepeatCommand(
				new SequentialCommandGroup(
					new ParallelCommandGroup(
						asSubsystemCommand(
							funnelStateHandler.setState(FunnelState.NEUTRAL).until(this::calibrationIsReadyToShoot),
							RobotState.PRE_SHOOT
						)
					),
					new ParallelCommandGroup(
						asSubsystemCommand(
							funnelStateHandler.setState(FunnelState.SHOOT).until(() -> !calibrationCanContinueShooting()),
							RobotState.SHOOT
						)
					)
				)
			)
		);
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(
			asSubsystemCommand(command, state.name()),
			new InstantCommand(() -> currentState = state),
			new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentState", state))
		);
	}

	private Command endState(RobotState state) {
		return switch (state) {
			case STAY_IN_PLACE -> driveWith(RobotState.STAY_IN_PLACE);
			case NEUTRAL, SHOOT, CALIBRATION_PRE_SHOOT, CALIBRATION_SHOOT, RESET_SUBSYSTEMS -> driveWith(RobotState.NEUTRAL);
			case PRE_SHOOT -> driveWith(RobotState.PRE_SHOOT);
		};
	}

	public IntakeStateHandler getIntakeStateHandler() {
		return intakeStateHandler;
	}

}
