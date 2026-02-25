package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.funnelstatehandler.FunnelState;
import frc.robot.statemachine.funnelstatehandler.FunnelStateHandler;
import frc.robot.statemachine.intakestatehandler.IntakeState;
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

		this.intakeStateHandler = new IntakeStateHandler(robot.getFourBar(), robot.getIntakeRoller(), logPath);
		this.funnelStateHandler = new FunnelStateHandler(robot.getTrain(), robot.getBelly(), logPath, robot.getTrainBallSensor());
		this.shooterStateHandler = new ShooterStateHandler(
			robot.getTurret(),
			robot.getHood(),
			robot.getFlyWheel(),
			ShootingCalculations::getShootingParams,

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
			|| robot.getTurret().isRunningIndependently() || true;
	}

	public void update() {
		intakeStateHandler.periodic();
		funnelStateHandler.periodic();
		shooterStateHandler.periodic();
		Logger.recordOutput(logPath + "/isRunningIndependently", isRunningIndependently());
	}

	public Command setState(RobotState robotState) {
		return asSubsystemCommand(switch (robotState) {
			case STAY_IN_PLACE -> stayInPlace();
			case NEUTRAL -> neutral();
			case PRE_SCORE, PRE_PASS -> preShoot();
			case SCORE, PASS -> shoot();
			case CALIBRATION_PRE_SCORE -> calibrationPreShoot();
			case RESET_SUBSYSTEMS -> resetSubsystems();
			case CALIBRATION_SCORE -> calibrationShoot();
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
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.NEUTRAL),
			funnelStateHandler.setState(FunnelState.ROLL_UNTIL_SENSOR)
		);
	}

	private Command preShoot() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.SHOOT),
			funnelStateHandler.setState(FunnelState.ROLL_UNTIL_SENSOR)
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.SHOOT), funnelStateHandler.setState(FunnelState.SHOOT));
	}

	private Command resetSubsystems() {
		return new ParallelDeadlineGroup(
			new ParallelCommandGroup(
				shooterStateHandler.setState(ShooterState.RESET_SUBSYSTEMS),
				intakeStateHandler.setState(IntakeState.RESET_FOUR_BAR)
			),
			funnelStateHandler.setState(FunnelState.ROLL_UNTIL_SENSOR)
		);
	}

	private Command calibrationPreShoot() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.CALIBRATION),
			funnelStateHandler.setState(FunnelState.ROLL_UNTIL_SENSOR)
		);
	}

	private Command calibrationShoot() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.CALIBRATION), funnelStateHandler.setState(FunnelState.SHOOT));
	}

	public boolean isReadyToScore() {
		return ShootingChecks.isReadyToScore(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_SCORING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_START_SCORING,
			StateMachineConstants.TURRET_TOLERANCE_TO_START_SCORING,
			StateMachineConstants.MAX_DISTANCE_TO_SCORE_METERS
		);
	}

	public boolean isReadyToPass() {
		return ShootingChecks.isReadyToPass(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_PASSING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_START_PASSING,
			StateMachineConstants.TURRET_TOLERANCE_TO_START_PASSING,
			StateMachineConstants.MAX_DISTANCE_TO_PASS_METERS
		);
	}

	public boolean canContinueScoring() {
		return ShootingChecks.canContinueScoring(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_SCORING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_CONTINUE_SCORING,
			StateMachineConstants.TURRET_TOLERANCE_TO_CONTINUE_SCORING,
			StateMachineConstants.MAX_DISTANCE_TO_SCORE_METERS
		);
	}

	public boolean canContinuePassing() {
		return ShootingChecks.canContinuePassing(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_PASSING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_CONTINUE_PASSING,
			StateMachineConstants.TURRET_TOLERANCE_TO_CONTINUE_PASSING,
			StateMachineConstants.MAX_DISTANCE_TO_PASS_METERS
		);
	}

	private boolean calibrationIsReadyToScore() {
		return ShootingChecks.calibrationIsReadyToShoot(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_SCORING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_START_SCORING,
			"Score"
		);
	}

	private boolean calibrationIsReadyToPass() {
		return ShootingChecks.calibrationIsReadyToShoot(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_PASSING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_START_PASSING,
			"Pass"
		);
	}

	private boolean calibrationCanContinueScoring() {
		return ShootingChecks.calibrationCanContinueShooting(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_SCORING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_CONTINUE_SCORING,
			"Scoring"

		);
	}

	public Command scoreSequence() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.SHOOT),
			new RepeatCommand(
				new SequentialCommandGroup(
					asSubsystemCommand(
						funnelStateHandler.setState(FunnelState.ROLL_UNTIL_SENSOR).until(this::isReadyToScore),
						RobotState.PRE_SCORE
					),
					asSubsystemCommand(funnelStateHandler.setState(FunnelState.SHOOT).until(() -> !canContinueScoring()), RobotState.SCORE)
				)
			)
		);
	}

	public Command passSequence() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.SHOOT),
			new RepeatCommand(
				new SequentialCommandGroup(
					new ParallelCommandGroup(
						asSubsystemCommand(
							funnelStateHandler.setState(FunnelState.ROLL_UNTIL_SENSOR).until(this::isReadyToPass),
							RobotState.PRE_PASS
						)
					),
					new ParallelCommandGroup(
						asSubsystemCommand(funnelStateHandler.setState(FunnelState.SHOOT).until(() -> !canContinuePassing()), RobotState.PASS)
					)
				)
			)
		);
	}

	public Command calibrationScoreSequence() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.CALIBRATION),
			new RepeatCommand(
				new SequentialCommandGroup(
					new ParallelCommandGroup(
						asSubsystemCommand(
							funnelStateHandler.setState(FunnelState.ROLL_UNTIL_SENSOR).until(this::calibrationIsReadyToScore),
							RobotState.CALIBRATION_PRE_SCORE
						)
					),
					new ParallelCommandGroup(
						asSubsystemCommand(
							funnelStateHandler.setState(FunnelState.SHOOT).until(() -> !calibrationCanContinueScoring()),
							RobotState.CALIBRATION_SCORE
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
			case NEUTRAL, SCORE, CALIBRATION_PRE_SCORE, CALIBRATION_SCORE, PASS, RESET_SUBSYSTEMS -> driveWith(RobotState.NEUTRAL);
			case PRE_SCORE -> driveWith(RobotState.PRE_SCORE);
			case PRE_PASS -> driveWith(RobotState.PRE_PASS);
		};
	}

	public IntakeStateHandler getIntakeStateHandler() {
		return intakeStateHandler;
	}

}
