package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Robot;
import frc.robot.statemachine.intakestatehandler.IntakeStateHandler;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;

import frc.robot.subsystems.swerve.Swerve;

import java.util.Set;

public class RobotCommander extends GBSubsystem {

	private final Robot robot;
	private final Swerve swerve;
	private final Superstructure superstructure;
	private final IntakeStateHandler intakeStateHandler;

	private RobotState currentState;

	public RobotCommander(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.superstructure = new Superstructure("StateMachine/Superstructure", robot, () -> ShootingCalculations.getShootingParams());
		this.intakeStateHandler = new IntakeStateHandler(
			robot.getFourBar(),
			robot.getIntakeRoller(),
			robot.getIntakeRollerSensor(),
			"/IntakeStateHandler"
		);
		this.currentState = RobotState.STAY_IN_PLACE;

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

	public RobotState getCurrentState() {
		return currentState;
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}

	public boolean isRunningIndependently() {
		return superstructure.isRunningIndependently() || swerve.isRunningIndependently();
	}

	@Override
	protected void subsystemPeriodic() {
		superstructure.periodic();
	}

	public Command driveWith(RobotState state, Command command) {
		Command swerveDriveCommand = swerve.getCommandsBuilder().driveByDriversInputs(state.getSwerveState());
		Command wantedCommand = command.deadlineFor(swerveDriveCommand);
		return asSubsystemCommand(wantedCommand, state);
	}

	public Command driveWith(RobotState state) {
		return driveWith(state, superstructure.setState(state));
	}

	private boolean isReadyToScore() {
		return ShootingChecks.isReadyToScore(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_SCORING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_START_SCORING,
			StateMachineConstants.TURRET_TOLERANCE_TO_START_SCORING,
			StateMachineConstants.MAX_ANGLE_FROM_GOAL_CENTER,
			StateMachineConstants.MAX_DISTANCE_TO_SCORE_METERS
		);
	}

	private boolean isReadyToPass() {
		return ShootingChecks.isReadyToPass(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_PASSING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_START_PASSING,
			StateMachineConstants.TURRET_TOLERANCE_TO_START_PASSING,
			StateMachineConstants.MAX_ANGLE_FROM_GOAL_CENTER,
			StateMachineConstants.MAX_DISTANCE_TO_SCORE_METERS
		);
	}

	private boolean canContinueScoring() {
		return ShootingChecks.canContinueScoring(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_SCORING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_CONTINUE_SCORING,
			StateMachineConstants.TURRET_TOLERANCE_TO_CONTINUE_SCORING,
			StateMachineConstants.MAX_ANGLE_FROM_GOAL_CENTER,
			StateMachineConstants.MAX_DISTANCE_TO_SCORE_METERS
		);
	}

	private boolean canContinuePassing() {
		return ShootingChecks.canContinuePassing(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_PASSING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_CONTINUE_PASSING,
			StateMachineConstants.TURRET_TOLERANCE_TO_CONTINUE_PASSING,
			StateMachineConstants.MAX_ANGLE_FROM_GOAL_CENTER,
			StateMachineConstants.MAX_DISTANCE_TO_SCORE_METERS
		);
	}

	private boolean calibrationIsReadyToScore() {
		return ShootingChecks.calibrationIsReadyToScore(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_SCORING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_START_SCORING
		);
	}

	private boolean calibrationIsReadyToPass() {
		return ShootingChecks.calibrationIsReadyToPass(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_PASSING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_START_PASSING
		);
	}

	private boolean calibrationCanContinueScoring() {
		return ShootingChecks.calibrationCanContinueScoring(
			robot,
			StateMachineConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_SCORING,
			StateMachineConstants.HOOD_POSITION_TOLERANCE_TO_CONTINUE_SCORING

		);
	}

	public Command ScoreSequence() {
		return new RepeatCommand(
			new SequentialCommandGroup(
				driveWith(RobotState.PRE_SCORE).until(this::isReadyToScore),
				driveWith(RobotState.SCORE).until(() -> (!canContinueScoring()))
			)
		);
	}

	public Command passSequence() {
		return new RepeatCommand(
			new SequentialCommandGroup(
				driveWith(RobotState.PRE_PASS).until(this::isReadyToPass),
				driveWith(RobotState.PASS).until(() -> (!canContinuePassing()))
			)
		);
	}

	public Command calibrationScoreSequence() {
		return new RepeatCommand(
			new SequentialCommandGroup(
				driveWith(RobotState.CALIBRATION_PRE_SCORE).until(this::isReadyToScore),
				driveWith(RobotState.CALIBRATION_SCORE).until(() -> !calibrationCanContinueScoring())
			)
		);
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(RobotState state) {
		return switch (state) {
			case STAY_IN_PLACE -> driveWith(RobotState.STAY_IN_PLACE);
			case DRIVE, SCORE, CALIBRATION_PRE_SCORE, CALIBRATION_SCORE, PASS -> driveWith(RobotState.DRIVE);
			case PRE_SCORE -> driveWith(RobotState.PRE_SCORE);
			case PRE_PASS -> driveWith(RobotState.PRE_PASS);
		};
	}

	public IntakeStateHandler getIntakeStateHandler() {
		return intakeStateHandler;
	}

}
