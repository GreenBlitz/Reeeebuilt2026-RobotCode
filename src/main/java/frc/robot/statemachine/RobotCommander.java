package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;

import frc.robot.subsystems.swerve.Swerve;

import java.util.Set;

public class RobotCommander extends GBSubsystem {

	private final Robot robot;
	private final Swerve swerve;
	private final Superstructure superstructure;

	private RobotState currentState;

	public RobotCommander(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.superstructure = new Superstructure("StateMachine/Superstructure", robot, () -> ShootingCalculations.getShootingParams());
		this.currentState = RobotState.STAY_IN_PLACE;

		setDefaultCommand(
			new ConditionalCommand(
				asSubsystemCommand(Commands.none(), "Disabled"),
				new InstantCommand(
					() -> CommandScheduler.getInstance()
						.schedule(
							new DeferredCommand(
								() -> endState(currentState),
								Set.of(
									this,
									swerve,
									robot.getIntakeRoller(),
									robot.getTurret(),
									robot.getFourBar(),
									robot.getHood(),
									robot.getTrain(),
									robot.getBelly(),
									robot.getFlyWheel()
								)
							)
						)
				),
				this::isSubsystemRunningIndependently
			)

		);
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}

	public boolean isSubsystemRunningIndependently() {
		return superstructure.isSubsystemRunningIndependently() || swerve.getCommandsBuilder().isSubsystemRunningIndependently();
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

	public Command shootSequence() {
		return new RepeatCommand(
			new SequentialCommandGroup(
				driveWith(RobotState.PRE_SHOOT).until(this::isReadyToShoot),
				driveWith(RobotState.SHOOT).until(() -> (!canContinueShooting()))
			)
		);
	}

	public Command calibrationShootSequence() {
		return new RepeatCommand(
			new SequentialCommandGroup(
				driveWith(RobotState.CALIBRATION_PRE_SHOOT).until(this::calibrationIsReadyToShoot),
				driveWith(RobotState.CALIBRATION_SHOOT).until(() -> !canContinueShooting())
			)
		);
	}

	public Command shootWhileIntakeSequence() {
		return new SequentialCommandGroup(driveWith(RobotState.PRE_SHOOT).until(this::isReadyToShoot), driveWith(RobotState.SHOOT_WHILE_INTAKE));
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(RobotState state) {
		return switch (state) {
			case STAY_IN_PLACE -> driveWith(RobotState.STAY_IN_PLACE);
			case DRIVE, INTAKE, SHOOT, SHOOT_WHILE_INTAKE, CALIBRATION_PRE_SHOOT, CALIBRATION_SHOOT -> driveWith(RobotState.DRIVE);
			case PRE_SHOOT -> driveWith(RobotState.PRE_SHOOT);
		};
	}

}
