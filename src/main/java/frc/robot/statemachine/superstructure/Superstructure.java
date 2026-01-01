package frc.robot.statemachine.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.statemachine.funnelstatehandler.FunnelState;
import frc.robot.statemachine.funnelstatehandler.FunnelStateHandler;
import frc.robot.statemachine.shooterstatehandler.ShooterState;
import frc.robot.statemachine.shooterstatehandler.ShooterStateHandler;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Superstructure {

	private final Robot robot;
	private final TargetChecks targetChecks;
	private boolean isSubsystemRunningIndependently;
	private final String logPath;

	private RobotState currentState;

	private final FunnelStateHandler funnelStateHandler;
	private final ShooterStateHandler shooterStateHandler;

	public Superstructure(String logPath, Robot robot, Supplier<Pose2d> robotPoseSupplier) {
		this.robot = robot;
		this.logPath = logPath;

		this.funnelStateHandler = new FunnelStateHandler(robot.getOmni(), logPath, robot.getFunnelDigitalInput());
		this.shooterStateHandler = new ShooterStateHandler(robot.getTurret(), robot.getHood(), robot.getFlyWheel(), robotPoseSupplier, logPath);

		this.targetChecks = new TargetChecks(this);

		this.currentState = RobotState.STAY_IN_PLACE;
		this.isSubsystemRunningIndependently = false;
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

	public boolean isSubsystemRunningIndependently() {
		return isSubsystemRunningIndependently
			|| robot.getFlyWheel().getCommandBuilder().isSubsystemRunningIndependently()
			|| robot.getHood().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getOmni().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getTurret().getCommandsBuilder().isSubsystemRunningIndependently();
	}

	public void setIsSubsystemRunningIndependently(boolean isSubsystemRunningIndependently) {
		this.isSubsystemRunningIndependently = isSubsystemRunningIndependently;
	}

	public TargetChecks getTargetChecks() {
		return targetChecks;
	}

	public Command setState(RobotState robotState) {
		return new ParallelCommandGroup(
			new InstantCommand(() -> currentState = robotState),
			new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentState", robotState)),
			switch (robotState) {
				case STAY_IN_PLACE -> stayInPlace();
				case DRIVE -> idle();
				case INTAKE -> intake();
				case PRE_SHOOT -> preShoot();
				case SHOOT -> shoot();
				case SHOOT_WHILE_INTAKE -> shootWhileIntake();
			}
		);
	}

	public boolean isObjectIn() {
		return funnelStateHandler.isBallAtSensor();
	}

	private Command stayInPlace() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.STAY_IN_PLACE), funnelStateHandler.setState(FunnelState.STOP));
	}

	private Command idle() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.IDLE), funnelStateHandler.setState(FunnelState.DRIVE));
	}

	private Command intake() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.IDLE), funnelStateHandler.setState(FunnelState.INTAKE));
	}

	private Command preShoot() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.SHOOT), funnelStateHandler.setState(FunnelState.DRIVE));
	}

	private Command shoot() {
		return new SequentialCommandGroup(
			new ParallelDeadlineGroup(funnelStateHandler.setState(FunnelState.SHOOT), shooterStateHandler.setState(ShooterState.SHOOT))
				.until(() -> !isObjectIn()),
			new ParallelCommandGroup(funnelStateHandler.setState(FunnelState.SHOOT), shooterStateHandler.setState(ShooterState.SHOOT))
				.withTimeout(StateMachineConstants.SECONDS_TO_WAIT_AFTER_SHOOT)
		);
	}

	private Command shootWhileIntake() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.SHOOT),
			funnelStateHandler.setState(FunnelState.SHOOT_WHILE_INTAKE)
		);
	}

	public void periodic() {
		funnelStateHandler.periodic();
		Logger.recordOutput(logPath + "/IsSubsystemRunningIndependently", isSubsystemRunningIndependently());
	}

}
