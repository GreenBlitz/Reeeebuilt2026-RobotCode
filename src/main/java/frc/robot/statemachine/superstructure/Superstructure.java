package frc.robot.statemachine.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.statemachine.funnelstatehandler.FunnelState;
import frc.robot.statemachine.funnelstatehandler.FunnelStateHandler;
import frc.robot.statemachine.shooterstatehandler.ShooterState;
import frc.robot.statemachine.shooterstatehandler.ShooterStateHandler;
import frc.robot.statemachine.shooterstatehandler.ShootingParams;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Superstructure {

	private final Robot robot;
	private boolean isSubsystemRunningIndependently;
	private final String logPath;

	private RobotState currentState;

	private final FunnelStateHandler funnelStateHandler;
	private final ShooterStateHandler shooterStateHandler;

	public Superstructure(String logPath, Robot robot, Supplier<ShootingParams> shootingParamsSupplier) {
		this.robot = robot;
		this.logPath = logPath;

		this.funnelStateHandler = new FunnelStateHandler(robot.getTrain(), robot.getBelly(), logPath);

		this.shooterStateHandler = new ShooterStateHandler(
			robot.getTurret(),
			robot.getHood(),
			robot.getFlyWheel(),
			shootingParamsSupplier,
			logPath
		);

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
			|| robot.getTrain().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getBelly().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getTurret().getCommandsBuilder().isSubsystemRunningIndependently();
	}

	public void setIsSubsystemRunningIndependently(boolean isSubsystemRunningIndependently) {
		this.isSubsystemRunningIndependently = isSubsystemRunningIndependently;
	}

	public Command setState(RobotState robotState) {
		return new ParallelCommandGroup(
			new InstantCommand(() -> currentState = robotState),
			new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentState", robotState)),
			switch (robotState) {
				case STAY_IN_PLACE -> stayInPlace();
				case DRIVE -> idle();
				case PRE_SHOOT -> preShoot();
				case SHOOT -> shoot();
				case CALIBRATION_PRE_SHOOT -> calibrationPreShoot();
				case CALIBRATION_SHOOT -> calibrationShoot();
			}
		);
	}

	private Command stayInPlace() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.STAY_IN_PLACE), funnelStateHandler.setState(FunnelState.STOP));
	}

	private Command idle() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.IDLE), funnelStateHandler.setState(FunnelState.DRIVE));
	}

	private Command preShoot() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.SHOOT), funnelStateHandler.setState(FunnelState.DRIVE));
	}

	private Command shoot() {
		return new SequentialCommandGroup(
			new ParallelDeadlineGroup(funnelStateHandler.setState(FunnelState.SHOOT), shooterStateHandler.setState(ShooterState.SHOOT)),
			new ParallelCommandGroup(funnelStateHandler.setState(FunnelState.SHOOT), shooterStateHandler.setState(ShooterState.SHOOT))
				.withTimeout(StateMachineConstants.SECONDS_TO_WAIT_AFTER_SHOOT)
		);
	}

	private Command calibrationPreShoot() {
		return new ParallelCommandGroup(shooterStateHandler.setState(ShooterState.CALIBRATION), funnelStateHandler.setState(FunnelState.DRIVE));
	}

	private Command calibrationShoot() {
		return new ParallelDeadlineGroup(shooterStateHandler.setState(ShooterState.CALIBRATION), funnelStateHandler.setState(FunnelState.SHOOT));
	}

	public void periodic() {
		funnelStateHandler.periodic();
		Logger.recordOutput(logPath + "/IsSubsystemRunningIndependently", isSubsystemRunningIndependently());
	}

}
