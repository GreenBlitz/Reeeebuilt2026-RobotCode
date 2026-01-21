package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.constants.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class TurretSafeMoveToPosition extends Command {

	private final Arm turret;
	private final String logPath;
	private final Supplier<Rotation2d> targetPosition;

	public TurretSafeMoveToPosition(Arm turret, Supplier<Rotation2d> targetPosition, String logPath) {
		this.turret = turret;
		this.targetPosition = targetPosition;
		this.logPath = logPath;
	}

	@Override
	public void execute() {
		Rotation2d targetPosition = TurretCalculations.getWrappedTurretPosition(this.targetPosition.get());
		if (TurretCalculations.isTurretMoveLegal(targetPosition, turret.getPosition())) {
			Logger.recordOutput(logPath + "/IsTurretGoingToPosition", true);
		} else {
			targetPosition = turret.getPosition().getDegrees() < TurretConstants.MIDDLE_OF_SHOOTING_RANGE.getDegrees()
				? TurretConstants.BACKWARDS_SOFTWARE_LIMIT
				: TurretConstants.FORWARD_SOFTWARE_LIMIT;
			Logger.recordOutput(logPath + "/IsTurretGoingToPosition", false);
		}

		turret.setTargetPosition(targetPosition);
	}

}
