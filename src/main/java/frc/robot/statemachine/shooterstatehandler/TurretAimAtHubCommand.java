package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.statemachine.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.constants.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;
import java.util.function.Supplier;

public class TurretAimAtHubCommand extends Command {

	private final Arm turret;
	private final Supplier<Pose2d> robotPose;
	private final String logPath;

	public TurretAimAtHubCommand(Arm turret, Supplier<Pose2d> robotPose, String logPath) {
		this.turret = turret;
		this.robotPose = robotPose;
		this.logPath = logPath;
	}

	@Override
	public void execute() {
		Rotation2d targetAngle = ShooterCalculations.getRobotRelativeLookAtHubAngleForTurret(robotPose.get(), turret.getPosition());

		if (ShooterCalculations.isTurretMoveLegal(targetAngle, turret.getPosition())) {
			Logger.recordOutput(logPath + "/IsTurretGoingToPosition", true);
		} else {
			targetAngle = turret.getPosition().getDegrees() < TurretConstants.MIDDLE_OF_SHOOTING_RANGE.getDegrees()
				? TurretConstants.BACKWARDS_SOFTWARE_LIMIT
				: TurretConstants.FORWARD_SOFTWARE_LIMIT;
			Logger.recordOutput(logPath + "/IsTurretGoingToPosition", false);
		}

		turret.setTargetPosition(targetAngle);
	}

}
