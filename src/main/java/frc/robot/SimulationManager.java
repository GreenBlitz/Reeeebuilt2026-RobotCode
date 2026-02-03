package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.constants.MathConstants;
import frc.robot.subsystems.constants.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

public class SimulationManager {

	private final String logPath;
	private final Robot robot;

	public SimulationManager(String logPath, Robot robot) {
		this.logPath = logPath;
		this.robot = robot;
	}

	public void logPoses() {
		logIntakePosition3d();
		logHopperPosition3d();
		logTurretPosition3d();
		logHoodPosition3d();
	}

	private void logIntakePosition3d() {
		Logger.recordOutput(logPath + "/Intake", getIntakePosition3d());
	}

	private void logHopperPosition3d() {
		Logger.recordOutput(logPath + "/Hopper", getHopperPosition3d());
	}

	public void logTurretPosition3d() {
		Logger.recordOutput(logPath + "/Turret", getTurretPosition3d());
	}

	public void logHoodPosition3d() {
		Logger.recordOutput(logPath + "/Hood", getHoodPosition3d());
	}

	public Pose3d getIntakePosition3d() {
		return new Pose3d(new Translation3d(-0.3, 0, 0.1), new Rotation3d(0.0, robot.getFourBar().getPosition().getRadians(), 0.0));
	}

	public Pose3d getHopperPosition3d() {
		return new Pose3d(new Translation3d(findHopperExtensionLength(0.4), 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));
	}

	public Pose3d getTurretPosition3d() {
		return new Pose3d(
			TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT,
			new Rotation3d(0.0, 0.0, robot.getTurret().getPosition().getRadians() + MathConstants.QUARTER_CIRCLE.getRadians())
		);
	}

	public Pose3d getHoodPosition3d() {
		return new Pose3d(
			getTurretPosition3d().getTranslation(),
			new Rotation3d(
				robot.getHood().getPosition().getRadians(),
				0.0,
				robot.getTurret().getPosition().getRadians() + MathConstants.QUARTER_CIRCLE.getRadians()
			)
		);
	}

	public double findHopperExtensionLength(double intakeLength) {
		return 0.35 - intakeLength * Math.cos(getIntakePosition3d().getRotation().getAngle()) * 1.2;
	}

}
