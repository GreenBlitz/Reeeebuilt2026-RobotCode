package frc.robot.subsystems.swerve.module.extrainputs;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DriveInputs {

	public Rotation2d[] uncoupledVelocityAnglesPerSecond = new Rotation2d[0];
	public Rotation2d[] uncoupledPositions = new Rotation2d[0];

	public double[] velocityMetersPerSecond = new double[0];
	public double[] positionsMeters = new double[0];

}
