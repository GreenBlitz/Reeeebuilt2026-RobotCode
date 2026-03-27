package frc.robot.subsystems.swerve.states;

public enum AccelerationLimit {

	NONE(Double.MAX_VALUE),
	SHOOTING(1.5);

	private final double maxAccelerationMetersPerSecondSquared;

	AccelerationLimit(double maxAccelerationMetersPerSecondSquared) {
		this.maxAccelerationMetersPerSecondSquared = maxAccelerationMetersPerSecondSquared;
	}

	public double getMaxAccelerationMetersPerSecondSquared() {
		return maxAccelerationMetersPerSecondSquared;
	}

}
