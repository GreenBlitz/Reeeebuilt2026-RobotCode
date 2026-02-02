package frc.robot.statemachine;

import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;

public enum RobotState {

	STAY_IN_PLACE,
	DRIVE,
	PRE_SHOOT(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.LOOK_AT_HUB)),
	SHOOT(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.LOOK_AT_HUB)),
	CALIBRATION_PRE_SHOOT,
	CALIBRATION_SHOOT;

	private final SwerveState swerveState;
	private boolean swerveStateActive;

	RobotState() {
		this(SwerveState.DEFAULT_DRIVE);
	}

	RobotState(SwerveState state) {
		this.swerveState = state;
		this.swerveStateActive = true;
	}

	public RobotState activateSwerveAssist(boolean active) {
		swerveStateActive = active;
		return this;
	}

	public SwerveState getSwerveState() {
		return swerveStateActive ? swerveState : SwerveState.DEFAULT_DRIVE;
	}

}
