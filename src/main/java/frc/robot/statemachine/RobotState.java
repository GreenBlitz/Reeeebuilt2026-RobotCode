package frc.robot.statemachine;

import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;

public enum RobotState {

	STAY_IN_PLACE,
	NEUTRAL,
	PRE_SCORE(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.LOOK_AT_TARGET)),
	SCORE(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.LOOK_AT_TARGET)),
	PRE_PASS(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.LOOK_AT_TARGET)),
	PASS(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.LOOK_AT_TARGET)),
	RESET_SUBSYSTEMS,
	CALIBRATION_PRE_SCORE,
	CALIBRATION_SCORE;

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
