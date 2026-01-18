package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Rotation2d;

public record ShootingParams(
	Rotation2d targetFlywheelVelocityRotation2dPerSecond,
	Rotation2d targetHoodPosition,
	Rotation2d targetTurretPosition,
	Rotation2d targetTurretVelocityRotation2dPerSecond
) {}
