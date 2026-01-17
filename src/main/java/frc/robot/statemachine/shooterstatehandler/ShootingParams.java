package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Rotation2d;

public record ShootingParams(Rotation2d flywheelVelocityRPS, Rotation2d hoodPosition, Rotation2d turretPosition, Rotation2d turretVelocityRPS) {}
