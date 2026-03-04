package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.*;
import com.google.common.collect.Range;
import com.google.common.collect.RangeMap;
import com.google.common.collect.TreeRangeMap;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Supplier;

public class Phoenix6RequestBuilder {

	public static Phoenix6VelocityPositionRequest build(PositionVoltage positionVoltage, double defaultArbitraryFeedForward, boolean enableFOC) {
		return new Phoenix6VelocityPositionRequest(
			setPoint -> positionVoltage.withVelocity(setPoint.getRotations()),
			setPoint -> positionVoltage.withPosition(setPoint.getRotations()),
			Rotation2d.fromRotations(positionVoltage.Position),
			positionVoltage.withEnableFOC(enableFOC),
			positionVoltage::withFeedForward,
			defaultArbitraryFeedForward,
			Rotation2d.fromRotations(positionVoltage.Velocity)
		);
	}

	public static Phoenix6FeedForwardRequest build(VelocityVoltage velocityVoltage, double defaultArbitraryFeedForward, boolean enableFOC) {
		return new Phoenix6FeedForwardRequest(
			Rotation2d.fromRotations(velocityVoltage.Velocity),
			velocityVoltage.withEnableFOC(enableFOC),
			setPoint -> velocityVoltage.withVelocity(setPoint.getRotations()),
			velocityVoltage::withFeedForward,
			defaultArbitraryFeedForward
		);
	}

	public static Phoenix6FeedForwardRequest build(
		MotionMagicVelocityVoltage velocityVoltage,
		double defaultArbitraryFeedForward,
		boolean enableFOC
	) {
		return new Phoenix6FeedForwardRequest(
			Rotation2d.fromRotations(velocityVoltage.Velocity),
			velocityVoltage.withEnableFOC(enableFOC),
			setPoint -> velocityVoltage.withVelocity(setPoint.getRotations()),
			velocityVoltage::withFeedForward,
			defaultArbitraryFeedForward
		);
	}

	public static Phoenix6FeedForwardRequest build(VelocityTorqueCurrentFOC velocityTorqueCurrentFOC, double defaultArbitraryFeedForward) {
		return new Phoenix6FeedForwardRequest(
			Rotation2d.fromRotations(velocityTorqueCurrentFOC.Velocity),
			velocityTorqueCurrentFOC,
			setPoint -> velocityTorqueCurrentFOC.withVelocity(setPoint.getRotations()),
			velocityTorqueCurrentFOC::withFeedForward,
			defaultArbitraryFeedForward
		);
	}

	public static Phoenix6MotionMagicRequest build(
		MotionMagicVoltage motionMagicVoltage,
		double defaultArbitraryFeedForward,
		boolean enableFOC
	) {
		return new Phoenix6MotionMagicRequest(
			Rotation2d.fromRotations(motionMagicVoltage.Position),
			motionMagicVoltage.withEnableFOC(enableFOC),
			setPoint -> motionMagicVoltage.withPosition(setPoint.getRotations()),
			motionMagicVoltage::withFeedForward,
			defaultArbitraryFeedForward
		);
	}

	public static Phoenix6DynamicMotionMagicRequest build(
		DynamicMotionMagicVoltage dynamicMotionMagicVoltage,
		double defaultArbitraryFeedForward,
		boolean enableFOC
	) {
		return new Phoenix6DynamicMotionMagicRequest(
			Rotation2d.fromRotations(dynamicMotionMagicVoltage.Position),
			dynamicMotionMagicVoltage.withEnableFOC(enableFOC),
			setPoint -> dynamicMotionMagicVoltage.withPosition(setPoint.getRotations()),
			dynamicMotionMagicVoltage::withFeedForward,
			maxVelocity -> dynamicMotionMagicVoltage.withVelocity(maxVelocity.getRotations()),
			maxAcceleration -> dynamicMotionMagicVoltage.withAcceleration(maxAcceleration.getRotations()),
			defaultArbitraryFeedForward
		);
	}

	public static Phoenix6Request<Double> build(VoltageOut voltageOut, boolean enableFOC) {
		return new Phoenix6Request<>(voltageOut.Output, voltageOut.withEnableFOC(enableFOC), voltageOut::withOutput);
	}

	public static Phoenix6Request<Double> build(TorqueCurrentFOC torqueCurrentFOC) {
		return new Phoenix6Request<>(torqueCurrentFOC.Output, torqueCurrentFOC, torqueCurrentFOC::withOutput);
	}

	public static Phoenix6Request<Rotation2d> buildBangBangRequest(Supplier<Rotation2d> currentVelocity, boolean enableFOC) {
		RangeMap<Double, Double> errorToOutputRangeMap = TreeRangeMap.create();
		errorToOutputRangeMap.put(Range.atLeast(0.0), 1.0);
		errorToOutputRangeMap.put(Range.atMost(0.0), -1.0);

		DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(enableFOC);
		return new Phoenix6Request<>(
			Rotation2d.kZero,
			dutyCycleOut,
			(Rotation2d targetVelocity) -> dutyCycleOut.withOutput(
				java.util.Objects
					.requireNonNullElse(errorToOutputRangeMap.get(targetVelocity.getRotations() - currentVelocity.get().getRotations()), 0.0)
			)
		);
	}

	public static Phoenix6Request<Rotation2d> buildBangBangRequest(
		Supplier<Rotation2d> currentVelocity,
		RangeMap<Double, Double> errorToOutputRangeMap,
		boolean enableFOC
	) {
		DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(enableFOC);
		return new Phoenix6Request<>(
			Rotation2d.kZero,
			dutyCycleOut,
			(Rotation2d targetVelocity) -> dutyCycleOut.withOutput(
				java.util.Objects
					.requireNonNullElse(errorToOutputRangeMap.get(targetVelocity.getRotations() - currentVelocity.get().getRotations()), 0.0)
			)
		);
	}

}
