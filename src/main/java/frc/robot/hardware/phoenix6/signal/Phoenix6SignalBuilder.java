package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6Util;
import frc.utils.AngleUnit;

public class Phoenix6SignalBuilder {

	private static final int UPDATE_FREQUENCY_RETRIES = 5;

	private static void setFrequencyWithRetry(StatusSignal<?> signal, double frequency) {
		Phoenix6Util.checkStatusCodeWithRetry(() -> signal.setUpdateFrequency(frequency), UPDATE_FREQUENCY_RETRIES);
	}

	private static boolean shouldUseThread(boolean useThread) {
		return useThread && RobotConstants.ENABLE_ODOMETRY_THREAD && !Robot.ROBOT_TYPE.isSimulation();
	}

	private static double resolveAppliedFrequency(double frequency, boolean useThread, boolean threadedActive) {
		return useThread && !threadedActive ? RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ : frequency;
	}

	private static StatusSignal<?> cloneWithFrequency(StatusSignal<?> signal, double frequency, BusChain busChain, boolean threadedActive) {
		if (threadedActive) {
			if (!Robot.ROBOT_TYPE.isReal()) {
				return signal;
			}
			StatusSignal<?> signalClone = signal.clone();
			setFrequencyWithRetry(signalClone, frequency);
			signalClone.refresh();
			return signalClone;
		}
		StatusSignal<?> signalClone = signal.clone();
		setFrequencyWithRetry(signalClone, frequency);
		busChain.registerSignal(signalClone);
		return signalClone;
	}

	public static Phoenix6DoubleSignal build(StatusSignal<?> signal, double frequency, BusChain busChain) {
		return build(signal, frequency, busChain, false);
	}

	public static Phoenix6DoubleSignal build(StatusSignal<?> signal, double frequency, BusChain busChain, boolean useThread) {
		boolean threadedActive = shouldUseThread(useThread);
		double appliedFrequency = resolveAppliedFrequency(frequency, useThread, threadedActive);
		StatusSignal<?> signalClone = cloneWithFrequency(signal, appliedFrequency, busChain, threadedActive);
		Phoenix6DoubleSignal phoenixSignal = new Phoenix6DoubleSignal(signalClone.getName(), signalClone);
		if (threadedActive) {
			busChain.registerThreadedSignal(phoenixSignal, appliedFrequency, signalClone);
		}
		return phoenixSignal;
	}

	public static Phoenix6AngleSignal build(StatusSignal<?> signal, double frequency, AngleUnit angleUnit, BusChain busChain) {
		return build(signal, frequency, angleUnit, busChain, false);
	}

	public static Phoenix6AngleSignal build(
		StatusSignal<?> signal,
		double frequency,
		AngleUnit angleUnit,
		BusChain busChain,
		boolean useThread
	) {
		boolean threadedActive = shouldUseThread(useThread);
		double appliedFrequency = resolveAppliedFrequency(frequency, useThread, threadedActive);
		StatusSignal<?> signalClone = cloneWithFrequency(signal, appliedFrequency, busChain, threadedActive);
		Phoenix6AngleSignal phoenixSignal = new Phoenix6AngleSignal(signalClone.getName(), signalClone, angleUnit);
		if (threadedActive) {
			busChain.registerThreadedSignal(phoenixSignal, appliedFrequency, signalClone);
		}
		return phoenixSignal;
	}

	public static Phoenix6LatencySignal build(
		StatusSignal<?> signal,
		SignalGetter signalSlope,
		double frequency,
		AngleUnit angleUnit,
		BusChain busChain
	) {
		return build(signal, signalSlope, frequency, angleUnit, busChain, false);
	}

	public static Phoenix6LatencySignal build(
		StatusSignal<?> signal,
		SignalGetter signalSlope,
		double frequency,
		AngleUnit angleUnit,
		BusChain busChain,
		boolean useThread
	) {
		boolean threadedActive = shouldUseThread(useThread);
		double appliedFrequency = resolveAppliedFrequency(frequency, useThread, threadedActive);
		StatusSignal<?> signalClone = cloneWithFrequency(signal, appliedFrequency, busChain, threadedActive);
		if (!threadedActive || Robot.ROBOT_TYPE.isReal()) {
			setFrequencyWithRetry(signalSlope.getSignal(), appliedFrequency);
		}
		Phoenix6LatencySignal phoenixSignal = new Phoenix6LatencySignal(signalClone.getName(), signalClone, signalSlope.getSignal(), angleUnit);
		if (threadedActive) {
			busChain.registerThreadedSignal(phoenixSignal, appliedFrequency, signalClone, signalSlope.getSignal());
		}
		return phoenixSignal;
	}

	/**
	 * Use this function only if you are not fetching the slope signal. Ex: If you care only about position and not velocity do:
	 * Phoenix6BothLatencySignal position = Phoenix6SignalBuilder.generatePhoenix6Signal(motor.getPosition(), motor.getVelocity(),...);
	 */
	public static Phoenix6LatencyAndSlopeSignal build(
		StatusSignal<?> signal,
		StatusSignal<?> signalSlope,
		double frequency,
		AngleUnit angleUnit,
		BusChain busChain
	) {
		return build(signal, signalSlope, frequency, angleUnit, busChain, false);
	}

	public static Phoenix6LatencyAndSlopeSignal build(
		StatusSignal<?> signal,
		StatusSignal<?> signalSlope,
		double frequency,
		AngleUnit angleUnit,
		BusChain busChain,
		boolean useThread
	) {
		boolean threadedActive = shouldUseThread(useThread);
		double appliedFrequency = resolveAppliedFrequency(frequency, useThread, threadedActive);
		StatusSignal<?> signalClone = cloneWithFrequency(signal, appliedFrequency, busChain, threadedActive);
		StatusSignal<?> signalSlopeClone = cloneWithFrequency(signalSlope, appliedFrequency, busChain, threadedActive);
		Phoenix6LatencyAndSlopeSignal phoenixSignal = new Phoenix6LatencyAndSlopeSignal(
			signalClone.getName(),
			signalClone,
			signalSlopeClone,
			angleUnit
		);
		if (threadedActive) {
			busChain.registerThreadedSignal(phoenixSignal, appliedFrequency, signalClone, signalSlopeClone);
		}
		return phoenixSignal;
	}

}
