package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6OdometryThread;
import frc.robot.hardware.signal.AngleSignal;
import frc.utils.AngleUnit;
import frc.robot.hardware.phoenix6.Phoenix6Util;

public class Phoenix6SignalBuilder {

	private static final int UPDATE_FREQUENCY_RETRIES = 5;

	private static void setFrequencyWithRetry(StatusSignal<?> signal, double frequency) {
		Phoenix6Util.checkStatusCodeWithRetry(() -> signal.setUpdateFrequency(frequency), UPDATE_FREQUENCY_RETRIES);
	}

	private static StatusSignal<?> cloneWithFrequency(StatusSignal<?> signal, double frequency, BusChain busChain) {
		StatusSignal<?> signalClone = signal.clone();
		setFrequencyWithRetry(signalClone, frequency);
		busChain.registerSignal(signalClone);
		return signalClone;
	}

	public static Phoenix6DoubleSignal build(StatusSignal<?> signal, double frequency, BusChain busChain) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency, busChain);
		return new Phoenix6DoubleSignal(signalClone.getName(), signalClone);
	}

	public static Phoenix6AngleSignal build(StatusSignal<?> signal, double frequency, AngleUnit angleUnit, BusChain busChain) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency, busChain);
		return new Phoenix6AngleSignal(signalClone.getName(), signalClone, angleUnit);
	}

	public static Phoenix6LatencySignal build(
		StatusSignal<?> signal,
		SignalGetter signalSlope,
		double frequency,
		AngleUnit angleUnit,
		BusChain busChain
	) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency, busChain);
		setFrequencyWithRetry(signalSlope.getSignal(), frequency);
		return new Phoenix6LatencySignal(signalClone.getName(), signalClone, signalSlope.getSignal(), angleUnit);
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
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency, busChain);
		StatusSignal<?> signalSlopeClone = cloneWithFrequency(signalSlope, frequency, busChain);
		return new Phoenix6LatencyAndSlopeSignal(signalClone.getName(), signalClone, signalSlopeClone, angleUnit);
	}

	public static AngleSignal buildThreaded(
		StatusSignal<?> signal,
		StatusSignal<?> signalSlope,
		double frequency,
		AngleUnit angleUnit,
		BusChain busChain
	) {
		if (!RobotConstants.ENABLE_ODOMETRY_THREAD || Robot.ROBOT_TYPE.isSimulation()) {
			return build(signal, signalSlope, RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, angleUnit, busChain);
		}
		Phoenix6OdometryThread.SignalQueues queues = Phoenix6OdometryThread.getInstance()
			.registerLatencySignal(signal, signalSlope, frequency, busChain);
		return new Phoenix6ThreadLatencySignal(
			queues.clonedSignal().getName(),
			queues.clonedSignal(),
			signalSlope,
			angleUnit,
			queues.values(),
			queues.timestamps()
		);
	}

}
