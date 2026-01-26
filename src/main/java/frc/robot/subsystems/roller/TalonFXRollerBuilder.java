package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6FeedForwardRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;
import frc.utils.battery.BatteryUtil;

public class TalonFXRollerBuilder {

	public static VelocityRoller buildVelocityRoller(
		String logPath,
		Phoenix6DeviceID deviceID,
		double kV,
		double kA,
		int currentLimit,
		double arbitraryFeedForward,
		double gearRatio,
		double momentOfInertia
	) {
		SimpleMotorSimulation rollerSimulation = new SimpleMotorSimulation(
			new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), momentOfInertia, gearRatio), DCMotor.getKrakenX60(1))
		);
		TalonFXMotor roller = new TalonFXMotor(logPath, deviceID, new TalonFXFollowerConfig(), new SysIdRoutine.Config(), rollerSimulation);

		roller.applyConfiguration(buildConfiguration(gearRatio, currentLimit, kV, kA));

		InputSignal<Double> voltageSignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, deviceID.busChain());
		InputSignal<Double> currentSignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getStatorCurrent(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, deviceID.busChain());
		InputSignal<Rotation2d> positionSignal = Phoenix6SignalBuilder.build(
			roller.getDevice().getPosition(),
			roller.getDevice().getVelocity(),
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS,
			deviceID.busChain()
		);
		InputSignal<Rotation2d> velocitySignal = Phoenix6SignalBuilder.build(roller.getDevice().getVelocity(),RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,AngleUnit.ROTATIONS, BusChain.ROBORIO);

		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), true);
		Phoenix6FeedForwardRequest velocityVoltage = Phoenix6RequestBuilder.build(new VelocityVoltage(0), arbitraryFeedForward, true);

		return new VelocityRoller(logPath, roller, voltageSignal, currentSignal, positionSignal,velocitySignal , voltageRequest, velocityVoltage);
	}

	public static Roller build(
		String logPath,
		Phoenix6DeviceID id,
		boolean inverted,
		double gearRatio,
		int currentLimit,
		double momentOfInertia
	) {
		SimpleMotorSimulation rollerSimulation = new SimpleMotorSimulation(
			new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), momentOfInertia, gearRatio), DCMotor.getKrakenX60(1))
		);
		TalonFXMotor roller = new TalonFXMotor(logPath, id, new TalonFXFollowerConfig(), new SysIdRoutine.Config(), rollerSimulation);

		roller.applyConfiguration(buildConfiguration(inverted, gearRatio, currentLimit));

		InputSignal<Double> voltageSignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, id.busChain());
		InputSignal<Double> currentSignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getStatorCurrent(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, id.busChain());
		InputSignal<Rotation2d> positionSignal = Phoenix6SignalBuilder.build(
			roller.getDevice().getPosition(),
			roller.getDevice().getVelocity(),
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS,
			id.busChain()
		);

		Phoenix6Request<Double> VoltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), true);

		return new Roller(logPath, roller, voltageSignal, currentSignal, positionSignal, VoltageRequest);
	}

	public static TalonFXConfiguration buildConfiguration(boolean inverted, double gearRatio, int currentLimit) {
		TalonFXConfiguration configs = new TalonFXConfiguration();
		configs.CurrentLimits.StatorCurrentLimit = currentLimit;
		configs.CurrentLimits.StatorCurrentLimitEnable = true;
		configs.Feedback.SensorToMechanismRatio = gearRatio;
		configs.Voltage.PeakForwardVoltage = BatteryUtil.DEFAULT_VOLTAGE;
		configs.Voltage.PeakReverseVoltage = -BatteryUtil.DEFAULT_VOLTAGE;
		configs.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
		return (configs);
	}

	public static TalonFXConfiguration buildConfiguration(double gearRatio, int currentLimit, double kV, double kA) {
		TalonFXConfiguration configs = new TalonFXConfiguration();
		configs.CurrentLimits.StatorCurrentLimit = currentLimit;
		configs.CurrentLimits.StatorCurrentLimitEnable = true;
		configs.Feedback.SensorToMechanismRatio = gearRatio;
		configs.Slot0.kV = kV;
		configs.Slot0.kA = kA;
		configs.Voltage.PeakForwardVoltage = BatteryUtil.DEFAULT_VOLTAGE;
		configs.Voltage.PeakReverseVoltage = BatteryUtil.DEFAULT_VOLTAGE;
		return (configs);
	}

}
