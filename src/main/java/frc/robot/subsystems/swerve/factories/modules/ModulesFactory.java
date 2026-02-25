package frc.robot.subsystems.swerve.factories.modules;

import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.subsystems.swerve.factories.modules.constants.ModuleSpecificConstantsFactory;
import frc.robot.subsystems.swerve.factories.modules.drive.DriveFactory;
import frc.robot.subsystems.swerve.factories.modules.encoder.EncoderFactory;
import frc.robot.subsystems.swerve.factories.modules.steer.SteerFactory;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.module.Modules;

public class ModulesFactory {

	private static Module createModule(String logPath, ModuleUtil.ModulePosition modulePosition, BusChain busChain) {
		IAngleEncoder angleEncoder = EncoderFactory.createEncoder(logPath, modulePosition);
		ControllableMotor steer = SteerFactory.createSteer(logPath, modulePosition);
		ControllableMotor drive = DriveFactory.createDrive(logPath, modulePosition);

		return new Module(
			ModuleSpecificConstantsFactory.create(logPath, modulePosition),
			angleEncoder,
			EncoderFactory.createSignals(angleEncoder, busChain),
			steer,
			SteerFactory.createRequests(),
			SteerFactory.createSignals(steer, busChain),
			drive,
			DriveFactory.createRequests(),
			DriveFactory.createSignals(drive, busChain)
		);
	}

	public static Modules create(String logPath, BusChain busChain) {
		return new Modules(
			logPath,
			createModule(logPath, ModuleUtil.ModulePosition.FRONT_LEFT, busChain),
			createModule(logPath, ModuleUtil.ModulePosition.FRONT_RIGHT, busChain),
			createModule(logPath, ModuleUtil.ModulePosition.BACK_LEFT, busChain),
			createModule(logPath, ModuleUtil.ModulePosition.BACK_RIGHT, busChain)
		);
	}

}
