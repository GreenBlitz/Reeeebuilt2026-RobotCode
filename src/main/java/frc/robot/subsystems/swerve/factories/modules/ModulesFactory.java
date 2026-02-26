package frc.robot.subsystems.swerve.factories.modules;

import frc.robot.IDs;
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
			EncoderFactory.createSignals(angleEncoder),
			steer,
			SteerFactory.createRequests(),
			SteerFactory.createSignals(steer),
			drive,
			DriveFactory.createRequests(),
			DriveFactory.createSignals(drive)
		);
	}

	public static Modules create(String logPath) {
		return new Modules(
			logPath,
			createModule(logPath, ModuleUtil.ModulePosition.FRONT_LEFT, IDs.TalonFXIDs.SWERVE_FRONT_LEFT_STEER.busChain()),
			createModule(logPath, ModuleUtil.ModulePosition.FRONT_RIGHT, IDs.TalonFXIDs.SWERVE_FRONT_RIGHT_STEER.busChain()),
			createModule(logPath, ModuleUtil.ModulePosition.BACK_LEFT, IDs.TalonFXIDs.SWERVE_BACK_LEFT_STEER.busChain()),
			createModule(logPath, ModuleUtil.ModulePosition.BACK_RIGHT, IDs.TalonFXIDs.SWERVE_BACK_RIGHT_STEER.busChain())
		);
	}

}
