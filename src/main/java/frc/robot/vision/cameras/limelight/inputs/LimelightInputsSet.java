package frc.robot.vision.cameras.limelight.inputs;


public record LimelightInputsSet(
	MTInputsAutoLogged mt1Inputs,
	MTInputsAutoLogged mt2Inputs,
	NeuralDetectionInputsAutoLogged neuralDetectionInputs,
	ColorDetectionInputsAutoLogged colorDetectionInputs,
<<<<<<< HEAD
	LimelightHardwareInputsAutoLogged hardwareInputs,
	ConnectedInputAutoLogged connectedInput
=======
	LimelightHardwareInputsAutoLogged hardwareInputs
>>>>>>> template/master
) {

	public LimelightInputsSet() {
		this(
			new MTInputsAutoLogged(),
			new MTInputsAutoLogged(),
			new NeuralDetectionInputsAutoLogged(),
			new ColorDetectionInputsAutoLogged(),
<<<<<<< HEAD
			new LimelightHardwareInputsAutoLogged(),
			new ConnectedInputAutoLogged()
=======
			new LimelightHardwareInputsAutoLogged()
>>>>>>> template/master
		);
	}

}
