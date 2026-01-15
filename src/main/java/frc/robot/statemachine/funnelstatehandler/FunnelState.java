package frc.robot.statemachine.funnelstatehandler;

public enum FunnelState {

	STOP,
	INTAKE(3,1),
	SHOOT(3,1),
	SHOOT_WHILE_INTAKE(3,1),
	DRIVE(),
	CALIBRATION;

	private final double omniVoltage;
    private final double conveyorBeltVoltage;
	FunnelState(double omniVoltage,double ConveyorBeltVoltage) {
		this.omniVoltage = omniVoltage;
        this.conveyorBeltVoltage = ConveyorBeltVoltage;
	}

	FunnelState() {
		this.omniVoltage = 0;
        this.conveyorBeltVoltage = 0;
	}

	public double getOmniVoltage() {
		return omniVoltage;
	}

    public double getConveyorBeltVoltage(){
        return conveyorBeltVoltage;
    }
}
