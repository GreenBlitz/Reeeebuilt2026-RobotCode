package frc.constants.field;

public enum AllianceSide {

	DEPOT,
	OUTPOST;

	public AllianceSide getOtherSide() {
		if(this == AllianceSide.DEPOT)
			return AllianceSide.OUTPOST;
		return AllianceSide.DEPOT;
	}
}
