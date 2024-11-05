package frc.robot.autos;

public enum AutoSelection {
  DO_NOTHING("", ""),
  DYNAMIC_AMP_5_PIECE("Red Amp 5 Piece", "");

  public final String redAutoName;
  public final String blueAutoName;

  private AutoSelection(String redAutoName, String blueAutoName) {
    this.redAutoName = redAutoName;
    this.blueAutoName = blueAutoName;
  }
}
