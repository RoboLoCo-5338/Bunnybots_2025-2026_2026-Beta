package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.AutoLog;

public class LEDIO {

  @AutoLog
  public static class LEDIOInputs {}

  /**
   * Sets the color
   *
   * @param color the color input to change to
   */
  public void setColor(Color color) {}

  /**
   * Sets the blink
   *
   * @param color the color input to blink
   * @param blinkTime the time that each blink should take
   */
  public void setBlink(Color color, double blinkTime) {}

  /**
   * Updates inputs
   *
   * @param inputs LEDIOInputs to update
   */
  public void updateInputs(LEDIOInputs inputs) {}
}
