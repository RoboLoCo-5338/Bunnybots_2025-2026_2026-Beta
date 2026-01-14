package frc.robot.subsystems.led;

import edu.wpi.first.units.measure.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIOAddressable extends LEDIO {

  private static final AddressableLED led = new AddressableLED(LEDConstants.PWM_PORT_NUM);
  private static final AddressableLEDBuffer buffer =
      new AddressableLEDBuffer(LEDConstants.STRIP_LENGTH);

  {
    {
      led.setLength(buffer.getLength());
      led.setData(buffer);
      led.start();
    }
  }

  private final AddressableLEDBufferView view;

  /**
   * Creates a BufferView from the AddressableLEDBuffer
   *
   * @param startingIndex starting LED index for the BufferView
   * @param endingIndex ending LED index for the BufferView
   */
  public LEDIOAddressable(int startingIndex, int endingIndex) {
    view = new AddressableLEDBufferView(buffer, startingIndex, endingIndex);
  }

  /**
   * Sets the color to the specified Color input
   *
   * @param color the input color
   */
  public void setColor(Color color) {

    LEDPattern ledColor = LEDPattern.solid(color);
    ledColor.applyTo(view);
    led.setData(buffer);
  }

  /**
   * Makes the LED blink a specific color and set the amount of time each blink lasts
   *
   * @param color the input color
   * @param blinkTime the input time for each singular blink to take
   */
  public void setBlink(Color color, double blinkTime) {

    LEDPattern ledColor = LEDPattern.solid(color);
    LEDPattern pattern = ledColor.blink(Units.Seconds.of(blinkTime));
    pattern.applyTo(view);
    led.setData(buffer);
  }

  /** Updates inputs */
  public void updateInputs() {}
}
