package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {

  public final LEDIO[] io;
  private final LEDIOInputsAutoLogged[] inputs;

  /**
   * Sets LEDIO to be the io
   *
   * @param io the input io
   */
  public LED(LEDIO... io) {
    this.io = io;
    inputs = new LEDIOInputsAutoLogged[io.length];
  }

  /** Logs the io inputs */
  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("LED/Section" + i, inputs[i]);
    }
  }

  /**
   * Turns the LED color to red. The LED will remain the color that is commanded indefinitely unless
   * cancelled or interrupted
   *
   * @param sectionNum section number for the LED view
   * @return command to turn the LED color to red
   */
  public Command turnRed(int sectionNum) {
    return Commands.run(() -> io[sectionNum].setColor(Color.kRed), this);
  }

  /**
   * Turns the LED color to yellow. The LED will remain the color that is commanded indefinitely
   * unless cancelled or interrupted
   *
   * @param sectionNum section number for the LED view
   * @return command to turn the LED color to yellow
   */
  public Command turnYellow(int sectionNum) {
    return Commands.run(() -> io[sectionNum].setColor(Color.kYellow), this);
  }

  /**
   * Turns the LED color to green. The LED will remain the color that is commanded indefinitely
   * unless cancelled or interrupted
   *
   * @param sectionNum section number for the LED view
   * @return command to turn the LED color to green
   */
  public Command turnGreen(int sectionNum) {
    return Commands.run(() -> io[sectionNum].setColor(Color.kGreen), this);
  }

  /**
   * Turns the LED color to orange. The LED will remain the color that is commanded indefinitely
   * unless cancelled or interrupted
   *
   * @param sectionNum section number for the LED view
   * @return command to turn the LED color to orange
   */
  public Command turnOrange(int sectionNum) {
    return Commands.run(() -> io[sectionNum].setColor(Color.kOrange), this);
  }

  /**
   * Makes the LED blink red. The LED will remain blinking indefinitely unless cancelled or
   * interrupted
   *
   * @param sectionNum section number for the LED view
   * @return command to make the the LED color blink to red
   */
  public Command blinkRed(int sectionNum) {
    return Commands.run(() -> io[sectionNum].setBlink(Color.kRed, LEDConstants.BLINK_TIME), this);
  }
}
