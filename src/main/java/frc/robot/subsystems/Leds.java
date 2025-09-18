package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

public class Leds {
  private static final int ledPort = 0;
  private static final int ledLangth = 60;

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  private final AddressableLEDBufferView leftLedBuffer;
  private final AddressableLEDBufferView middleLedBuffer;
  private final AddressableLEDBufferView rightLedBuffer;

  public LEDPattern leftPattern;
  public LEDPattern middlePattern;
  public LEDPattern rightPattern;

  public LEDPattern elevatorProgressMask;
  public LEDPattern redOrangeBase;
  public LEDPattern redOrangeBlinkWithRsl;
  public LEDPattern redBase;
  public LEDPattern redProgressMaskWithElevator;
  public LEDPattern redSlowBlink;
  public LEDPattern greenBase;
  public LEDPattern lightGreenBase;
  public LEDPattern blueBase;
  public LEDPattern blueBlink;
  public LEDPattern blueProgressMaskWithElevator;
  public LEDPattern whiteBase;
  public LEDPattern whiteBlink;
  public LEDPattern brownBase;
  public LEDPattern yellowBase;

  public Leds(Elevator elevator) {
    ledStrip = new AddressableLED(ledPort);
    ledStrip.setLength(ledLangth);
    ledBuffer = new AddressableLEDBuffer(ledLangth);
    rightLedBuffer = ledBuffer.createView(0, 19);
    middleLedBuffer = ledBuffer.createView(20, 39).reversed();
    leftLedBuffer = ledBuffer.createView(40, 59);
    ledStrip.start();

    elevatorProgressMask =
        LEDPattern.progressMaskLayer(() -> elevator.getCurrentPosition() / elevator.LEVELBARGE);

    redOrangeBase = LEDPattern.solid(Color.kOrangeRed);
    redOrangeBlinkWithRsl = redOrangeBase.synchronizedBlink(RobotController::getRSLState);

    redBase = LEDPattern.solid(Color.kRed);
    redProgressMaskWithElevator = redBase.mask(elevatorProgressMask);
    redSlowBlink = redBase.blink(Seconds.of(2), Seconds.of(1));

    greenBase = LEDPattern.solid(Color.kGreen);

    lightGreenBase = LEDPattern.solid(Color.kLightGreen);

    blueBase = LEDPattern.solid(Color.kBlue);
    blueBlink = blueBase.blink(Seconds.of(0.5));
    blueProgressMaskWithElevator = blueBase.mask(elevatorProgressMask);

    whiteBase = LEDPattern.solid(Color.kWhite);
    whiteBlink = whiteBase.blink(Seconds.of(0.5));

    brownBase = LEDPattern.solid(Color.kBrown);

    yellowBase = LEDPattern.solid(Color.kYellow);

    leftPattern = redOrangeBlinkWithRsl;
    middlePattern = redOrangeBlinkWithRsl;
    rightPattern = redOrangeBlinkWithRsl;
  }

  public void applyPatternsToStrips() {
    leftPattern.applyTo(leftLedBuffer);
    middlePattern.applyTo(middleLedBuffer);
    rightPattern.applyTo(rightLedBuffer);
    ledStrip.setData(ledBuffer);
  }
}
/*p order for left leds
 * defalt blink with rsl
 * first priority shooter motors moving (blinking Green)
 * Second priority algae mech Deployed(blinkingblue)
 * third priority deep climber out (white)
 * forth priority to be determind
 *
 *  order for middle leds: used for detecting errors
 * defalt light green
 * first priority overheating CPU: above 85 celsius
 * second priority comms error
 * last priority battery voltage low (yellow), browning out (brown)
 *
 * right leds are a progress mark of elevator height (red)
 */
