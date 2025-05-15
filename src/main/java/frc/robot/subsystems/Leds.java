package frc.robot.subsystems;

import java.util.regex.Pattern;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

public class Leds {
    private static final int leftPort = 9;
    private static final int leftLength = 120;
    private static final int middlePort = 8;
    private static final int middleLength = 120;
    private static final int rightPort = 7;
    private static final int rightLength = 120;


    private final AddressableLED leftLed;
    private final AddressableLEDBuffer leftLedBuffer;
    private final AddressableLED middleLed;
    private final AddressableLEDBuffer middleLedBuffer;
    private final AddressableLED rightLed;
    private final AddressableLEDBuffer rightLedBuffer;

    public LEDPattern leftPattern;
    public LEDPattern middlePattern;
    public LEDPattern rightPattern;
    
    public LEDPattern redOrangeBase;
    public LEDPattern redOrangeBlinkWithRsl;
    public LEDPattern greenBase;


  public Leds() {
    leftLed = new AddressableLED(leftPort);
    leftLedBuffer = new AddressableLEDBuffer(leftLength);
    leftLed.setLength(leftLength);
    leftLed.start();
    middleLed = new AddressableLED(middlePort);
    middleLedBuffer = new AddressableLEDBuffer(middleLength);
    middleLed.setLength(middleLength);
    middleLed.start();
    rightLed = new AddressableLED(rightPort);
    rightLedBuffer = new AddressableLEDBuffer(rightLength);
    rightLed.setLength(rightLength);
    rightLed.start();


    redOrangeBase = LEDPattern.solid(Color.kOrangeRed);
    redOrangeBlinkWithRsl = redOrangeBase.synchronizedBlink(RobotController::getRSLState);
    greenBase = LEDPattern.solid(Color.kGreen);

    leftPattern = redOrangeBlinkWithRsl;
    middlePattern = redOrangeBlinkWithRsl;
    rightPattern = redOrangeBlinkWithRsl;


  }

  public void applyPatternsToStrips(){
    leftPattern.applyTo(leftLedBuffer);
    leftLed.setData(leftLedBuffer);
    middlePattern.applyTo(middleLedBuffer);
    middleLed.setData(middleLedBuffer);
    rightPattern.applyTo(rightLedBuffer);
    rightLed.setData(rightLedBuffer);
  }
  
}
/*p order for left leds
 * defalt blink with rsl
 * first priority shooter motors moving (blinking Green)
 * Second priority algae mech Deployed
 * third priority deep climber out ()
 * forth priority to be determind
 * 
 *  order for middle leds: used for detecting errors
 * defalt light green 
 * first priority overheating CPU: above 85 celsius
 * second priority can error
 * third priority  comms error
 * last priority battery voltage low (yellow), browning out (brown)
 * 
 */