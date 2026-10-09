package frc.robot.subsystems.candle;

import com.ctre.phoenix6.signals.RGBWColor;

/** Constants for the CANdle subsystem */
public class CANdleConstants {
  /** The device id of the CANdle for controlling the LEDs */
  public static final int kDeviceID = 60;

  /** LED start index to apply color to */
  public static final int kFirstLED = 0;
  /** LED end index to apply color to */
  public static final int kEndLED = 100; // TODO: Get the proper end value

  /** Maximum brightness of all the LEDs for the brightness scalar */
  public static final double kFullBrightness = 1; 
  /** Minimum brightness of all the LEDs for the brightness scalar */
  public static final double kOffBrightness = 0;
  /** Default brightness of all the LEDs for the brightness scalar */
  public static final double kDefaultBrightness = 0.2;

  /** Strobe red: driver or operator manual override */
  public static final RGBWColor kManualOverrideColor = new RGBWColor(255, 0, 0, 255);
  /** Solid red: shoot-when-ready active and can shoot */
  public static final RGBWColor kHangActiveColor = new RGBWColor(255, 0, 0, 255); // TODO: SET COLOR
  /** Solid purple: shoot command is active */
  public static final RGBWColor kShootActiveColor = new RGBWColor(166, 86, 247, 255); // TODO: SET COLOR
  /** Strobe blue: intake state is "REVERSING" */
  public static final RGBWColor kIntakeOutColor = new RGBWColor(0, 0, 255, 255); // TODO: SET COLOR
  /** Solid yellow (dim): intake state is "INTAKING" */
  public static final RGBWColor kIntakeInColor = new RGBWColor(247, 233, 86, 100); // TODO: SET COLOR
  /** Solid orange: default enabled idle indication */
  public static final RGBWColor kIdleColor = new RGBWColor(255, 40, 0, 255);
 

  /** Strobe animation: manual override is on (Driver or operator) */
  public static final AnimationType kManualOverrideAnimation = AnimationType.SingleFade;
  /** Strobe animation: manual override is on (Driver or operator) */
  public static final AnimationType kLedAnimationActive = AnimationType.None;
  /** Rainbow animation: robot is disabled */
  public static final AnimationType kDisabledAnimation = AnimationType.Rainbow;

  /**
  * LED Animation type
  */
  public enum AnimationType {
      None,
      ColorFlow,
      Rainbow,
      Strobe,
      SingleFade,
      RgbFade,
      Twinkle,
      TwinkleOff,
      Larson,
      Fire,
  }
}
