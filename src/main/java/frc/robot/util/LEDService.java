package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotMap;

/**
 * Class which is responsible for doing cool animations with the LED strips.
 */
public class LEDService extends CorrectAddressableLEDWrapper {

    private static final int NUMBER_OF_LEDS = 150;

    private int animation;

    private static LEDService instance;

    public static LEDService getInstance() {
        if (instance == null) {
            instance = new LEDService(RobotMap.PWM.LED, NUMBER_OF_LEDS);
        }
        return instance;
    }

    private LEDService(int port, int numberOfLEDs) {
        super(port, numberOfLEDs);
        enableStrip();
        animation = 0;
    }

    public void periodic() {
        animation++;
        animation %= NUMBER_OF_LEDS;
        update();
    }

    public void attemptIntake() {
        for (int i = 0; i < NUMBER_OF_LEDS; i++) {
            setColorAt((i + animation) % NUMBER_OF_LEDS, 255, NUMBER_OF_LEDS - i, 100 + i);
        }
    }

    public void intakeSuccessful() {
        new Thread(() -> {
            try {
                for (int i = 0; i < 4; i++) {
                    setStripColor(255, 0, 255);
                    Thread.sleep(125);
                    turnOff();
                    Thread.sleep(125);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }).start();
    }

    public void attemptShoot() {
        for (int i = 0; i < NUMBER_OF_LEDS / 2; i++) {
            setColorAt((i + animation) % NUMBER_OF_LEDS / 2, 255, (int) (i * 3.4), 0);
        }
        for (int i = NUMBER_OF_LEDS / 2; i < NUMBER_OF_LEDS; i++) {
            setColorAt((i - NUMBER_OF_LEDS / 2 + animation) % 75 + 75, 255, 255 - (int) (i * 3.4), 0);
        }
        animation %= 75;
    }

    public void shootSuccessful() {
        for (int i = 0; i < NUMBER_OF_LEDS; i++) {
            if (i % 20 == 0) setColorAt((i + animation * 2) % NUMBER_OF_LEDS, 255, 255, 0);
            else setColorAt((i + animation * 2) % NUMBER_OF_LEDS, 0, 0, 0);
        }
    }

    public void preGame() {
        if (DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            for (int i = 0; i < NUMBER_OF_LEDS; i++) {
                setColorAt((i + animation) % NUMBER_OF_LEDS, i * NUMBER_OF_LEDS / 255, 0, 255);
            }
        } else {
            for (int i = 0; i < NUMBER_OF_LEDS; i++) {
                setColorAt((i + animation) % NUMBER_OF_LEDS, 255, i * NUMBER_OF_LEDS / 255, 0);
            }
        }
    }
}
