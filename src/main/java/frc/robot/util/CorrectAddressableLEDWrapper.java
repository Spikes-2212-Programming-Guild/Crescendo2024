package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.awt.*;

public class CorrectAddressableLEDWrapper {

    private final AddressableLED led;

    private final AddressableLEDBuffer ledBuffer;

    public CorrectAddressableLEDWrapper(int ledPort, int numberOfLEDs) {
        this.led = new AddressableLED(ledPort);
        led.setLength(numberOfLEDs);
        this.ledBuffer = new AddressableLEDBuffer(numberOfLEDs);
    }

    public void setStripColor(int red, int green, int blue) {
        setColorInRange(0, ledBuffer.getLength() - 1, red, green, blue);
    }

    public void setStripColor(Color color) {
        setStripColor(color.getRed(), color.getGreen(), color.getBlue());
    }

    public void turnOff() {
        setStripColor(0, 0, 0);
    }

    public void setColorInRange(int start, int end, int red, int green, int blue) {
        for (int i = start; i <= end; i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
    }

    public void setColorInRange(int start, int end, Color color) {
        setColorInRange(start, end, color.getRed(), color.getGreen(), color.getBlue());
    }

    public void setColorAt(int index, int red, int green, int blue) {
        ledBuffer.setRGB(index, red, green, blue);
    }

    public void setColorAt(int index, Color color) {
        ledBuffer.setRGB(index, color.getRed(), color.getGreen(), color.getBlue());
    }

    public void update() {
        led.setData(ledBuffer);
    }

    public void enableStrip() {
        led.start();
    }

    public void disableStrip() {
        led.stop();
    }
}
