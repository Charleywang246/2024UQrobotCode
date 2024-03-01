package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedContants;

public class LED extends SubsystemBase{
    private AddressableLED led = new AddressableLED(LedContants.ledPwmPort);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LedContants.ledLenfth);

    public LED () {
        led.setLength(LedContants.ledLenfth);
        setLED(0, 0, 0);
        led.start();
    }

    public void setLED(int red, int green, int blue) {
        for(int i=0;i<buffer.getLength();i++) buffer.setRGB(i, red, green, blue);
        led.setData(buffer);
    }

}
