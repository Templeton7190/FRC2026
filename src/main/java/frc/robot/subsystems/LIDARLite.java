package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class LIDARLite {
    private static final int DEVICE_ADDRESS = 0x62;
    private static final int ACQ_COMMAND = 0x00;
    private static final int STATUS = 0x01;
    private static final int DISTANCE_REGISTER = 0x8f;
    private static final int MEASURE = 0x04;

    private final I2C lidar;

    public LIDARLite(I2C.Port port) {
        lidar = new I2C(port, DEVICE_ADDRESS);
        System.out.println("�o. LIDAR-Lite initialized on " + port.name() +
            ", connected: " + lidar.addressOnly());
    }

    public void startMeasuring() {
        writeRegister(0x04, 0x08 | 32); // default plus bit 5
        writeRegister(0x11, 0xff);
        writeRegister(ACQ_COMMAND, MEASURE);
    }

    public void stopMeasuring() {
        writeRegister(0x11, 0x00);
    }

    public int getDistance() {
        byte[] buffer = new byte[2];

        // Trigger measurement
        lidar.write(ACQ_COMMAND, MEASURE);
        Timer.delay(0.05);

        // Read back distance
        boolean failed = lidar.read(DISTANCE_REGISTER, 2, buffer);
        if (failed) {
            System.out.println("�s��,? LIDAR read failed!");
            return -1;
        }

        // Combine two bytes
        return ((buffer[0] & 0xFF) << 8) | (buffer[1] & 0xFF);
    }

    private void writeRegister(int address, int value) {
        lidar.write(address, value);
    }
}
