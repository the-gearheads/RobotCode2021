package frc.robot.util;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class Lidar {
    private final int address;
    private final I2C lidar;
    private final Port port;

    public Lidar(Port port) {
        this(port, 0x62);
    }

    public Lidar(Port port, int address) {
        this.address = address;
        this.port = port;
        this.lidar = new I2C(this.port, this.address);
    }

    public void initialize() {
        this.lidar.write(0x02, 0x80);
        this.lidar.write(0x04, 0x08);
        this.lidar.write(0x1C, 0x00);
    }

    public int getDistance(boolean bias) {
        if (bias) {
            this.lidar.write(0x00, 0x03);
        } else {
            this.lidar.write(0x00, 0x04);
        }

        byte buffer[] = new byte[2];
        this.lidar.read(0x8F, 2, buffer);
        return (buffer[1] << 8) | buffer[0];
    }
}