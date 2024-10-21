package org.firstinspires.ftc.teamcode.hardware.i2c;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@SuppressWarnings("unused")
@I2cDeviceType()
@DeviceProperties(name = "Modern robotics Core Color Beacon", description = "Color Beacon", xmlTag = "ColorBeacon")
public class ColorBeacon extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    I2cDevice colorBeacon;
    I2cDeviceSynch colorBeaconSync;

    public ColorBeacon(I2cDeviceSynch colorBeaconSync) {
        //default address: 0x4c
        super(colorBeaconSync, true);
        this.colorBeaconSync = colorBeaconSync;
        this.colorBeaconSync.setI2cAddress(I2cAddr.create8bit(0x4c));
        super.registerArmingStateCallback(false);
        this.colorBeaconSync.engage();
        setColor(0, 0, 0);
    }

    public void setColor(int r, int g, int b) {
        colorBeaconSync.write8(4, 8); // custom color mode
        colorBeaconSync.write8(5, r); // r
        colorBeaconSync.write8(6, g); // g
        colorBeaconSync.write8(7, b); // b
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.ModernRobotics;
    }

    @Override
    public String getDeviceName() {
        return "ColorBeacon";
    }
}
