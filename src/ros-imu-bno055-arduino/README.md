# ros-imu-bno055-arduino

This is to use a BNO055 + Arduino Pro Micro as a USB IMU. This is NOT a recommended way to connect an IMU for production robots, but is helpful to slap an IMU onto any computer that doesn't have an exposed I2C port, e.g. a NUC, laptop, or desktop PC, especially for hobbyists.

If you are looking for a BNO055 pure I2C driver for production use, please see https://github.com/dheera/ros-imu-bno055

# Assembly instructions

You will need an Adafruit BNO055 IMU board, an Arduino 5V Pro Micro, 1x3 header, and a small wire.

The header is used to connect the GND, SDA, SCL pins, which line up nicely between the Pro Micro and the Adafruit board.

The wire is used to connect the power separately to pin 10. Section 29.2 of the 32u4 datasheet suggests that each GPIO pin can sink upto 20 mA. The Adafruit BNO055 consumes about 12.5 mA in steady state, so it can be powered by a GPIO pin, which allows easy power resetting if necessary.

![image](/images/assembly0.jpg "assembly0")

![image](/images/assembly1.jpg "assembly1")
GND, SDA, SCL pins line up nicely.

![image](/images/assembly2.jpg "assembly2")

![image](/images/assembly3.jpg "assembly3")
