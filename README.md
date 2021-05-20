# Digital-Level-Meter
This project explores the functioning, calibration and mathematics involved in getting comprehensible data from MPU6050 (a low-cost, highly accurate IMU with 6 DOF). IMUs can be very difficult to process and set up. MPU6050 uses I2C to communicate with the microcontroller it is connected to, and always acts as a slave device. Using the raw data from the accelerometer and gyroscope, I have tried to compute Roll and Pitch. I have built upon a few codes that I found on the internet, modified them with respect to my goals, and tried to combine their most striking features to give me the most accurate data. 

# Some Important concepts
## Slave Address Selection
The MPU-60X0 always operates as a slave device when communicating to the system processor, which thus acts as the master. SDA and SCL lines typically need pull-up resistors to VDD. The maximum bus speed is 400 kHz.
The slave address of the MPU-60X0 is b110100X which is 7 bits long. The LSB bit of the 7 bit address is determined by the logic level on pin AD0. This allows two MPU-60X0s to be connected to the same I2C bus. When used in this configuration, the address of the one of the devices should be b1101000 (pin AD0 is logic low) and the address of the other should be b1101001 (pin AD0 is logic high).

---- Taken from Datasheet (Sec 9.2)

According to the above description:
If pin AD0 is logic low, address of the device is b1101000 ( = 0x68).
If pin AD0 is logic high, address of the device is b1101001 ( = 0x69).
_So in my case it is 0x68. (AD0 is connected to GND or left open)_

## Complementary Filter
Orientation data from gyroscope sensors is prone to drift significantly over time, so gyroscopic sensors are frequently combined with additional sensors, such as accelerometers or magnetometers to correct for this effect. (In this case, it is an accelerometer). The standard method of combining these two inputs is with a Kalman Filter, which is quite a complex methodology.  Fortunately, there is a simpler approximation for combining these two data types, called a Complementary Filter. Also, gyroscope gives readings in degrees per second, so we'll have to measure the angular velocity (ω) around the X, Y and Z axes at measured intervals (Δt) and then multiply it with the interval. 

>Filtered Angle = α × (Gyroscope Angle) + (1 − α) × (Accelerometer Angle)       
>Gyroscope Angle = (Last Measured Filtered Angle) + ω×Δt
>
>Where,
>
>α = τ/(τ + Δt) 
>
>Δt = sampling rate
>
>τ = time constant greater than timescale of typical accelerometer noise
>
>ω = Gyroscope reading in degrees/second (dps)

# Procedure
## (1) Setting up MPU6050 Registers 
```MPU6050_Init()``` Function
![image](https://user-images.githubusercontent.com/61982410/118938502-5b0b3600-b96c-11eb-9b1d-f0a6a6522193.png)



