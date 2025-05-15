import time
import board
import digitalio
import busio
from adafruit_lsm6ds.lsm6ds3trc import LSM6DS3TRC

class IMU:
    def __init__(self):
        # On the Seeed XIAO Sense the LSM6DS3TR-C IMU is connected on a separate
        # I2C bus and it has its own power pin that we need to enable.
        self.imupwr = digitalio.DigitalInOut(board.IMU_PWR)
        self.imupwr.direction = digitalio.Direction.OUTPUT
        self.imupwr.value = True
        time.sleep(0.1)

        self.imu_i2c = busio.I2C(board.IMU_SCL, board.IMU_SDA)
        self.sensor = LSM6DS3TRC(self.imu_i2c)
        self.velocity = [0,0,0]
        self.position = [0,0,0]
        self.rot_velocity = [0,0,0]
        self.orientation = [0,0,0]
        self.accel_offsets = [0.18, -0.02, -0.02]
        self.gyro_offsets = [0, 0.07, 0]
        self.dt = 0

    def get_acceleration(self):
        return tuple(map(sum, zip(self.sensor.acceleration, self.accel_offsets)))

    def get_rot_acceleration(self):
        return tuple(map(sum, zip(self.sensor.gyro, self.gyro_offsets)))

    def get_rot_velocity(self):
        return tuple(self.rot_velocity)

    def get_orientation(self):
        return tuple(self.orientation)

    def get_velocity(self):
        return tuple(self.velocity)

    def get_position(self):
        return tuple(self.position)

    def calculate_velocity(self):
        acc = list(self.get_acceleration())
        acc[2] -= 9.81
        self.velocity = [self.velocity[i]+acc[i]*self.dt for i in range(0,3)]

    def calculate_position(self):
        self.position = [self.position[i]+self.velocity[i]*self.dt for i in range(0,3)]

    def calculate_rot_velocity(self):
        rotaccel = self.get_rot_acceleration()
        self.rot_velocity = [self.rot_velocity[i]+rotaccel[i]*self.dt for i in range(0,3)]
        
    def calculate_orientation(self):
        self.orientation = [self.orientation[i]+self.rot_velocity[i]*self.dt for i in range(0,3)]

    def zero_velocity(self):
        self.velocity = [0,0,0]

    def zero_position(self):
        self.position = [0,0,0]

    def zero_orientation(self):
        self.rot_position = [0,0,0]

    def zero_rot_velocity(self):
        self.rot_velocity = [0,0,0]

    def update(self, dt):
        self.dt = dt
        self.calculate_velocity()
        self.calculate_position()
        self.calculate_rot_velocity()
        self.calculate_orientation()
    

# To see data on Serial Monitor:
if __name__ == '__main__':
    imu = IMU()
    dt = time.monotonic()
    while True:
        print(time.monotonic()-dt)
        imu.update(time.monotonic()-dt)
        dt = time.monotonic()
        print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (imu.get_acceleration()))
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s" % (imu.get_rot_acceleration()))
        print("velocity: X:%.2f, Y: %.2f, Z: %.2f m/s" % (imu.get_velocity()))
        print("position: X:%.2f, Y: %.2f, Z: %.2f m" % (imu.get_position()))
        print("")
        time.sleep(0.1)
