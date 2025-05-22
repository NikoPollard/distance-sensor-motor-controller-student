import time
import board
import digitalio
import busio
from adafruit_lsm6ds.lsm6ds3trc import LSM6DS3TRC
import math

class IMU:
    def __init__(self):
        # Hardware setup
        self.imupwr = digitalio.DigitalInOut(board.IMU_PWR)
        self.imupwr.direction = digitalio.Direction.OUTPUT
        self.imupwr.value = True
        time.sleep(0.1)

        self.imu_i2c = busio.I2C(board.IMU_SCL, board.IMU_SDA)
        self.sensor = LSM6DS3TRC(self.imu_i2c)
        
        # State variables
        self.velocity = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.0]
        self.rot_velocity = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0]  # Roll, Pitch, Yaw in radians
        
        # Filtered orientation for complementary filter
        self.filtered_orientation = [0.0, 0.0, 0.0]
        
        # Calibration offsets
        self.accel_offsets = [0.0, 0.0, 0.0]
        self.gyro_offsets = [0.0, 0.0, 0.0]
        
        # Filter parameters
        self.alpha = 0.96  # Complementary filter coefficient (trust gyro more)
        self.use_complementary_filter = True
        
        # Zero velocity update parameters
        self.velocity_threshold = 0.02   # m/s - threshold for considering stationary
        self.accel_threshold = 0.08      # m/s² - acceleration noise threshold
        self.gyro_threshold = 0.05       # rad/s - gyro noise threshold
        self.stationary_count = 0
        self.stationary_threshold = 100  # samples before applying ZUPT
        self.max_stationary_time = 300   # max samples to stay in stationary state
        
        # Moving average for stability detection
        self.accel_history = [[0.0, 0.0, 0.0] for _ in range(10)]
        self.gyro_history = [[0.0, 0.0, 0.0] for _ in range(10)]
        self.history_index = 0
        
        self.dt = 0
        self.gravity = 9.81
        
        print("IMU initialized with advanced filtering")

    def get_acceleration(self):
        """Get calibrated acceleration in body frame"""
        raw_accel = self.sensor.acceleration
        return [raw_accel[i] + self.accel_offsets[i] for i in range(3)]

    def get_rot_acceleration(self):
        """Get calibrated angular velocity (gyroscope)"""
        raw_gyro = self.sensor.gyro
        return [raw_gyro[i] + self.gyro_offsets[i] for i in range(3)]

    def get_rot_velocity(self):
        return tuple(self.rot_velocity)

    def get_orientation(self):
        """Return the filtered orientation if complementary filter is enabled"""
        if self.use_complementary_filter:
            return tuple(self.filtered_orientation)
        return tuple(self.orientation)

    def get_velocity(self):
        return tuple(self.velocity)

    def get_position(self):
        return tuple(self.position)

    def rotation_matrix_from_euler(self, roll, pitch, yaw):
        """Create rotation matrix from Euler angles (ZYX convention)"""
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        
        # Rotation matrix from body to world frame
        R = [
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ]
        ]
        return R

    def rotate_vector_to_world(self, vector):
        """Rotate vector from body frame to world frame"""
        roll, pitch, yaw = self.get_orientation()
        R = self.rotation_matrix_from_euler(roll, pitch, yaw)
        
        # Matrix-vector multiplication
        world_vector = [0.0, 0.0, 0.0]
        for i in range(3):
            for j in range(3):
                world_vector[i] += R[i][j] * vector[j]
        
        return world_vector

    def calculate_accel_angles(self, accel):
        """Calculate roll and pitch from accelerometer readings"""
        ax, ay, az = accel
        
        # Avoid division by zero and handle edge cases
        if abs(az) < 0.1:  # Nearly vertical
            roll = 0.0 if ay >= 0 else math.pi
            pitch = math.pi/2 if ax >= 0 else -math.pi/2
        else:
            roll = math.atan2(ay, az)
            pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        
        return roll, pitch

    def update_moving_average(self, accel, gyro):
        """Update moving average history for stability detection"""
        self.accel_history[self.history_index] = accel[:]
        self.gyro_history[self.history_index] = gyro[:]
        self.history_index = (self.history_index + 1) % len(self.accel_history)

    def is_stationary(self):
        """Determine if IMU is stationary based on recent history"""
        # Calculate variance in recent measurements
        accel_var = [0.0, 0.0, 0.0]
        gyro_var = [0.0, 0.0, 0.0]
        
        # Calculate means
        accel_mean = [0.0, 0.0, 0.0]
        gyro_mean = [0.0, 0.0, 0.0]
        
        for i in range(len(self.accel_history)):
            for j in range(3):
                accel_mean[j] += self.accel_history[i][j]
                gyro_mean[j] += self.gyro_history[i][j]
        
        for j in range(3):
            accel_mean[j] /= len(self.accel_history)
            gyro_mean[j] /= len(self.gyro_history)
        
        # Calculate variance
        for i in range(len(self.accel_history)):
            for j in range(3):
                accel_var[j] += (self.accel_history[i][j] - accel_mean[j]) ** 2
                gyro_var[j] += (self.gyro_history[i][j] - gyro_mean[j]) ** 2
        
        # Check if variance is low (indicating stability)
        accel_stable = all(var/len(self.accel_history) < self.accel_threshold**2 for var in accel_var)
        gyro_stable = all(var/len(self.gyro_history) < self.gyro_threshold**2 for var in gyro_var)
        velocity_low = all(abs(v) < self.velocity_threshold for v in self.velocity)
        
        return accel_stable and gyro_stable and velocity_low

    def apply_complementary_filter(self):
        """Apply complementary filter for orientation estimation"""
        accel = self.get_acceleration()
        gyro = self.get_rot_acceleration()
        
        # Update gyro-based orientation (high-pass)
        for i in range(3):
            self.orientation[i] += gyro[i] * self.dt
        
        # Calculate accelerometer-based roll and pitch (low-pass)
        if abs(accel[0]) < 15 and abs(accel[1]) < 15 and abs(accel[2]) < 15:  # Reasonable acceleration range
            accel_roll, accel_pitch = self.calculate_accel_angles(accel)
            
            # Apply complementary filter for roll and pitch
            self.filtered_orientation[0] = self.alpha * self.orientation[0] + (1 - self.alpha) * accel_roll
            self.filtered_orientation[1] = self.alpha * self.orientation[1] + (1 - self.alpha) * accel_pitch
            
            # Yaw cannot be corrected by accelerometer alone
            self.filtered_orientation[2] = self.orientation[2]
        else:
            # If acceleration is too high (dynamic motion), trust gyro completely
            self.filtered_orientation = self.orientation[:]

    def calculate_velocity_and_position(self):
        """Calculate velocity and position in world frame with ZUPT"""
        # Get body frame acceleration
        body_accel = self.get_acceleration()
        gyro = self.get_rot_acceleration()
        
        # Update moving average for stationary detection
        self.update_moving_average(body_accel, gyro)
        
        # Transform acceleration to world frame
        world_accel = self.rotate_vector_to_world(body_accel)
        
        # Remove gravity (gravity acts in negative Z direction in world frame)
        world_accel[2] -= self.gravity
        
        # Apply noise thresholding in world frame
        for i in range(3):
            if abs(world_accel[i]) < self.accel_threshold:
                world_accel[i] = 0.0
        
        # Zero Velocity Update (ZUPT) logic
        if self.is_stationary():
            self.stationary_count += 1
            if self.stationary_count > self.stationary_threshold:
                # Apply ZUPT - zero the velocity
                self.velocity = [0.0, 0.0, 0.0]
                # Don't update position while stationary
                return
        else:
            self.stationary_count = 0
        
        # Limit stationary time to prevent getting stuck
        if self.stationary_count > self.max_stationary_time:
            self.stationary_count = 0
        
        # Update velocity with world frame acceleration
        prev_velocity = self.velocity[:]
        for i in range(3):
            self.velocity[i] += world_accel[i] * self.dt
        
        # Update position using trapezoidal integration
        for i in range(3):
            avg_velocity = (prev_velocity[i] + self.velocity[i]) / 2
            self.position[i] += avg_velocity * self.dt

    def zero_velocity(self):
        self.velocity = [0.0, 0.0, 0.0]

    def zero_position(self):
        self.position = [0.0, 0.0, 0.0]

    def zero_orientation(self):
        self.orientation = [0.0, 0.0, 0.0]
        self.filtered_orientation = [0.0, 0.0, 0.0]

    def zero_rot_velocity(self):
        self.rot_velocity = [0.0, 0.0, 0.0]

    def reset_all(self):
        """Reset all calculated values"""
        self.zero_velocity()
        self.zero_position()
        self.zero_orientation()
        self.zero_rot_velocity()
        self.stationary_count = 0

    def force_zupt(self):
        """Force a Zero Velocity Update - useful for manual correction"""
        self.velocity = [0.0, 0.0, 0.0]
        self.stationary_count = 0
        print("Manual ZUPT applied")

    def update(self, dt):
        """Main update function with advanced filtering"""
        self.dt = dt
        
        if self.use_complementary_filter:
            # Apply complementary filter for orientation
            self.apply_complementary_filter()
        else:
            # Simple gyro integration
            gyro = self.get_rot_acceleration()
            for i in range(3):
                self.orientation[i] += gyro[i] * self.dt
        
        # Calculate velocity and position in world frame
        self.calculate_velocity_and_position()

    def calibrate(self, samples=3000):
        """Enhanced calibration procedure"""
        print("=== IMU Calibration ===")
        print("IMPORTANT: Keep the IMU completely stationary and level!")
        print("Calibration will take about 30 seconds...")
        
        # Wait for user to position IMU
        for i in range(5, 0, -1):
            print(f"Starting in {i} seconds...")
            time.sleep(1)
        
        lastTime = time.monotonic()
        averageAccel = [0.0, 0.0, 0.0]
        averageRot = [0.0, 0.0, 0.0]
        self.accel_offsets = [0.0, 0.0, 0.0]
        self.gyro_offsets = [0.0, 0.0, 0.0]
        
        print("Calibrating...")
        
        for sample in range(samples):
            current_time = time.monotonic()
            self.dt = current_time - lastTime
            lastTime = current_time
            
            # Get raw readings
            raw_accel = self.sensor.acceleration
            raw_gyro = self.sensor.gyro
            
            for i in range(3):
                averageAccel[i] += raw_accel[i]
                averageRot[i] += raw_gyro[i]
            
            if sample % 300 == 0:
                progress = int((sample / samples) * 20)
                bar = "█" * progress + "░" * (20 - progress)
                print(f"Progress: [{bar}] {sample}/{samples}")
            
            time.sleep(0.01)  # 100Hz sampling
        
        # Calculate offsets
        for i in range(3):
            self.accel_offsets[i] = -averageAccel[i] / samples
            self.gyro_offsets[i] = -averageRot[i] / samples
        
        # Adjust Z-axis accelerometer offset for gravity
        self.accel_offsets[2] += self.gravity
        
        print("✓ Calibration complete!")
        print(f"Accelerometer offsets: [{self.accel_offsets[0]:.4f}, {self.accel_offsets[1]:.4f}, {self.accel_offsets[2]:.4f}]")
        print(f"Gyroscope offsets: [{self.gyro_offsets[0]:.4f}, {self.gyro_offsets[1]:.4f}, {self.gyro_offsets[2]:.4f}]")

    def get_status_string(self):
        """Get formatted status string for debugging"""
        accel = self.get_acceleration()
        gyro = self.get_rot_acceleration()
        vel = self.get_velocity()
        pos = self.get_position()
        orient = self.get_orientation()
        
        status = f"A:[{accel[0]:6.3f},{accel[1]:6.3f},{accel[2]:6.3f}] "
        status += f"G:[{gyro[0]:6.3f},{gyro[1]:6.3f},{gyro[2]:6.3f}] "
        status += f"V:[{vel[0]:6.3f},{vel[1]:6.3f},{vel[2]:6.3f}] "
        status += f"P:[{pos[0]:6.3f},{pos[1]:6.3f},{pos[2]:6.3f}] "
        status += f"O:[{math.degrees(orient[0]):5.1f},{math.degrees(orient[1]):5.1f},{math.degrees(orient[2]):5.1f}]° "
        
        if self.stationary_count > self.stationary_threshold:
            status += "[ZUPT]"
        elif self.stationary_count > 0:
            status += f"[{self.stationary_count}]"
        
        return status

# Main execution with enhanced features
if __name__ == '__main__':
    imu = IMU()
    imu.calibrate()
    
    # Reset all values after calibration
    imu.reset_all()
    
    last_time = time.monotonic()
    print_counter = 0
    
    print("\n=== IMU Data Stream ===")
    print("A=Acceleration, G=Gyro, V=Velocity, P=Position, O=Orientation")
    print("Commands: Press Ctrl+C to exit")
    
    try:
        while True:
            current_time = time.monotonic()
            dt = current_time - last_time
            last_time = current_time
            
            imu.update(dt)
            
            # Print data every 20 samples to avoid flooding (5Hz display rate at 100Hz update)
            print_counter += 1
            if print_counter >= 20:
                print_counter = 0
                print(imu.get_status_string())
            
            time.sleep(0.01)  # 100Hz update rate
            
    except KeyboardInterrupt:
        print("\nIMU data logging stopped")
        final_pos = imu.get_position()
        print(f"Final position: X={final_pos[0]:.3f}m, Y={final_pos[1]:.3f}m, Z={final_pos[2]:.3f}m")
