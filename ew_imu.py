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
        
        # Motion detection parameters
        self.motion_accel_threshold = 2.0    # m/s² - threshold for detecting motion vs stationary
        self.motion_gyro_threshold = 0.1     # rad/s - threshold for detecting rotation

        # Reduce the stationary threshold since we have better detection now
        self.stationary_threshold = 50       # Reduced from 100
        self.velocity_threshold = 0.05       # Slightly higher threshold

        # Optional: Add high-pass filter for acceleration to remove slow drift
        self.enable_accel_highpass = True
        self.accel_highpass_alpha = 0.95     # High-pass filter coefficient
        self.accel_highpass_prev = [0.0, 0.0, 0.0]
        
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
        """Apply motion-aware complementary filter for orientation estimation"""
        accel = self.get_acceleration()
        gyro = self.get_rot_acceleration()
        
        # Calculate total acceleration magnitude
        accel_magnitude = math.sqrt(accel[0]**2 + accel[1]**2 + accel[2]**2)
        
        # Calculate gyro magnitude (how much rotation is happening)
        gyro_magnitude = math.sqrt(gyro[0]**2 + gyro[1]**2 + gyro[2]**2)
        
        # Adaptive alpha based on motion
        # When stationary (accel ≈ gravity, low gyro), trust accelerometer more
        # When moving (accel ≠ gravity, high gyro), trust gyro more
        
        base_alpha = self.alpha  # Your original 0.96
        
        # Factor 1: How close is acceleration to gravity magnitude
        gravity_error = abs(accel_magnitude - self.gravity)
        if gravity_error < 1.0:  # Close to gravity - probably stationary
            alpha_from_accel = base_alpha * 0.7  # Reduce alpha (trust accel more)
        elif gravity_error < 3.0:  # Moderate motion
            alpha_from_accel = base_alpha * 0.9
        else:  # High acceleration - definitely moving
            alpha_from_accel = 0.98  # Trust gyro almost completely
        
        # Factor 2: Gyro activity
        if gyro_magnitude > 0.2:  # Significant rotation
            alpha_from_gyro = 0.98  # Trust gyro more during rotation
        elif gyro_magnitude > 0.05:  # Moderate rotation
            alpha_from_gyro = base_alpha * 1.1
        else:  # Low rotation
            alpha_from_gyro = base_alpha
        
        # Combine factors - use the higher alpha (more gyro trust)
        adaptive_alpha = min(0.99, max(alpha_from_accel, alpha_from_gyro))
        
        # Calculate accelerometer-based roll and pitch
        if abs(accel[0]) < 20 and abs(accel[1]) < 20 and abs(accel[2]) < 20:  # Reasonable range
            accel_roll, accel_pitch = self.calculate_accel_angles(accel)
            
            # Apply adaptive complementary filter
            self.filtered_orientation[0] = adaptive_alpha * (self.filtered_orientation[0] + gyro[0] * self.dt) + (1 - adaptive_alpha) * accel_roll
            self.filtered_orientation[1] = adaptive_alpha * (self.filtered_orientation[1] + gyro[1] * self.dt) + (1 - adaptive_alpha) * accel_pitch
            
            # Yaw - only gyro integration
            self.filtered_orientation[2] += gyro[2] * self.dt
            
        else:
            # Extreme acceleration - trust gyro completely
            for i in range(3):
                self.filtered_orientation[i] += gyro[i] * self.dt
        
        # Keep angles in reasonable range
        for i in range(3):
            while self.filtered_orientation[i] > math.pi:
                self.filtered_orientation[i] -= 2 * math.pi
            while self.filtered_orientation[i] < -math.pi:
                self.filtered_orientation[i] += 2 * math.pi
        
        # Update basic orientation for consistency
        self.orientation = self.filtered_orientation[:]
        
        # # Debug output (remove after testing)
        # if hasattr(self, 'debug_counter'):
        #     self.debug_counter += 1
        # else:
        #     self.debug_counter = 0
        
        # if self.debug_counter % 100 == 0:  # Print every second at 100Hz
        #     print(f"Motion: Alpha={adaptive_alpha:.3f}, AccelMag={accel_magnitude:.2f}, GyroMag={gyro_magnitude:.3f}")

    def is_stationary(self):
        """Enhanced stationary detection that considers motion state"""
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
        
        # Check if variance is low AND acceleration magnitude is close to gravity
        accel_stable = all(var/len(self.accel_history) < self.accel_threshold**2 for var in accel_var)
        gyro_stable = all(var/len(self.gyro_history) < self.gyro_threshold**2 for var in gyro_var)
        velocity_low = all(abs(v) < self.velocity_threshold for v in self.velocity)
        
        # Additional check: is average acceleration magnitude close to gravity?
        avg_accel_magnitude = math.sqrt(sum(accel_mean[i]**2 for i in range(3)))
        gravity_consistent = abs(avg_accel_magnitude - self.gravity) < 1.0
        
        return accel_stable and gyro_stable and velocity_low and gravity_consistent
        # return False

    def calculate_velocity_and_position(self):
        """Calculate velocity and position in world frame with ZUPT - IMPROVED VERSION"""
        # Get body frame acceleration
        body_accel = self.get_acceleration()
        gyro = self.get_rot_acceleration()
        
        # Update moving average for stationary detection
        self.update_moving_average(body_accel, gyro)
        
        # Transform acceleration to world frame
        world_accel = self.rotate_vector_to_world(body_accel)
        
        # Store pre-gravity-removal values for debugging
        world_accel_before_gravity = world_accel[:]
        
        # Remove gravity (gravity acts in negative Z direction in world frame)
        world_accel[2] -= self.gravity
        
        # Debug: Check if gravity removal seems reasonable
        gravity_removed_magnitude = abs(world_accel_before_gravity[2] - world_accel[2])
        if abs(gravity_removed_magnitude - self.gravity) > 2.0:  # More than 2 m/s² error
            print(f"WARNING: Gravity removal may be inaccurate. Expected ~{self.gravity:.1f}, got {gravity_removed_magnitude:.1f}")
            print(f"  Orientation (deg): Roll={math.degrees(self.filtered_orientation[0]):.1f}, Pitch={math.degrees(self.filtered_orientation[1]):.1f}, Yaw={math.degrees(self.filtered_orientation[2]):.1f}")
        
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
        
        # Apply velocity decay to combat drift (optional)
        velocity_decay = 0.995  # Very slight decay to prevent unbounded growth
        for i in range(3):
            self.velocity[i] *= velocity_decay
        
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
