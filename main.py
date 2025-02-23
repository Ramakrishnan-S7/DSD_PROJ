from machine import Pin, I2C
import time
from array import array

class ADXL345:
    # [Previous ADXL345 implementation remains the same]
    def __init__(self, i2c, addr=0x53):
        self.i2c = i2c
        self.addr = addr
        
        # Initialize the ADXL345
        self.i2c.writeto_mem(self.addr, 0x2D, b'\x08')  # Power on
        self.i2c.writeto_mem(self.addr, 0x31, b'\x0B')  # ±16g range, full resolution
        self.i2c.writeto_mem(self.addr, 0x2C, b'\x0A')  # 100Hz data rate
        
    def read_accel(self):
        data = self.i2c.readfrom_mem(self.addr, 0x32, 6)
        
        x = data[0] | (data[1] << 8)
        if x & (1 << 16 - 1):
            x = x - (1 << 16)
            
        y = data[2] | (data[3] << 8)
        if y & (1 << 16 - 1):
            y = y - (1 << 16)
            
        z = data[4] | (data[5] << 8)
        if z & (1 << 16 - 1):
            z = z - (1 << 16)
            
        x = x * 0.004 * 9.81
        y = y * 0.004 * 9.81
        z = z * 0.004 * 9.81
        
        return (x, y, z)

class StepCounter:
    def __init__(self, adxl345):
        self.sensor = adxl345
        self.steps = 0
        self.total_distance = 0.0  # Total distance in meters
        
        # Step length in meters (2.5 feet ≈ 0.762 meters)
        self.step_length = 0.762
        
        # Parameters for step detection
        self.window_size = 25
        self.min_step_distance = 0.04  # For detection, not actual step length
        self.min_step_time = 0.3
        self.velocity_decay = 0.95
        
        # Initialize buffers
        self.acc_x = array('f', [0] * self.window_size)
        self.acc_y = array('f', [0] * self.window_size)
        self.acc_z = array('f', [0] * self.window_size)
        self.current_index = 0
        
        # Motion tracking
        self.vel_x = self.vel_y = self.vel_z = 0
        self.pos_x = self.pos_y = self.pos_z = 0
        self.last_step_time = time.time()

    def set_step_length(self, length_meters):
        """Set the step length in meters"""
        self.step_length = length_meters
        
    def update(self):
        x, y, z = self.sensor.read_accel()
        
        # Update circular buffers
        self.acc_x[self.current_index] = x
        self.acc_y[self.current_index] = y
        self.acc_z[self.current_index] = z
        
        # Calculate average acceleration
        avg_x = sum(self.acc_x) / self.window_size
        avg_y = sum(self.acc_y) / self.window_size
        avg_z = sum(self.acc_z) / self.window_size
        
        dt = 0.01  # 100Hz sampling rate
        
        # Update velocities with decay
        self.vel_x = (self.vel_x + (x - avg_x) * dt) * self.velocity_decay
        self.vel_y = (self.vel_y + (y - avg_y) * dt) * self.velocity_decay
        self.vel_z = (self.vel_z + (z - avg_z) * dt) * self.velocity_decay
        
        # Update positions
        self.pos_x += self.vel_x * dt
        self.pos_y += self.vel_y * dt
        self.pos_z += self.vel_z * dt
        
        # Calculate displacement for step detection
        displacement = (self.pos_x**2 + self.pos_y**2 + self.pos_z**2)**0.5
        current_time = time.time()
        
        # Detect step and update distance
        if (displacement > self.min_step_distance and 
            (current_time - self.last_step_time) > self.min_step_time):
            self.steps += 1
            self.total_distance += self.step_length  # Add one step length
            self.last_step_time = current_time
            
            # Reset position tracking for next step detection
            self.pos_x = self.pos_y = self.pos_z = 0
            return True
            
        self.current_index = (self.current_index + 1) % self.window_size
        return False
    
    def get_steps(self):
        return self.steps
    
    def get_distance(self):
        """Get total distance in meters"""
        return self.total_distance
    
    def get_distance_feet(self):
        """Get total distance in feet"""
        return self.total_distance * 3.28084  # Convert meters to feet

def main():
    # Initialize I2C
    i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)
    
    try:
        adxl = ADXL345(i2c)
        stepper = StepCounter(adxl)
        
        # Optional: Set custom step length (in meters)
        # stepper.set_step_length(0.8)  # For longer steps
        # stepper.set_step_length(0.7)  # For shorter steps
        
        print("Step counter started. Press Ctrl+C to exit.")
        print("Default step length: 2.5 feet (0.762 meters)")
        
        while True:
            if stepper.update():
                steps = stepper.get_steps()
                distance_m = stepper.get_distance()
                distance_ft = stepper.get_distance_feet()
                print(f"Steps: {steps} | Distance: {distance_m:.1f}m ({distance_ft:.1f}ft)")
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        steps = stepper.get_steps()
        distance_m = stepper.get_distance()
        distance_ft = stepper.get_distance_feet()
        print(f"\nFinal measurements:")
        print(f"Steps: {steps}")
        print(f"Distance: {distance_m:.1f} meters ({distance_ft:.1f} feet)")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
