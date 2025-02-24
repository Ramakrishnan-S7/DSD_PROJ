from machine import Pin, I2C
import time
from array import array
from pico_i2c_lcd import I2cLcd

#library for ADXL345
class ADXL345:
    def __init__(self, i2c, addr=0x53):
        self.i2c = i2c
        self.addr = addr
        
      
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
    def __init__(self, adxl345, lcd):
        self.sensor = adxl345
        self.lcd = lcd
        self.steps = 0
        self.total_distance = 0.0
        
        # Step length in meters (2.5 feet ≈ 0.762 meters)
        self.step_length = 0.762
        
        # min req to count step
        self.window_size = 25
        self.min_step_distance = 0.04
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
        
        # Stabilization flag and counter
        self.is_stabilized = False
        self.stabilization_samples = 0
        self.required_samples = 50  # Number of samples needed before starting detection
        
        # Initialize LCD
        self.lcd.clear()
        self.lcd.move_to(0, 0)
        self.lcd.putstr("Calibrating...")
        self.lcd.move_to(0, 1)
        self.lcd.putstr("Please wait")
        
        # Initial buffer fill
        self._fill_initial_buffers()

    def _fill_initial_buffers(self):
        """Fill the buffers with initial readings to stabilize the sensor"""
        for _ in range(self.window_size):
            x, y, z = self.sensor.read_accel()
            self.acc_x[self.current_index] = x
            self.acc_y[self.current_index] = y
            self.acc_z[self.current_index] = z
            self.current_index = (self.current_index + 1) % self.window_size
            time.sleep(0.01)

    def set_step_length(self, length_meters):
        self.step_length = length_meters
        
    def update_display(self):
        #print steps and distance
        self.lcd.clear()
        self.lcd.move_to(0, 0)
        self.lcd.putstr(f"Steps: {self.steps}")
        self.lcd.move_to(0, 1)
        distance_ft = self.total_distance * 3.28084
        self.lcd.putstr(f"Dist:{distance_ft:.1f}ft")
        
    def update(self):
        x, y, z = self.sensor.read_accel()
        
        # Handle stabilization period
        if not self.is_stabilized:
            self.stabilization_samples += 1
            if self.stabilization_samples >= self.required_samples:
                self.is_stabilized = True
                self.update_display()  # Show initial zeros
                return False
            return False
        
        # Update circular buffers
        self.acc_x[self.current_index] = x
        self.acc_y[self.current_index] = y
        self.acc_z[self.current_index] = z
        
        # Calculate average acceleration
        avg_x = sum(self.acc_x) / self.window_size
        avg_y = sum(self.acc_y) / self.window_size
        avg_z = sum(self.acc_z) / self.window_size
        
        dt = 0.01
        
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
            self.total_distance += self.step_length
            self.last_step_time = current_time
            
            # Update LCD display
            self.update_display()
            
            # Reset position tracking for next step detection
            self.pos_x = self.pos_y = self.pos_z = 0
            return True
            
        self.current_index = (self.current_index + 1) % self.window_size
        return False

def main():
    # Initialize I2C for ADXL345
    i2c_sensor = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)
    
    # Initialize I2C for LCD
    i2c_lcd = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
    
    try:
        # Initialize LCD 
        lcd = I2cLcd(i2c_lcd, 0x27, 2, 16)  
        
        # Initialize ADXL345
        adxl = ADXL345(i2c_sensor)
        
        # Create step counter with LCD
        stepper = StepCounter(adxl, lcd)
        
        print("Step counter started. Press Ctrl+C to exit.")
        
        while True:
            stepper.update()
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        lcd.clear()
        lcd.putstr("Counter stopped")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
