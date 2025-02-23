from machine import Pin, I2C
import time

class ADXL345:
    def __init__(self, i2c, addr=0x53):
        self.i2c = i2c
        self.addr = addr
        
        # Initialize the ADXL345
        self.i2c.writeto_mem(self.addr, 0x2D, b'\x08')  # Power on
        self.i2c.writeto_mem(self.addr, 0x31, b'\x0B')  # ±16g range, full resolution
        self.i2c.writeto_mem(self.addr, 0x2C, b'\x0A')  # 100Hz data rate
        
    def read_accel(self):
        # Read 6 bytes of data starting from register 0x32
        data = self.i2c.readfrom_mem(self.addr, 0x32, 6)
        
        # Convert the data to 16-bit integers
        x = data[0] | (data[1] << 8)
        if x & (1 << 16 - 1):
            x = x - (1 << 16)
            
        y = data[2] | (data[3] << 8)
        if y & (1 << 16 - 1):
            y = y - (1 << 16)
            
        z = data[4] | (data[5] << 8)
        if z & (1 << 16 - 1):
            z = z - (1 << 16)
            
        # Convert to g force (±16g range)
        x = x * 0.004 
        y = y * 0.004
        z = z * 0.004
        
        return (x, y, z)

class StepCounter:
    def __init__(self, adxl345):
        self.sensor = adxl345
        self.steps = 0
        self.last_peak = 0
        self.threshold = 1.2  # Acceleration threshold for step detection (in g)
        self.min_step_time = 0.3  # Minimum time between steps (in seconds)
        
    def update(self):
        x, y, z = self.sensor.read_accel()
        # Calculate total acceleration magnitude
        total_accel = (x*x + y*y + z*z) ** 0.5
        
        current_time = time.time()
        
        # Check if we detected a step
        if (total_accel > self.threshold and 
            (current_time - self.last_peak) > self.min_step_time):
            self.steps += 1
            self.last_peak = current_time
            return True
        return False
    
    def get_steps(self):
        return self.steps

def main():
    # Initialize I2C
    i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)
    
    # Initialize ADXL345
    try:
        adxl = ADXL345(i2c)
    except:
        print("Error: Could not initialize ADXL345")
        return
    
    # Create step counter
    stepper = StepCounter(adxl)
    
    # Main loop
    print("Step counter started. Press Ctrl+C to exit.")
    try:
        while True:
            if stepper.update():
                print(f"Steps: {stepper.get_steps()}")
            time.sleep(0.01)  # Small delay to prevent overwhelming the sensor
            
    except KeyboardInterrupt:
        print(f"\nFinal step count: {stepper.get_steps()}")

if __name__ == "__main__":
    main()
