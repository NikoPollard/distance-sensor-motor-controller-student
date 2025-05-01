import board
import adafruit_vl53l4cd
import time

i2c = board.I2C()  # uses board.SCL and board.SDA
vl53 = adafruit_vl53l4cd.VL53L4CD(i2c)
vl53.inter_measurement = 0 # the delay between measurements - slightly larger than timing_budget
vl53.timing_budget = 100 # how long in ms to take a measurement - 10 - 200 ms
vl53.start_ranging()

print("VL53L4CD Simple Test.")
print("--------------------")
model_id, module_type = vl53.model_info
print(f"Model ID: 0x{model_id:0X}")
print(f"Module Type: 0x{module_type:0X}")
print(f"Timing Budget: {vl53.timing_budget}")
print(f"Inter-Measurement: {vl53.inter_measurement}")
print("--------------------")


while True:
    if vl53.data_ready:
        print(f"Distance: {vl53.distance} cm")
        vl53.clear_interrupt()
