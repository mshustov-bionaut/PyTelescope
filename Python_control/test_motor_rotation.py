import hw_control_2 as hw2
import time

arduino_board = hw2.ArduinoMegaBoard('COM3')
driver = hw2.CoilPololuArduinoController(arduino_board=arduino_board, pwm_pins=[9,9,9,9,9,9,9,9],
                                         direction_pins=[8,8,8,8,8,8,8,8])
driver.new_coil_state_single(coil_num=1, state=0)
time.sleep(1)
for i in range(30):
    driver.new_coil_state_single(coil_num=1, state=0.2)
    time.sleep(0.1)
    driver.new_coil_state_single(coil_num=1, state=0)
    time.sleep(0.1)
#time.sleep(3)
driver.close_connection()