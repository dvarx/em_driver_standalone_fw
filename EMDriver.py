import serial
import time

class EMDriver:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, waitime=1e-3):
        self.ser = serial.Serial(port, baudrate)
        self.waittime = waitime
        if not self.ser.is_open:
            raise serial.SerialException(f"Could not open port {port}")
        
    def counter(self, des_frequency):
        return int(32e6/(2*des_frequency))

    def start(self):
        self.ser.write(b"STRT\n")
        time.sleep(self.waittime)

    def stop(self):
        self.ser.write(b"STOP\n")
        time.sleep(self.waittime)

    def set_frequency(self, frequency_mHz):
        des_counter=self.counter(frequency_mHz)
        self.ser.write(b"CNTR\0"+bytes("%08d"%des_counter,"ascii")+b"\n")
        time.sleep(self.waittime)

    def close(self):
        self.ser.close()

if __name__=="__main__":
    driver=EMDriver(port="/dev/ttyUSB0", baudrate=115200, waitime=1e-3)
    driver.stop()

    #set the frequency to 1kHz and run for 5s
    driver.set_frequency(1000)
    driver.start()
    time.sleep(5)
    driver.stop()

    #set the frequencu to 1.12kHz and run for 5s
    driver.set_frequency(1.12e3)
    driver.start()
    time.sleep(5)
    driver.stop()