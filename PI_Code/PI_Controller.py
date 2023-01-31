
import time,os,urllib3,board,busio,pwmio,threading
from adafruit_extended_bus import ExtendedI2C as I2C
import RPI.GPIO as gpio
import math as mt
import numpy as np
from scipy.interpolate import InterpolatedUnivariateSpline
import matplotlib.pyplot as plt  
from pathlib2 import Path
from astropy.io import fits 
import astropy.modeling.functional_models as astromodels
from scipy.integrate import simpson as sps   
import adafruit_mcp4725 as MCP
import adafruit_ads1x15.ads1115 as ADS

#constants
path_length = 5 # in meters
DAC_res =  12
ADC_res =  16
V_Max = 3.3 #max gpio output voltage

#constant functions
dac_raw = lambda volt: volt/V_Max*2**DAC_res
#create I2C buses

dac_address = 0x62
adc_address = 0x48
i2c1 = I2C(1)
i2c2 = I2C(6)

dac = MCP.MCP4725(address=dac_address,i2c=i2c1)
adc = ADS.ADS1115(address=adc_address,i2c = i2c2)
get_dacvolt  = lambda x: x*V_Max/2**ADC_res
get_adcvolt = lambda x: x*V_Max/2**DAC_res
Gain = 1

#pwm init
pwm = pwmio(board.D13,frequency = 10e3)
duty_cycle = lambda x: 2**16/100*x #Duty cycle is 16bits, return duty cycle percentage

class PI_Controller:
    """Raspberry pi code to activate the (S)LED, heat the u-chip and collect signal from the photodiode"""
    
    #Initialisation
    def __init__(self,test = 0,continuous = 0,test_duration = 5,prototype = 0) -> None:
        self.test_status = test 
        self.continuous = continuous #if reading loops itself
        self.active = False
        
        
        match self.test_status:
            case 0: #PI simple signals
                self._test = np.bool_([0,0]) #voltage test
                self.start_time = time.time()
                self.test_duration = test_duration  
                self.Run() #switch on
            case 1: #Proto 1
                self._test = np.bool_([0,1]) 
                self.time = time.time()
                self.test_duration = test_duration*60 #duration in sec
            case 2: #Proto 2
                self._test = np.bool_([1,0])  #signal test
                self.time = time.time()
                self.test_duration = test_duration*60 #duration in sec   
            case 3: #Real case
                self._test = np.bool_([1,1]) 
        
            

    """Private functions in charge of hardware signal sending and collection"""
    #function governing LED activation, DAC
    def __LED(self):
        print("LED activating...")
        #no proto or proto 1
        if not self._test[1]:
            if self.counter == 2**DAC_res:
                self.counter = 0
            dac.raw_value = self.counter
            self.counter += 2**DAC_res/16
        else:
            pass
        pass

    #function producing ramp signal to heat the u-chip, DAC
    def __Temp(self):
        print("Applying voltage to the Chip...")
        pwm.duty_cycle = duty_cycle(99) #99% Duty cycle for DC voltage


    #function reading the photodiode output , ADC    
    def __PhotoDRead(self):
        print("Photodiode reading start...")
        if not self._test[0]: #case signals
            values = [0]*4
            for i in range(4):
                 # Read the specified ADC channel using the previously set gain value.
                values[i] = adc.read_adc(i, gain=Gain) 
            values = np.vectorize(get_dacvolt)(np.array(values))
            print(values+" V")

        else:
            pass
        pass    

    #function storing initial T°=0 absorption throughport spectrum to compare with T°>0 spectrums 
    def _Calibrate(self):
        if not self._test[1]:
            pass
        else:
            pass
        pass

    def _HW_start(self): #activate all hardware signals and readers
        self._Calibrate()
        while self.active:    #main parallel thread loop
            self.__LED()
            self.__Temp()
            self.__PhotoDRead()
            time.sleep(0.1)
    

    def Run(self): #activate switch function , auto-start
        if self.active:
             self.th1 = threading.Thread(target = self._HW_start)
             self.th1.daemon = True
        if (time.time()-self.start_time) == 60*self.test_duration: #finish test
            print(f"{self.test_duration} min have passed, test finished!")
            self.De_Activate()


    def Switch(self): #toggle activate deactivate
        
        self.active = not self.active

    def De_Activate(self):
        self.active = False

if __name__ == '__main__':
    test0 = PI_Controller(test_duration=3)
