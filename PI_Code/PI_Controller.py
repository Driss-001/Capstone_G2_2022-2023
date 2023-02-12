
import time,os,urllib3,board,busio,pwmio,threading
from adafruit_extended_bus import ExtendedI2C as I2C
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
from adafruit_ads1x15.analog_in import AnalogIn
import datetime as dt

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
#get_dacvolt  = lambda x: x*V_Max/2**ADC_res
#get_adcvolt = lambda x: x*V_Max/2**DAC_res
Gain = ADC_res/16 #gain of 1

#pwm init
pwm = pwmio.PWMOut(board.D13,frequency = 200e3,variable_frequency = True)
duty_cycle = lambda x: 2**16/100*x #Duty cycle is 16bits, return duty cycle percentage

class PI_Controller:
    """Raspberry pi code to activate the (S)LED, heat the u-chip and collect signal from the photodiode"""
    
    #Initialisation
    def __init__(self,test = 0,continuous = 0,test_duration = 5,prototype = 0,dpi = 300,sampling_f = 1000) -> None:
        self.test_status = test 
        self.continuous = continuous #if reading loops itself
        self.active = True
        self.dpi = dpi
        self.Reset_time()
        self.counter = 0
        self.sampling_f = sampling_f
        
        match self.test_status:
            case 0: #PI simple signals
                self.dac_order = []
                self.dac_order_time =[]
                self.adc_output = []
                self.adc_output_time = []
                self.pwm_output = []
                self._test = np.bool_([0,0]) #voltage test
                ADS.mode = 0 # put ads in continuous mode for reading speed   
                self.test_duration = test_duration*60  
                self.Run() #switch on
            case 1: #Proto 1
                self.adc_output = []
                self.adc_output_time = []                
                self._test = np.bool_([1,0]) 
                self.time = time.time()
                self.test_duration = test_duration*60 #duration in sec
                self.Run() #switch on
            case 2: #Proto 2
                self._test = np.bool_([0,1])  #signal test
                self.time = time.time()
                self.test_duration = test_duration*60 #duration in sec   
            case 3: #Real case
                self._test = np.bool_([1,1]) 
        
            

    
    #Streamlined ADC/DAC function to manipulate in/out voltages

    def set_DAC(self,voltage): #set dac ouput voltage
        if voltage > V_Max:
            voltage = V_Max
        try:
            dac.raw_value = round(voltage/V_Max*2**DAC_res)        
        except:
            print(f"DAC voltage output must be between 0 and {V_Max} V")  
    
    def ADC_volt(self,channel = 0)->float: #return ADC voltage from selected channel
        match channel:
            case 0:
                chan0 = AnalogIn(adc,ADS.P0)
                return chan0.voltage
            case 1:
                chan1 = AnalogIn(adc,ADS.P1)
                return chan1.voltage                  
            case 2:
                chan2 = AnalogIn(adc,ADS.P2)
                return chan2.voltage                 
            case 3:
                chan3 = AnalogIn(adc,ADS.P3)
                return chan3.voltage 

    def Run(self): #activate switch function , auto-start
    
        #    #print("flag")
        self.th1 = threading.Thread(target = self._HW_start)
        self.th1.daemon = True
        self.th1.start()
        self.counter = 0

        while self._now()<= self.test_duration: #wait for test to finish
            pass

         #finish test
        self._save_figs()
        print(f"{self.test_duration} secs have passed, test finished!")
        self.Switch()
        if not self.active:
            self.th1.join()        
    


    def Switch(self): #toggle activate deactivate
        
        self.active = not self.active
        print(f"active bool is now {self.active}")

    def De_Activate(self):
        self.active = False

    def Reset_time(self):
        self.start_time = time.time()

    """Private functions in charge of hardware signal sending and collection"""
    #function governing LED activation, DAC
    def __LED(self):
        print(f"LED activating at {self._now()}s...")
        #no proto or proto 1
        if not self._test[1] and not self._test[0]:
            if self.counter == 100:
                self.counter = 0
            #print(f"counter aues is: {self.counter}")
            dac_order = abs(mt.sin(2*mt.pi*self._now()/(60*self.test_duration*1.1)))*V_Max    
            self.set_DAC(dac_order)
            self.counter += 1
            self.dac_order.append(dac_order)
            self.dac_order_time.append(self._now())
            return
        if not self._test[1] and  self._test[0]:
            pass
        pass

    #function producing ramp signal to heat the u-chip, DAC
    def __Temp(self):
        print("Applying voltage to the Chip...")
        pwm.duty_cycle = duty_cycle(99) #99% Duty cycle for DC voltage


    #function reading the photodiode output , ADC    
    def __PhotoDRead(self):
        print(f"Photodiode reading start t= {self._now()}s...")
   
        if not self._test[1] and not self._test[0]: #case signals
            self.adc_output.append(self.ADC_volt(0))
            self.pwm_output.append(self.ADC_volt(1))
            self.adc_output_time.append(self._now())
            return 
        if not self._test[1] and  self._test[0]:
            self.adc_output.append(self.ADC_volt(1))
            self.adc_output_time.append(self._now())
        pass    

    def _now(self): #current time for performance tracking
        return round(time.time()-self.start_time,3) 
        
    #function storing initial T°=0 absorption throughport spectrum to compare with T°>0 spectrums 
    def _Calibrate(self):
        if not self._test[1] and not self._test[0]:
            pass
        else:
            pass
        pass

    def _HW_start(self): #activate all hardware signals and readers
        self._Calibrate()
        self.__Temp()
        while self.active:    #main parallel thread loop
            self.__LED()
            self.__PhotoDRead()
            time.sleep(1/self.sampling_f) #100us intervall  

    def _progbar(self):
        bar_length = 100
        pass


    """Private functions for data handling"""
    def _save_figs(self):
        if not self._test[1] and not self._test[0]:
            plt.plot(self.dac_order_time,self.dac_order,c ="black")
            plt.scatter(self.adc_output_time,self.adc_output,c="red")
            plt.scatter(self.adc_output_time,self.pwm_output,c="green")
            plt.legend(["DAC py-order","ADC chan0 output (DAC)","ADC chan 1 output(PWM)"])
            plt.ylabel('Voltage (V)')
            plt.xlabel('time(s)')
            plt.title(f"Rpi4 IO DAC/ADC Test0,sampling @ {self.sampling_f}Hz,date:{dt.datetime.now()}")
            plt.savefig("Test0_ADC_DAC_output",dpi = self.dpi)   

            plt.clf()

            #plt.plot(np.array(range(1,len(self.dac_order_time)+1)),self.dac_order_time,c="blue")
            #plt.plot(np.array(range(1,len(self.adc_output_time)+1)),self.adc_output_time,c="red")
            #plt.legend(["DAC order time","ADC output time"])
            #plt.ylabel("time(s)")
            #plt.title("Rpi4 IO Latency Test0")
            #plt.savefig("Test0_Latency_output",dpi = self.dpi)
            return
        if not self._test[1] and  self._test[0]:  
            plt.plot(self.adc_output_time,self.adc_output,c="blue")
            plt.legend(["ADC chan1 output  (Photodiode)"])    
            plt.ylabel('Voltage (V)')
            plt.xlabel('time(s)')
            plt.title(f"Rpi4 IO DAC/ADC Test1,sampling @ {self.sampling_f/1000}kHz,date:{dt.datetime.now().strftime('%Y-%m-%d-%H-%M')}")
            plt.savefig(f"Test1_ADC_output_{int(round(self.sampling_f/1000))}_{dt.datetime.now().strftime('%Y-%m-%d-%H-%M')}.png",dpi = self.dpi)
            plt.clf()   

        print("figure Saved!")   

if __name__ == '__main__':
    #test0 = PI_Controller(test_duration=20/60)
    test1 = PI_Controller(test_duration=100/60,test =1,sampling_f = 1e5)
