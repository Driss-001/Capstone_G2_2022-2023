
import time,os,urllib3,board,busio,pwmio,threading
from adafruit_extended_bus import ExtendedI2C as I2C
import math as mt
import numpy as np
import matplotlib.pyplot as plt  
from pathlib2 import Path
from astropy.io import fits 
from scipy.integrate import simpson as sps   
import adafruit_mcp4725 as MCP
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import datetime as dt
import pickle as pkl

#constants
path_length = 5 # in meters
DAC_res =  12
ADC_res =  16
VMAX = 3.3 #max volt-pin output voltage

#constant functions
dac_raw = lambda volt: volt/VMAX*2**DAC_res
#create I2C buses

DAC_ADDRESS = 0x62
ADC_ADDRESS = 0x48
i2c1 = I2C(1)
i2c2 = I2C(6)

dac = MCP.MCP4725(address=DAC_ADDRESS,i2c=i2c1)
adc = ADS.ADS1115(address=ADC_ADDRESS,i2c = i2c2,data_rate=860)
#get_dacvolt  = lambda x: x*VMAX/2**ADC_res
#get_adcvolt = lambda x: x*VMAX/2**DAC_res
Gain = ADC_res/16 #gain of 1

#pwm init
pwm = pwmio.PWMOut(board.D13,frequency = 1e3,variable_frequency = True)
duty_cycle = lambda x: (2**16-1)/100*x #Duty cycle is 16bits, return duty cycle percentage

#Triangle ramp signal/correlations wanted

CORR_NUM = 2
CURRENT_WD  = os.getcwd()

class PI_Controller:
    """Raspberry pi code to activate the (S)LED, heat the u-chip and collect signal from the photodiode
       input: test #(0,1) currently supported, continuous 0, test duration: wanted test duration in sec
                dpi for graphical output, sampling_f sampling frequency 
    
    
    """
    
    #Initialisation
    def __init__(self,test = 0,continuous = 0,test_duration = 5,prototype = 0,dpi = 300,sampling_f = 200, autorun = 0,save_dir = CURRENT_WD+'/PI_Code/data', c_noise = False ) -> None:

        self.test_status = test
        self.save_dir = save_dir
        self.m_run = c_noise
        self._init_arrays()
        self.continuous = continuous #if reading loops itself
        self.test_duration = test_duration           
        self.active = True
        self.dpi = dpi
        self.Reset_time()
        self.counter = 0
        self.sampling_f = sampling_f
        if self.sampling_f >=860:
            self.sampling_f = 860
        self.num_samples = self.sampling_f*self.test_duration
        
        if bool(autorun): #Determine autorun
            if not self.m_run:
                self.Run() #switch on
            else:
                self.M_Rrun()

        
            
    
    #Streamlined ADC/DAC function to manipulate in/out voltages

    def set_DAC(self,voltage): #set dac ouput voltage
        if voltage > VMAX:
            voltage = VMAX
        dac_volt = round(voltage/VMAX*(2**DAC_res-1))    
          
        try:
            dac._write_fast_mode(dac_volt)#attempt write fast-mode
        except:
            print(f"DAC voltage output must be between 0 and {VMAX} V")  
    
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

    def Run(self) -> None: #activate switch function , auto-start
    
        #    #print("flag")
        self.t_start = self._now()
        self.th1 = threading.Thread(target = self._HW_start)
        self.th1.daemon = True
        self.th1.start()
        self.counter = 0
        self.perc_1=0
        
        
        while self.counter< self.num_samples: #finish signal by points collected
            self._freq_AutoCal()

         #finish test
        n = self.num_samples 
        self._figure_pkl(n)
        if not self.m_run:
            self._save_figs(n)
        print(f"{self._now(self.t_start)} secs have passed, test finished!")
        self.Switch()


    def M_Run(self):
        for i in range(0,10):
            self.Run()

        self._Gauss_Cancel()           
    


    def Switch(self) -> None: #toggle activate deactivate
        
        self.active = not self.active
        print(f"active bool is now {self.active}")

    def De_Activate(self):
        self.active = False

    def Reset_time(self):
        self.start_time = time.time()

    """Private functions in charge of hardware signal sending and collection"""

    """Array Init private function"""
    def _init_arrays(self) -> None:
        self.current_date = dt.datetime.now().strftime('%Y-%m-%d-%H-%M')
        self.t_period =  self.test_duration/CORR_NUM
        match self.test_status:
            case 0: #PI simple signals
                self.dac_order = []
                self.dac_order_time =[]
                self.adc_output = []
                self.adc_output_time = []
                self.pwm_output = []
                self._test = np.bool_([0,0]) #voltage test
                ADS.mode = 0 # put ads in continuous mode for reading speed  
            case 1: #Proto 1
                self.adc_output = []
                self.adc_output_time = []                
                self._test = np.bool_([1,0]) 
                self.time = time.time()
                ADS.mode = 0
            case 2: #Proto 2
                self._test = np.bool_([0,1])  #signal test
                self.time = time.time()
         
            case 3: #Real case
                self._test = np.bool_([1,1]) 

    #function governing LED activation, DAC
    def __LED(self) -> None:
        #no proto or proto 1
        if not self._test[1] and not self._test[0]:
            
            #print(f"counter aues is: {self.counter}")
            dac_order =self._triangle(self.t_period,VMAX)     
            self.set_DAC(dac_order)
            self.counter += 1
            self.dac_order.append(dac_order)
            self.dac_order_time.append(self._now())
            return
        if not self._test[1] and  self._test[0]:
            pass
        pass

    #function driving ramp signal to heat the u-chip, DAC
    def __Temp(self) -> None:
        print("Applying voltage to the Chip...")
        pwm.duty_cycle = duty_cycle(99) #99% Duty cycle for DC voltage


    #function reading the photodiode output , ADC    
    def __PhotoDRead(self) -> None:
   
        if not self._test[1] and not self._test[0]: #case signals
            self.adc_output.append(self.ADC_volt(0))
            self.pwm_output.append(self.ADC_volt(1))
            self.adc_output_time.append(self._now())
            return 
        if not self._test[1] and  self._test[0]:
            self.adc_output.append(self.ADC_volt(2))
            self.adc_output_time.append(self._now())
        pass    

        self.counter = len(self.adc_output) #recorded DAC output length

    def _now(self,t=0): #current time for performance tracking
        return round(time.time()-self.start_time-t,3) 
        
    #function storing initial T°=0 absorption throughport spectrum to compare with T°>0 spectrums 
    def _Calibrate(self) -> None:
        if not self._test[1] and not self._test[0]:
            pass
        else:
            pass
        pass

    def _freq_AutoCal(self) -> None:
        new_freq - self.sampling_f
        self.perc_2 = self.counter/self.num_samples*100
        if abs(self.perc_2-self.perc_1)>1:
            new_freq = self.counter/self._now(self.t_start)
            print(self.perc_2,self.counter/self._now(self.t_start))
            self.perc_1 = self.perc_2
        if abs(new_freq-self.sampling_f)/self.sampling_f <1e-2:
            self.Switch() #Turn off parallel thread
            print(f"Real frequncy is {new_freq} HZ instead of {self.sampling_f} Hz, recalibrating...")
            self.sampling_f = new_freq
            self.test_duration = self.num_samples/self.sampling_f
            print(f"New Test duration is {self.test_duration}")
            self._init_arrays()
            self.Run()
            return



    def _HW_start(self) -> None: #activate all hardware signals and readers
        self._Calibrate()
        self.__Temp()
        while self.active:    #main parallel thread loop
            self.__LED()
            self.__PhotoDRead()
            time.sleep(1/self.sampling_f-1/860) #adjust to sampling frequency + 860Hz round for DAC conversion
        if not self.active: #Thread autocloses upon deactivation
            self.th1.join()     

    def _progbar(self):
        bar_length = 100
        pass

    #triangular signal function for PWM-DC system    
    def _triangle(self,period,peak) -> float:
        now_time = self._now()
        now_frac = now_time//(period/2)
        now_mod = now_frac%2
        if now_mod == 0:
            return peak*2/period*(now_time-period/2*now_frac)
        else:
            return peak*(1-2/period*(now_time-period/2*now_frac)) 

    """Private functions for data handling"""
    
    def _figure_pkl(self,n):
        if not self._test[1] and not self._test[0]:
            self._topkl(self.adc_output_time[0:n],self.adc_output[0:n],self.pwm_output[0:n])
        if not self._test[1] and  self._test[0]:
            self._topkl(self.adc_output_time[0:n],self.adc_output[0:n])         

    def _save_figs(self,n) -> None:
        if not self._test[1] and not self._test[0]:
            plt.plot(self.dac_order_time[0:n],self.dac_order[0:n],c ="black")
            plt.scatter(self.adc_output_time[0:n],self.adc_output[0:n],c="red")
            plt.scatter(self.adc_output_time[0:n],self.pwm_output[0:n],c="green")
            plt.legend(["DAC py-order","ADC chan0 output (DAC)","ADC chan 1 output(PWM)"])
            plt.ylabel('Voltage (V)')
            plt.xlabel('time(s)')
            plt.title(f"Rpi4 IO DAC/ADC Test0,sampling @ {self.sampling_f}Hz,date:{self.current_date}")
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
            plt.plot(self.adc_output_time[0:n],self.adc_output[0:n],c="blue")
            plt.legend(["ADC chan1 output  (Photodiode)"])    
            plt.ylabel('Voltage (V)')
            plt.xlabel('time(s)')
            plt.title(f"Rpi4 IO DAC/ADC Test1,sampling @ {self.sampling_f}Hz,{self.num_samples} points,date:{self.current_date}")
            plt.savefig(f"Test1_ADC_output_{int(round(self.sampling_f))}_{self.num_samples}_{self.current_date}.png",dpi = self.dpi)
            plt.clf()   

        print("figure Saved!")  
        

    def _Gauss_Cancel(self):
        pass     

    """" Save & load Graphs pkl  """
    def _topkl(self,*args, j = 0) -> None: 
        dump_pkl = []
        file_name = f"test{self.test_status}_{self.current_date}_num{j}.pkl"
        for i in args:
            dump_pkl += [i]
        with open(self.save_dir+file_name, 'wb') as f:
            pkl.dump(dump_pkl,f)
        print(f"{file_name} saved!")

    def _frompkl(self, current_date,j =0) -> list:

        file_name = f"test{self.test_status}_{current_date}_num{j}.pkl"    
        with open(self.save_dir+file_name, 'rb') as f:
            pkl_fetch = pkl.load(f)
        return pkl_fetch    

if __name__ == '__main__':
    #test0 = PI_Controller(test_duration=20/60)
    test1 = PI_Controller(test =1,test_duration =1,sampling_f=860,autorun=1)
