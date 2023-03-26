# Capstone_G2_2022-2023: Developing a Near Infrared On Chip CO2 Sensor

This software project goal is to use the strong waveguide properties of a silicon ring resonator built on a microchip to create very acurate absorption spectra to be analysed by a raspberry pi


## Step 1 raspberry Pi + Desktop simulation
  
Control System centered around Rpi-4  
![](https://assets.raspberrypi.com/static/raspberry-pi-4-labelled-f5e5dcdf6a34223235f83261fa42d1e8.png) 
## Step 2: Prototype 1

### Current Design Diagram
![](PostVisit_software.drawio.png)

### Test0 Results

Voltage Outputs<br>
![ADC/DAC Graph](https://github.com/Driss-001/Capstone_G2_2022-2023/blob/main/Test0_ADC_DAC_output.png?raw=true)<br>

Latency<br>
![Latency](https://github.com/Driss-001/Capstone_G2_2022-2023/blob/main/Test0_Latency_output.png?raw=true)<br>

### Test1 Results

No ramp<br>

![No gas](https://github.com/Driss-001/Capstone_G2_2022-2023/blob/main/Test1_ADC_output_100_2023-02-12-13-20.png?raw=true)<br>

![T°=0 graph](https://github.com/Driss-001/Capstone_G2_2022-2023/blob/main/Test1_ADC_output_100_2023-02-10-22-17-57.png?raw=true)<br>

Ramp Temperature applied<br>
![T° ramp graph](https://github.com/Driss-001/Capstone_G2_2022-2023/blob/main/Test1_ADC_output_C100_10smpls_2023-03-25-19-01.png?raw=true)<br>