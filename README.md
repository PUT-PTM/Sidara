# Sidara

## Open source academic project for Microprocessors Lab

#### Overview
Sidara is a project of basic acoustic guitar simulator, that uses optic sensor, DAC, and card reader to produce basic guitar chord sounds.
Built on STM32F407G microprocessor and Arduino.

Communication with components:
<ul>
  <li>I2C - optic sensor</li>
  <li>SPI - card reader</li>
  <li>USART - Arduino&STM32F4</li>
</ul>

<br/>

Guitar chords on buttons: (from guitar head)
<ol>
  <li>High D</li>
  <li>Low D</li>
  <li>H</li>
  <li>C</li>
  <li>F#</li>
  <li>G</li>
  <li>A</li>
</ol>

<br/>
<br/>

#### Working Scheme
When you hold one of chord buttons and make gesture over optic sensor the sensor generates signal to Arduino which processess gesture and determines which direction it was. Then char with direction (L / R / U / D) goes by USART to STM which determines chord (by pressed button) and its volume (on ADC knob). Then STM generates DAC signal on audio output.
<br/>
<br/>

#### Software
You will need two IDE's to flash everything. First CooCox IDE (we used 1.7.8 version) to compile and flash program to STM32F4. Next You will need Arduino IDE (we used 1.8.3 version).
<br/>
<br/>

#### Physical connections
Optic sensor connection: (To Arduino)<br/>
<ul>
  <li>A5 -> SCL</li>
  <li>A4 -> SDA</li>
  <li>GND -> GND</li>
  <li>3.3V  -> VCC</li>
  <li>2 -> INT</li>
</ul>

USART Connection (STM32 -> Arduino)<br/>
<ul>
  <li>RxD(0) -> PC10</li>
  <li>TxD(1) -> PC11</li>
</ul>

SD Card Module connection: (To STM32F4)<br/>
<ul>
  <li>GND -> GND</li>
  <li>3V -> 3.3V</li>
  <li>PB11 -> CS</li>
  <li>3.3V  -> VCC</li>
  <li>PB15 -> MOSI</li>
  <li>PB13 -> SCK</li>
  <li>PB14 -> MISO</li>
  <li>GND -> GND</li>
</ul>

ADC connection: (To STM32F4)<br/>
<ul>
  <li>GND -> GND</li>
  <li>3V -> VCC</li>
  <li>PA1 -> ADC IN 1</li>
</ul>

<br/>
<br/>

#### Licence
Project is under Apache Software License 2.0
<br/>
Idea, implementation and copyrights to:
<ul>
  <li>Michał Sibera (Arebis)</li>
  <li>Dawid Wojciechowski (TheSensej)</li>
</ul>

#### Contact: <a href="mailto:michal.sibera@windowslive.com">Arebis</a> or <a href="mailto:wojciechdavid@gmail.com">TheSensej</a>
