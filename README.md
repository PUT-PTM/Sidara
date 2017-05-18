# Sidara

## Open source academic project for Microprocessors Lab

Sidara is a project of basic acoustic guitar simulator, that uses optic sensor, DAC, and card reader to produce basic guitar chord sounds.
Built on STM32F407G microprocessor and Arduino.

Communication with components:
<ul>
  <li>I2C - optic sensor</li>
  <li>SPI - card reader</li>
</ul>

Optic sensor connection: (To Arduino)<br/>
&emsp;&emsp;A5 -> SCL<br/>
&emsp;&emsp;A4 -> SDA<br/>
&emsp;&emsp;GND -> GND<br/>
&emsp;&emsp;3.3V  -> VCC<br/>
&emsp;&emsp;2 -> INT<br/>

USART Connection (Arduino -> STM32F4)
&emsp;&emsp;RxD(0) -> PC10<br/>
&emsp;&emsp;TxD(1) -> PC11<br/>

SD Card Module connection: (To STM32F4)<br/>
&emsp;&emsp;GND -> GND<br/>
&emsp;&emsp;3V -> 3V3<br/>
&emsp;&emsp;PB11 -> CS<br/>
&emsp;&emsp;PB15 -> MOSI<br/>
&emsp;&emsp;PB13 -> SCK<br/>
&emsp;&emsp;PB14 -> MISO<br/>
&emsp;&emsp;GND -> GND<br/>

Idea, implementation and copyrights to:
<ul>
  <li>Michał Sibera (Arebis)</li>
  <li>Dawid Wojciechowski (TheSensej)</li>
</ul>

#### Contact: <a href="mailto:michal.sibera@windowslive.com">Arebis</a> or <a href="mailto:wojciechdavid@gmail.com">TheSensej</a>
