#---------来自蛋内的数据---------
```
数据包起始："Egg,"
数据包内容：<蓝牙UUID>,<unix时间戳>,<加速度数据>,<陀螺仪数据>,<温度数据>,<湿度数据>
数据包结束：",End"

<蓝牙UUID>::=<CC2540 UUID,hex format,string sending>
<unix时间戳>::=<UNIX timestamp,dec format,string sending,UTC+8>
<加速度数据>::=<x-axis>,<y-axis>,<z-axis>
	<x-axis>,<y-axis>,<z-axis>::<float format,string sending,单位g>
<陀螺仪数据>::=<x-axis>,<y-axis>,<z-axis>
	<x-axis>,<y-axis>,<z-axis>::<float format,string sending,单位自定>
<温度数据>::=<temp01>,<temp02>,...,<temp14>
	<temp01>,<temp02>,...,<temp14>::<float format,string sending,单位℃>
<湿度数据>::=<float format,string sending,单位%RH>

样例数据包：
"Egg,EEF0,1405094400,0.6,0.2,-0.7,12.5,-7.8,13.9,35.2,33.1,34.2,28.9,29.9,32.9,35.8,33.8,34.1,28.7,29.3,32.8,-5.5,18.2,65.5,End"
样例数据：
UUID：EEF0；时间戳：1405094400(2014-7-12 0:00:00 UTC+8)；三轴加速度：0.6g,0.2g,-0.7g；三轴陀螺仪：12.5,-7.58,13.9；
十四路温度：35.2℃,33.1℃,34.2℃,28.9℃,29.9℃,32.9℃,35.8℃,33.8℃,34.1℃,28.7℃,29.3℃,32.8℃,-5.5℃,18.2℃；湿度：65.5%RH。
```
#---------来自气象站的数据------
```
数据包起始："Weather,"
数据包内容：<unix时间戳>,<温度数据>,<湿度数据>,<光照数据>
数据包结束：",End"

<unix时间戳>::=<UNIX timestamp,dec format,string sending>
<温度数据>::=<float format,string sending,单位℃>
<湿度数据>::=<float format,string sending,单位%RH>
<湿度数据>::=<float format,string sending,单位lux>

样例数据略
```
#------------------------------
全部浮点数据保留符号，保留一位小数

