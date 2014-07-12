#数据协议
```
      <stream> ::= <package><CRLF><package><CRLF>...
     <package> ::= <package type>;<package body>
<package type> ::= "Egg" | "Weather"

<package body> ::= <蓝牙UUID>;<unix时间戳>;<加速度数据>;<陀螺仪数据>;<温度数据>;<湿度数据> | <unix时间戳>;<温度数据>;<湿度数据>;<光照数据>

    <蓝牙UUID> ::= <CC2540 UUID,hexadecimal>
  <unix时间戳> ::= <UNIX timestamp,decimal,UTC+8>
  <加速度数据> ::= <x-axis-a>,<y-axis-a>,<z-axis-a>
            <x-axis-a>,<y-axis-a>,<z-axis-a> = <axis-a>
            <axis-a> ::= <float,单位g>
  <陀螺仪数据> ::= <x-axis-z>,<y-axis-z>,<z-axis-z>
            <x-axis-z>,<y-axis-z>,<z-axis-z> = <axis-z>
            <axis-z> ::= <float,单位自定>
    <温度数据> ::= <temp01>,<temp02>,...,<temp14>
	<temp01>,<temp02>,...,<temp14> ::= <float,单位℃>
    <湿度数据> ::= <float,单位%RH>
    <光照数据> ::= <float,单位lux>
```

#样例数据
##蛋内数据
```
"Egg,EEF0,1405094400,0.6,0.2,-0.7,12.5,-7.8,13.9,35.2,33.1,34.2,28.9,29.9,32.9,35.8,33.8,34.1,28.7,29.3,32.8,-5.5,18.2,65.5,End"
样例数据：
UUID：EEF0；时间戳：1405094400(2014-7-12 0:00:00 UTC+8)；三轴加速度：0.6g,0.2g,-0.7g；三轴陀螺仪：12.5,-7.58,13.9；
十四路温度：35.2℃,33.1℃,34.2℃,28.9℃,29.9℃,32.9℃,35.8℃,33.8℃,34.1℃,28.7℃,29.3℃,32.8℃,-5.5℃,18.2℃；湿度：65.5%RH。
```
##气象站数据
```
样例数据略
```

#备注
```
全部浮点数据保留符号，保留一位小数
```
