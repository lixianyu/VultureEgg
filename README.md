VultureEgg
==========
Based on TI BLE-CC254x-1.4.1.43908 stack.
http://www.geek-workshop.com/thread-10164-1-1.html

VultureEgg project based on 'SensorTag' project.

==========

Protocol between phone and VultureEgg:
AB0101 -- Start LM75A and measurement period is the default value 20s.
AB01010A -- Start LM75A and measurement period is 10s
AB0100 -- Stop LM75A

AB0201 -- Start humidity(SHT21) and measurement period is the default value 70s.
AB02010F -- Start humidity(SHT21) and measurement period is 15s
AB0200 -- Stop humidity(SHT21)

AB0301 -- Start MPU6050 and measurement period is the default value 2s.
AB030103 -- Start MPU6050 and measurement period is 3s
AB0300 -- Stop MPU6050
