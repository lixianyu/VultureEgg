/*
Copyright (c) 2007, Jim Studt  (original old version - many contributors since)

The latest version of this library may be found at:
  http://www.pjrc.com/teensy/td_libs_OneWire.html

Version 2.1:
  Arduino 1.0 compatibility, Paul Stoffregen
  Improve temperature example, Paul Stoffregen
  DS250x_PROM example, Guillermo Lovato
  PIC32 (chipKit) compatibility, Jason Dangel, dangel.jason AT gmail.com
  Improvements from Glenn Trewitt:
  - crc16() now works
  - check_crc16() does all of calculation/checking work.
  - Added read_bytes() and write_bytes(), to reduce tedious loops.
  - Added ds2408 example.
  Delete very old, out-of-date readme file (info is here)

Version 2.0: Modifications by Paul Stoffregen, January 2010:
http://www.pjrc.com/teensy/td_libs_OneWire.html
  Search fix from Robin James
    http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1238032295/27#27
  Use direct optimized I/O in all cases
  Disable interrupts during timing critical sections
    (this solves many random communication errors)
  Disable interrupts during read-modify-write I/O
  Reduce RAM consumption by eliminating unnecessary
    variables and trimming many to 8 bits
  Optimize both crc8 - table version moved to flash

Modified to work with larger numbers of devices - avoids loop.
Tested in Arduino 11 alpha with 12 sensors.
26 Sept 2008 -- Robin James
http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1238032295/27#27

Updated to work with arduino-0008 and to include skip() as of
2007/07/06. --RJL20

Modified to calculate the 8-bit CRC directly, avoiding the need for
the 256-byte lookup table to be loaded in RAM.  Tested in arduino-0010
-- Tom Pollard, Jan 23, 2008

Jim Studt's original library was modified by Josh Larios.

Tom Pollard, pollard@alum.mit.edu, contributed around May 20, 2008

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Much of the code was inspired by Derek Yerger's code, though I don't
think much of that remains.  In any event that was..
    (copyleft) 2006 by Derek Yerger - Free to distribute freely.

The CRC code was excerpted and inspired by the Dallas Semiconductor
sample code bearing this copyright.
//---------------------------------------------------------------------------
// Copyright (C) 2000 Dallas Semiconductor Corporation, All Rights Reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL DALLAS SEMICONDUCTOR BE LIABLE FOR ANY CLAIM, DAMAGES
// OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Except as contained in this notice, the name of Dallas Semiconductor
// shall not be used except as stated in the Dallas Semiconductor
// Branding Policy.
//--------------------------------------------------------------------------
*/

#include "OneWire.h"
//#include <ioCC2540.h>
#include <hal_mcu.h>

/**************************************************
  接口定义，移植此程序只需修改下列宏定义和延时函数
**************************************************/
#define DQ            P0_1             //DS18B20数据IO口
#define DQ_PIN        1                //DS18B20数据IO口
#define DQ_PORT       P0DIR

/**************************************************
  以下定义为DS18B20支持的所有命令
***************************************************/

#define SEARCH_ROM      0xF0              //搜索ROM
#define READ_ROM        0x33              //读ROM
#define MATCH_ROM       0x55              //匹配ROM(挂多个DS18B20时使用)
#define SKIP_ROM        0xCC              //跳过匹配ROM(单个DS18B20时跳过)
#define ALARM_SEARCH    0xEC              //警报搜索

#define CONVERT_T       0x44              //开始转换温度
#define WR_SCRATCHPAD   0x4E              //写内部ram3、4字节
#define RD_SCRATCHPAD   0xBE              //读内部ram中9字节的内容
#define CPY_CCTATCHPAD  0x48              //复制便笺
#define RECALL_EE       0xB8              //未启用
#define RD_PWR_SUPPLY   0xB4              //读电源供应

#define HIGH            1                 //高电平
#define LOW             0                 //低电平

#define CL_DQ()     (DQ = LOW)              //清除数据
#define SET_DQ()    (DQ = HIGH)             //设置数据

#define SET_OUT()    (DQ_PORT |= 0x02);
#define SET_IN()     (DQ_PORT &= 0xFD);
//#define SET_OUT()   DQ_PORT |=  BV(DQ_PIN);  //设置IO方向,out设置IO方向为输出
//#define SET_IN()    DQ_PORT &= ~(BV(DQ_PIN));  //设置IO方向,in设备IO方向为输入

//#define SET_OUT()   DQ_PORT |=  1;  //设置IO方向,out设置IO方向为输出
//#define SET_IN()    DQ_PORT &= ~(1);  //设置IO方向,in设备IO方向为输入

#if ONEWIRE_SEARCH
    // global search state
static unsigned char ROM_NO[8];
static uint8_t LastDiscrepancy = 0;
static uint8_t LastFamilyDiscrepancy = 0;
static uint8_t LastDeviceFlag = FALSE;
#endif

/*
 *    延时函数
 *    输入微妙
 */
static void delay_nus(uint16 timeout)
{
    while (timeout--)
    {
        asm("NOP");
        asm("NOP");
        asm("NOP");
    }
}

/*
OneWire_OneWire(uint8_t pin)
{
	pinMode(pin, INPUT);
	bitmask = PIN_TO_BITMASK(pin);
	baseReg = PIN_TO_BASEREG(pin);
#if ONEWIRE_SEARCH
	reset_search();
#endif
}
*/

// Perform the onewire reset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return a 0;
//
// Returns 1 if a device asserted a presence pulse, 0 otherwise.
//
uint8_t OneWire_reset(void)
{
//	IO_REG_TYPE mask = bitmask;
//	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	uint8_t r;
	uint8_t retries = 125;

	HAL_DISABLE_INTERRUPTS();
	SET_IN();
	HAL_ENABLE_INTERRUPTS();
	// wait until the wire is high... just in case
	do {
		if (--retries == 0) return 0;
		delay_nus(2);
	} while ( !(DQ));

	HAL_DISABLE_INTERRUPTS();
    CL_DQ();
    SET_OUT();
	HAL_ENABLE_INTERRUPTS();
	delay_nus(500);
	HAL_DISABLE_INTERRUPTS();
    SET_IN();
	delay_nus(80);
	r = !(DQ);
	HAL_ENABLE_INTERRUPTS();
	delay_nus(420);
	return r;
}

//
// Write a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
void OneWire_write_bit(uint8_t v)
{
	//IO_REG_TYPE mask=bitmask;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;

	if (v & 1) {
		HAL_DISABLE_INTERRUPTS();
		CL_DQ();
		SET_OUT();	// drive output low
		delay_nus(10);
		SET_DQ();	// drive output high
		HAL_ENABLE_INTERRUPTS();
		delay_nus(55);
	} else {
		HAL_DISABLE_INTERRUPTS();
		CL_DQ();
		SET_OUT();	// drive output low
		delay_nus(65);
		SET_DQ();	// drive output high
		HAL_ENABLE_INTERRUPTS();
		delay_nus(5);
	}
}

//
// Read a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
uint8_t OneWire_read_bit(void)
{
	//IO_REG_TYPE mask=bitmask;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	uint8_t r;

	HAL_DISABLE_INTERRUPTS();
	SET_OUT();
	CL_DQ();
	delay_nus(3);
	SET_IN();	// let pin float, pull up will raise
	delay_nus(10);
	r = DQ;
	HAL_ENABLE_INTERRUPTS();
	delay_nus(53);
	return r;
}

//
// Write a byte. The writing code uses the active drivers to raise the
// pin high, if you need power after the write (e.g. DS18S20 in
// parasite power mode) then set 'power' to 1, otherwise the pin will
// go tri-state at the end of the write to avoid heating in a short or
// other mishap.
//
void OneWire_write(uint8_t v, uint8_t power /* = 0 */) {
    uint8_t bitMask;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	    OneWire_write_bit( (bitMask & v)?1:0);
    }
    if ( !power) {
	    HAL_DISABLE_INTERRUPTS();
	    SET_IN();
	    CL_DQ();
	    HAL_ENABLE_INTERRUPTS();
    }
}

void OneWire_write_bytes(const uint8_t *buf, uint16_t count, bool power /* = 0 */) {
  for (uint16_t i = 0 ; i < count ; i++)
    OneWire_write(buf[i], 0);
  if (!power) {
    HAL_DISABLE_INTERRUPTS();
    SET_IN();
    CL_DQ();
    HAL_ENABLE_INTERRUPTS();
  }
}

//
// Read a byte
//
uint8_t OneWire_read() {
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	if ( OneWire_read_bit()) r |= bitMask;
    }
    return r;
}

void OneWire_read_bytes(uint8_t *buf, uint16_t count) {
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = OneWire_read();
}

//
// Do a ROM select
//
void OneWire_select( uint8_t rom[8])
{
    int i;

    OneWire_write(0x55, 0);           // Choose ROM

    for( i = 0; i < 8; i++) OneWire_write(rom[i], 0);
}

//
// Do a ROM skip
//
void OneWire_skip()
{
    OneWire_write(0xCC, 0);           // Skip ROM
}

void OneWire_depower()
{
	HAL_DISABLE_INTERRUPTS();
	SET_IN();
	HAL_ENABLE_INTERRUPTS();
}

#if ONEWIRE_SEARCH
//
// You need to use this function to start a search again from the beginning.
// You do not need to do it for the first search, though you could.
//
void OneWire_reset_search()
  {
  // reset the search state
  LastDiscrepancy = 0;
  LastDeviceFlag = FALSE;
  LastFamilyDiscrepancy = 0;
  for(int i = 7; ; i--)
    {
    ROM_NO[i] = 0;
    if ( i == 0) break;
    }
  }

//
// Perform a search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// OneWire_address variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.  Use OneWire_reset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
uint8_t OneWire_search(uint8_t *newAddr)
{
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number, search_result;
   uint8_t id_bit, cmp_id_bit;

   unsigned char rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = 0;

   // if the last call was not the last one
   if (!LastDeviceFlag)
   {
      // 1-Wire reset
      if (!OneWire_reset())
      {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = FALSE;
         LastFamilyDiscrepancy = 0;
         return FALSE;
      }

      // issue the search command
      OneWire_write(0xF0, 0);

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = OneWire_read_bit();
         cmp_id_bit = OneWire_read_bit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1))
            break;
         else
         {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit)
               search_direction = id_bit;  // bit write value for search
            else
            {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy)
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               else
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);

               // if 0 was picked then record its position in LastZero
               if (search_direction == 0)
               {
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            OneWire_write_bit(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0)
            {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65))
      {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0)
            LastDeviceFlag = TRUE;

         search_result = TRUE;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0])
   {
      LastDiscrepancy = 0;
      LastDeviceFlag = FALSE;
      LastFamilyDiscrepancy = 0;
      search_result = FALSE;
   }
   for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   return search_result;
  }

#endif

#if ONEWIRE_CRC
// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

#if ONEWIRE_CRC8_TABLE
// This table comes from Dallas sample code where it is freely reusable,
// though Copyright (C) 2000 Dallas Semiconductor Corporation
static uint8_t dscrc_table[] = {
      0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
    157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
     35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
    190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
     70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
    219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
    101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
     17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
     50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
    202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
     87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

//
// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (note: this might better be done without to
// table, it would probably be smaller and certainly fast enough
// compared to all those delayMicrosecond() calls.  But I got
// confused, so I use this table from the examples.)
//
uint8_t OneWire_crc8( uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		//crc = pgm_read_byte(dscrc_table + (crc ^ *addr++));
		crc = *(dscrc_table + (crc ^ *addr++));
	}
	return crc;
}
#else
//
// Compute a Dallas Semiconductor 8 bit CRC directly.
// this is much slower, but much smaller, than the lookup table.
//
uint8_t OneWire_crc8( uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}
#endif

#if ONEWIRE_CRC16
bool OneWire_check_crc16(uint8_t* input, uint16_t len, uint8_t* inverted_crc)
{
    uint16_t crc = ~OneWire_crc16(input, len);
    return (crc & 0xFF) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
}

uint16_t OneWire_crc16(uint8_t* input, uint16_t len)
{
    static const uint8_t oddparity[16] =
        { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };
    uint16_t crc = 0;    // Starting seed is zero.

    for (uint16_t i = 0 ; i < len ; i++) {
      // Even though we're just copying a byte from the input,
      // we'll be doing 16-bit computation with it.
      uint16_t cdata = input[i];
      cdata = (cdata ^ (crc & 0xff)) & 0xff;
      crc >>= 8;

      if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
          crc ^= 0xC001;

      cdata <<= 6;
      crc ^= cdata;
      cdata <<= 1;
      crc ^= cdata;
    }
    return crc;
}
#endif

#endif

// Let's write by myself ba.
void DS18B20_select( uint8_t rom[8])
{
    int i;

    DS18B20_Write(MATCH_ROM, 1);           // Choose ROM

    for( i = 0; i < 8; i++) {
        DS18B20_Write(rom[i], 1);
    }
}
/*
 *    写命令函数
 *    输入参数：  命令（DS18B20.H中定义）
 *    输出参数：  无
 *    返回参数：  无
 *
 */
void DS18B20_Write(unsigned char cmd, uint8 power)
{
    unsigned char i;
    SET_OUT();                  //设置IO为输出，2530->DS18B20
    
    /*每次一位，循环8次*/
    for(i=0; i<8; i++)
    {
        CL_DQ();              //IO为低    
        if( cmd & (1<<i) )    //写数据从低位开始
        {
          SET_DQ();           //IO输入高电平
        }
        else
        {
          CL_DQ();            //IO输出低电平
        }
        delay_nus(60);        //保持15~60us
        SET_DQ();             //IO口拉高
    }
    SET_DQ();                 //IO口拉高
    if ( !power) {
	    SET_IN();
	    CL_DQ();
    }
}


/*
 *    读数据函数
 *    输入参数：  无
 *    输出参数：  无
 *    返回参数：  读取的数据
 *
 */
unsigned char DS18B20_Read(void)
{
    unsigned char rdData;     //读出的数据
    unsigned char i, dat;     //临时变量
    
    rdData = 0;               //读出的数据初始化为0     
    
    /* 每次读一位，读8次 */
    for(i=0; i<8; i++)
    {
        CL_DQ();            //IO拉低
        SET_DQ();           //IO拉高
        SET_IN();           //设置IO方向为输入 DS18B20->CC2540
        dat = DQ;           //读数据,从低位开始
        
        if(dat)
        {
          rdData |= (1<<i); //如果读出的数据位为正
        }
        else
        {
          rdData &= ~(1<<i);//如果读出的数据位为负
        }
        
        delay_nus(70);      //保持60~120us
        SET_OUT();          //设置IO方向为输出 CC2540->DS18B20

    }
    return (rdData);        //返回读出的数据
}

/*
 *    DS18B20初始化/复位函数
 *    输入参数：  无
 *    输出参数：  无
 *    返回参数：  无
 *
 */
uint8 DS18B20_Init(void)
{
    uint8 r = 1;
    SET_OUT();
    SET_DQ();         //IO口拉高
    CL_DQ();          //IO口拉低
    delay_nus(550);   //IO拉低后保持一段时间 480-960us
    SET_DQ();         //释放
    SET_IN();         //IO方向为输入 DS18B20->CC2540
    delay_nus(40);    //释放总线后等待15-60us
    
    /* 等待DQ变低 */
    while(DQ)
    {
        ; 
    }
    r = 0;
    delay_nus(240);   //检测到DQ 变低后，延时60-240us
    SET_OUT();        //设置IO方向为输出 CC2540->DS18B20
    SET_DQ();         //IO拉高
    return r;
}
/*
 *    DS18B20 转换温度函数
 *    输入参数：  无
 *    输出参数：  无
 *    返回参数：  无
 *
 */
void DS18B20_SendConvert(void)
{
    DS18B20_Init();               //复位18B20
    DS18B20_Write(SKIP_ROM, 1);      //发出跳过ROM匹配操作
    DS18B20_Write(CONVERT_T, 1);     //启动温度转换
}
float DS18B20_ReadMain(uint8 *data, uint8 len)
{
    unsigned char tem_h,tem_l;    //温度高位字节及低位字节
    unsigned char a,b;            //临时变量
    unsigned char flag;           //温度正负标记，正为0，负为1
    float ft;
    
    DS18B20_Init();               //DS18B20复位       
    DS18B20_Write(SKIP_ROM, 1);      //跳过ROM匹配
    
    DS18B20_Write(RD_SCRATCHPAD, 1); //写入读9字节命令
    tem_l = DS18B20_Read();       //读温度低位。第一字节
    tem_h = DS18B20_Read();       //读温度高位，第二字节
    //data[1] = tem_l;
    //data[0] = tem_h;

    /* 判断RAM中存储的温度正负 
       并提取出符号位和真实的数据
    */
    if(tem_h & 0x80)
    {
        flag = 1;                 //温度为负
        a = (tem_l>>4);           //取温度低4位原码
        b = (tem_h<<4)& 0xf0;     //取温度高4位原码
        /*补码-原码转换
          负数的补码是在其原码的基础上, 符号位不变, 其余各位取反, 最后+1
        */
        tem_h = ~(a|b) + 1;       //取整数部分数值，不符号位
        
        tem_l = ~(a&0x0f) + 1;    //取小数部分原值，不含符号位
    }
    else
    {
        flag = 0;                 //为正
        a = tem_h<<4;
        a += (tem_l&0xf0)>>4;     //得到整数部分值 
        b = tem_l&0x0f;           //得出小数部分值
        tem_h = a;                //整数部分
        tem_l = b&0xff;           //小数部分
    }

    uint8 sensor_data_value[2];
    unsigned char FRACTION_INDEX[16] = {0, 1, 1, 2, 2, 3, 4, 4, 5, 6, 6, 7, 7, 8, 9, 9 };//小数值查询表
    sensor_data_value[0] = FRACTION_INDEX[tem_l]; //查表得小数值
    sensor_data_value[1] = tem_h| (flag<<7);      //整数部分，包括符号位

    ft = sensor_data_value[1] + ((float)sensor_data_value[0])/10.0;
    data[0] = sensor_data_value[0];
    data[1] = sensor_data_value[1];
        //开始转换
    DS18B20_SendConvert();

    return ft;
}

