/**************************************************************************************************
  Filename:       DS18B20.h
  Revised:        $Date: 2015-06-16 19:34:58 +0800 (Tue, 16 Jun 2015) $
  Revision:       $Revision: 0 $


**************************************************************************************************/

#ifndef DS18B20_H
#define DS18B20_H

#include "hal_types.h"

//ROM function commands
#define SEARCH_ROM      0xF0              //搜索ROM
#define READ_ROM        0x33              //读ROM
#define MATCH_ROM       0x55              //匹配ROM(挂多个DS18B20时使用)
#define SKIP_ROM        0xCC              //跳过匹配ROM(单个DS18B20时跳过)
#define ALARM_SEARCH    0xEC              //警报搜索

//Memory function commands
#define CONVERT_T       0x44              //开始转换温度
#define WR_SCRATCHPAD   0x4E              //写内部ram3、4字节
#define RD_SCRATCHPAD   0xBE              //读内部ram中9字节的内容
#define CPY_CCTATCHPAD  0x48              //复制便笺
#define RECALL_EE       0xB8              //Recall EEPROM
#define RD_PWR_SUPPLY   0xB4              //读电源供应

// Function prototypes
extern uint8 MyDS18B20_Reset(void);
extern uint8 MyDS18B20_Read(void);
extern uint8 MyDS18B20_Read1(void);
extern void MyDS18B20_Write(uint8 cmd);
extern void DS18B20_reset_search(void);
extern uint8 DS18B20_search(uint8 *newAddr);
extern uint8 DS18B20_crc8( uint8 *addr, uint8 len);
#endif
