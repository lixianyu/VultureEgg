/**************************************************************************************************
  Filename:       DS18B20.h
  Revised:        $Date: 2015-06-16 19:34:58 +0800 (Tue, 16 Jun 2015) $
  Revision:       $Revision: 0 $


**************************************************************************************************/

#ifndef DS18B20_H
#define DS18B20_H

#include "hal_types.h"

//ROM function commands
#define SEARCH_ROM      0xF0              //����ROM
#define READ_ROM        0x33              //��ROM
#define MATCH_ROM       0x55              //ƥ��ROM(�Ҷ��DS18B20ʱʹ��)
#define SKIP_ROM        0xCC              //����ƥ��ROM(����DS18B20ʱ����)
#define ALARM_SEARCH    0xEC              //��������

//Memory function commands
#define CONVERT_T       0x44              //��ʼת���¶�
#define WR_SCRATCHPAD   0x4E              //д�ڲ�ram3��4�ֽ�
#define RD_SCRATCHPAD   0xBE              //���ڲ�ram��9�ֽڵ�����
#define CPY_CCTATCHPAD  0x48              //���Ʊ��
#define RECALL_EE       0xB8              //Recall EEPROM
#define RD_PWR_SUPPLY   0xB4              //����Դ��Ӧ

// Function prototypes
extern uint8 MyDS18B20_Reset(void);
extern uint8 MyDS18B20_Read(void);
extern uint8 MyDS18B20_Read1(void);
extern void MyDS18B20_Write(uint8 cmd);
extern void DS18B20_reset_search(void);
extern uint8 DS18B20_search(uint8 *newAddr);
extern uint8 DS18B20_crc8( uint8 *addr, uint8 len);
#endif
