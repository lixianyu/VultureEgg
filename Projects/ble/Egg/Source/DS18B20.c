/**************************************************************************************************
  Filename:       DS18B20.c
  Revised:        $Date: 2015-06-16 19:35:31 +0800 (Tue, 16 Jun 2015) $
  Revision:       $Revision: 1 $

**************************************************************************************************/

#include "hal_mcu.h"
#include "DS18B20.h"

#define DQ            P0_0
#define DQ_PIN        0
#define DQ_PORT       P0DIR

#define HIGH            1                 //�ߵ�ƽ
#define LOW             0                 //�͵�ƽ

#define CL_DQ()     (DQ = LOW)              //�������
#define SET_DQ()    (DQ = HIGH)             //��������

#if 0
#define SET_OUT()    (DQ_PORT |= 0x01);
#define SET_IN()     (DQ_PORT &= 0xFE);
#else
#define SET_OUT()   DQ_PORT |=  BV(DQ_PIN);
#define SET_IN()    DQ_PORT &= ~(BV(DQ_PIN));
#endif

static void delay_nus(uint16 timeout);


    // global search state
static uint8 ROM_NO[8];
static uint8 LastDiscrepancy = 0;
static uint8 LastFamilyDiscrepancy = 0;
static uint8 LastDeviceFlag = FALSE;



uint8 MyDS18B20_Reset(void)
{
    uint8 r = 99;
    uint8 retries = 125;
#if 0
    SET_IN();
    // wait until the wire is high... just in case
    do
    {
        if (--retries == 0) return 0;
        delay_nus(2);
    }
    while (!(DQ));
#endif
    SET_OUT();
    SET_DQ();         //IO������
    CL_DQ();          //IO������
    delay_nus(500);   //IO���ͺ󱣳�һ��ʱ�� 480-960us
    //SET_DQ();         //�ͷ�
    SET_IN();         //IO����Ϊ���� DS18B20->CC2540
    delay_nus(60);    //�ͷ����ߺ�ȴ�15-60us

    r = (DQ);
    //delay_nus(110);
    //SET_OUT();        //����IO����Ϊ��� CC2540->DS18B20
    //SET_DQ();         //IO����
    delay_nus(420);   //��⵽DQ ��ͺ���ʱ60-240us

    return r;
}

uint8 MyDS18B20_Read(void)
{
    uint8 rdData;     //����������
    uint8 i, dat;     //��ʱ����

    rdData = 0;               //���������ݳ�ʼ��Ϊ0

    SET_OUT();
    SET_DQ();
    /* ÿ�ζ�һλ����8�� */
    for(i = 0; i < 8; i++)
    {
        CL_DQ();            //IO����
        delay_nus(3);
        SET_IN();           //����IO����Ϊ���� DS18B20->CC2540
        delay_nus(10);
        dat = DQ;           //������,�ӵ�λ��ʼ

        if(dat)
        {
            rdData |= (1 << i); //�������������λΪ��
        }
        else
        {
            rdData &= ~(1 << i); //�������������λΪ��
        }

        delay_nus(50);      //����60~120us
        SET_OUT();          //����IO����Ϊ��� CC2540->DS18B20
        SET_DQ();
        delay_nus(1);
    }
    return (rdData);        //���ض���������
}

uint8 MyDS18B20_Read1(void)
{
    uint8 bitMask;     //����������
    uint8 i, dat;     //��ʱ����

    uint8 rdData = 0;               //���������ݳ�ʼ��Ϊ0

    //SET_OUT();
    //SET_DQ();
    /* ÿ�ζ�һλ����8�� */
    for(bitMask=0x01; bitMask; bitMask<<=1)
    {
        SET_OUT();
        CL_DQ();            //IO����
        delay_nus(3);
        SET_IN();           //����IO����Ϊ���� DS18B20->CC2540
        delay_nus(10);
        dat = DQ;           //������,�ӵ�λ��ʼ

        if(dat)
        {
            rdData |= bitMask;
        }

        delay_nus(53);      //����60~120us
        //SET_OUT();          //����IO����Ϊ��� CC2540->DS18B20
        //SET_DQ();
        //delay_nus(1);
    }
    SET_IN();
    return (rdData);        //���ض���������
}
void MyDS18B20_Write(uint8 cmd)
{
    uint8 i;
    SET_OUT();                  //����IOΪ�����2530->DS18B20
    SET_DQ();
    /*ÿ��һλ��ѭ��8��*/
    for(i = 0; i < 8; i++)
    {

        if( cmd & (1 << i) )  //д���ݴӵ�λ��ʼ
        {
            CL_DQ();              //IOΪ��
            delay_nus(1); // recovery time must be a minimum of 1us
            SET_DQ();           //IO����ߵ�ƽ
            delay_nus(65);
        }
        else
        {
            CL_DQ();            //IO����͵�ƽ
            delay_nus(65);        //Must be a minimum of 60us
            SET_DQ();             //IO������
            delay_nus(1); // recovery time must be a minimum of 1us
        }
    }
    SET_DQ();                 //IO������
}

uint8 DS18B20_read_bit(void)
{
	uint8 r;

	SET_OUT();
	CL_DQ();
	delay_nus(3);
	SET_IN();	// let pin float, pull up will raise
	delay_nus(10);
	r = DQ;
	delay_nus(53);
	return r;
}

void DS18B20_write_bit(uint8 v)
{
	if (v & 1) {
		CL_DQ();
		SET_OUT();	// drive output low
		delay_nus(10);
		SET_DQ();	// drive output high
		delay_nus(55);
	} else {
		CL_DQ();
		SET_OUT();	// drive output low
		delay_nus(65);
		SET_DQ();	// drive output high
		delay_nus(5);
	}
}

//
// You need to use this function to start a search again from the beginning.
// You do not need to do it for the first search, though you could.
//
void DS18B20_reset_search(void)
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
uint8 DS18B20_search(uint8 *newAddr)
{
    uint8 id_bit_number;
    uint8 last_zero, rom_byte_number, search_result;
    uint8 id_bit, cmp_id_bit;

    uint8 rom_byte_mask, search_direction;

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
        if (MyDS18B20_Reset())
        {
            // reset the search
            LastDiscrepancy = 0;
            LastDeviceFlag = FALSE;
            LastFamilyDiscrepancy = 0;
            return FALSE;
        }

        // issue the search command
        MyDS18B20_Write(SEARCH_ROM);
        // loop to do the search
        do
        {
            // read a bit and its complement
            id_bit = DS18B20_read_bit();
            cmp_id_bit = DS18B20_read_bit();

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
                DS18B20_write_bit(search_direction);

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

uint8 DS18B20_crc8( uint8 *addr, uint8 len)
{
	uint8 crc = 0;

	while (len--) {
		uint8 inbyte = *addr++;
		for (uint8 i = 8; i; i--) {
			uint8 mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

/*
 *    ��ʱ����
 *    ����΢��
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
