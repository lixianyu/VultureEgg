/**************************************************************************************************
  Filename:       sensorTag.c
  Revised:        $Date: 2013-08-23 11:45:31 -0700 (Fri, 23 Aug 2013) $
  Revision:       $Revision: 35100 $

  Description:    This file contains the Sensor Tag sample application
                  for use with the TI Bluetooth Low Energy Protocol Stack.

  Copyright 2012-2013  Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_keys.h"
#include "hal_i2c.h"
//#include "hal_lcd.h"
#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

// Services
#include "st_util.h"
#include "devinfoservice-st.h"
#include "irtempservice.h"
#include "accelerometerservice.h"
#include "mpu6050service.h"
#include "ds18b20service.h"
#include "humidityservice.h"
#include "magnetometerservice.h"
#include "barometerservice.h"
#include "gyroservice.h"
#include "testservice.h"
#include "simplekeys.h"
#include "ccservice.h"

// Sensor drivers
#include "sensorTag.h"
#include "hal_sensor.h"

#include "hal_irtemp.h"
#include "hal_LM75A_nxp.h"
#include "hal_acc.h"
#include "hal_humi.h"
#include "hal_mag.h"
#include "hal_bar.h"
#include "hal_gyro.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20Egg.h"
#include "OneWire.h"
#include "i2c.h"

/*********************************************************************
 * MACROS
 */
// LCD macros
#if HAL_LCD == TRUE
#define LCD_WRITE_STRING(str, option)                       HalLcdWriteString( (str), (option))
#define LCD_WRITE_SCREEN(line1, line2)                      HalLcdWriteScreen( (line1), (line2) )
#define LCD_WRITE_STRING_VALUE(title, value, format, line)  HalLcdWriteStringValue( (title), (value), (format), (line) )

#else
#define LCD_WRITE_STRING(str, option)
#define LCD_WRITE_SCREEN(line1, line2)
#define LCD_WRITE_STRING_VALUE(title, value, format, line)
#endif
/*********************************************************************
 * CONSTANTS
 */

// How often to perform sensor reads (milliseconds)
#define TEMP_DEFAULT_PERIOD                   1000
#define HUM_DEFAULT_PERIOD                    70000
#define BAR_DEFAULT_PERIOD                    1000
#define MAG_DEFAULT_PERIOD                    2000
#define ACC_DEFAULT_PERIOD                    1000
#define GYRO_DEFAULT_PERIOD                   1000
#define MPU6050_DEFAULT_PERIOD                2000
#define DS18B20_DEFAULT_PERIOD                60000

// Constants for two-stage reading
#define TEMP_MEAS_DELAY                       275   // Conversion time 250 ms
#define BAR_FSM_PERIOD                        80
#define ACC_FSM_PERIOD                        20
#define HUM_FSM_PERIOD                        20
#define GYRO_STARTUP_TIME                     60    // Start-up time max. 50 ms
#define DS18B20_FSM_PERIOD                    900

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          1600

// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     303

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     319

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         4

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          600

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         4

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

// Side key bit
#define SK_KEY_SIDE                           0x04

// Test mode bit
#define TEST_MODE_ENABLE                      0x80

// Common values for turning a sensor on and off + config/status
#define ST_CFG_SENSOR_DISABLE                 0x00
#define ST_CFG_SENSOR_ENABLE                  0x01
#define ST_CFG_CALIBRATE                      0x02
#define ST_CFG_ERROR                          0xFF

// System reset
#define ST_SYS_RESET_DELAY                    3000

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 sensorTag_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x0B,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x56,   // 'V'
  0x75,   // 'u'
  0x6C,   // 'l'
  0x74,   // 't'
  0x75,   // 'u'
  0x72,   // 'r'
  0x65,   // 'e'
  0x45,   // 'E'
  0x67,   // 'g'
  0x67,   // 'g'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
#if 1
  19,
  GAP_ADTYPE_16BIT_MORE,

  0x45,0x45, // This the magic number.
/*7*/   0x01,0x02,0x03,0x04,
/*11*/  0x05,0x06,0x07,0x08,
/*15*/  0x09,0x0a,0x0b,0x0c,
/*19*/  0x0d,0x0e,0x0f,0x10,
#else
  0x03,
  GAP_ADTYPE_16BIT_MORE,
  0x88, 0xFF,
#endif
};

// GAP GATT Attributes
static uint8 attDeviceName[] = "Egg";

// Sensor State Variables
static bool   irTempEnabled = FALSE;
static bool   magEnabled = FALSE;
static uint8  accConfig = ST_CFG_SENSOR_DISABLE;
static uint8  mpu6050Config = ST_CFG_SENSOR_DISABLE;
static bool   ds18b20Enabled = FALSE;
static bool   barEnabled = FALSE;
static bool   humiEnabled = FALSE;
static bool   gyroEnabled = FALSE;

static uint8 gsendbuffer[36];
static uint8 gsendbufferI;
static int flagRom = 0;
typedef enum
{
    EGG_STATE_MEASURE_IDLE,
    EGG_STATE_MEASURE_HUMIDITY,
    EGG_STATE_MEASURE_LM75A,
    EGG_STATE_MEASURE_MPU6050
} t_enum_EggState;
static t_enum_EggState gEggState = EGG_STATE_MEASURE_IDLE; // 0 : Measure humidity; 1 : Measure DS18B20; 2 : Measure MPU6050
static bool   barBusy = FALSE;
static uint8  humiState = 0;
static uint8  ds18b20State = 0;

static bool   sysResetRequest = FALSE;

static uint16 sensorMagPeriod = MAG_DEFAULT_PERIOD;
static uint16 sensorAccPeriod = ACC_DEFAULT_PERIOD;
static uint16 sensorTmpPeriod = TEMP_DEFAULT_PERIOD;
static uint16 sensorHumPeriod = HUM_DEFAULT_PERIOD;
static uint16 sensorBarPeriod = BAR_DEFAULT_PERIOD;
static uint16 sensorGyrPeriod = GYRO_DEFAULT_PERIOD;
static uint16 sensorMpu6050Period = MPU6050_DEFAULT_PERIOD;
static uint16 sensorDs18b20Period = DS18B20_DEFAULT_PERIOD;

static uint8  sensorGyroAxes = 0;
static bool   sensorGyroUpdateAxes = FALSE;
static uint16 selfTestResult = 0;
static bool   testMode = FALSE;

static uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
static uint16_t fifoCount;     // count of all bytes currently in FIFO
static uint8_t fifoBuffer[64]; // FIFO storage buffer
static uint8_t mpuIntStatus;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorTag_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );

static void readIrTempData( void );
static void readHumData( void );
static void readAccData( void );
static void readMagData( void );
static void readBarData( void );
static void readBarCalibration( void );
static void readGyroData( void );
static void readMPU6050DataAdv( void );
static void readMPU6050DmpData( uint8 *packet );
static void readDs18b20Data( void );
static void readDs18b20Data1( uint8* mData, uint8 flagrom);
static void readDs18b20WithState(uint8 state, uint8 flagrom);
static void readDs18b20WithState1(uint8 state, uint8 flagrom);
static void barometerChangeCB( uint8 paramID );
static void irTempChangeCB( uint8 paramID );
static void accelChangeCB( uint8 paramID );
static void mpu6050ChangeCB( uint8 paramID );
static void ds18b20ChangeCB( uint8 paramID );
static void humidityChangeCB( uint8 paramID);
static void magnetometerChangeCB( uint8 paramID );
static void gyroChangeCB( uint8 paramID );
static void testChangeCB( uint8 paramID );
static void ccChangeCB( uint8 paramID );
static void gapRolesParamUpdateCB( uint16 connInterval, uint16 connSlaveLatency,uint16 connTimeout );

static void resetSensorSetup( void );
static void sensorTag_HandleKeys( uint8 shift, uint8 keys );
static void resetCharacteristicValue( uint16 servID, uint8 paramID, uint8 value, uint8 paramLen );
static void resetCharacteristicValues( void );
static void mpu6050StarWhenConnected(void);
static void humidityStarWhenConnected(void);
static void ds18b20StarWhenConnected(void);
static void lm75aStarWhenConnected(void);
static void eggSerialAppSendNoti(uint8 *pBuffer,uint16 length);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t sensorTag_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t sensorTag_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static sensorCBs_t sensorTag_BarometerCBs =
{
  barometerChangeCB,        // Characteristic value change callback
};

static sensorCBs_t sensorTag_IrTempCBs =
{
  irTempChangeCB,           // Characteristic value change callback
};

static sensorCBs_t sensorTag_AccelCBs =
{
  accelChangeCB,            // Characteristic value change callback
};

static sensorCBs_t sensorTag_Mpu6050CBs =
{
  mpu6050ChangeCB,              // Characteristic value change callback
};

static sensorCBs_t sensorTag_Ds18b20CBs =
{
  ds18b20ChangeCB,              // Characteristic value change callback
};

static sensorCBs_t sensorTag_HumidCBs =
{
  humidityChangeCB,         // Characteristic value change callback
};

static sensorCBs_t sensorTag_MagnetometerCBs =
{
  magnetometerChangeCB,     // Characteristic value change callback
};

static sensorCBs_t sensorTag_GyroCBs =
{
  gyroChangeCB,             // Characteristic value change callback
};

static testCBs_t sensorTag_TestCBs =
{
  testChangeCB,             // Charactersitic value change callback
};

static ccCBs_t sensorTag_ccCBs =
{
 ccChangeCB,               // Charactersitic value change callback
};

static gapRolesParamUpdateCB_t paramUpdateCB =
{
  gapRolesParamUpdateCB,
};

#define LED1 P1_0       //定义P1.0口为LED1控制端
#define LED2 P1_1       //定义P1.1口为LED2控制端
#define LED3 P1_4       //定义P1.4口为LED3控制端
#define LED4 P0_1       //定义P0.1口为LED4控制端
/****************************************************************************
* 名    称: DelayMS()
* 功    能: 以毫秒为单位延时，系统时钟不配置时默认为16M(用示波器测量相当精确)
* 入口参数: msec 延时参数，值越大，延时越久
* 出口参数: 无
****************************************************************************/
void DelayMS(uint32 msec)
{
    uint32 i,j;

    for (i=0; i<msec; i++)
        for (j=0; j<535; j++);
}
/****************************************************************************
* 名    称: LedOnOrOff()
* 功    能: 点亮或熄灭所有LED灯
* 入口参数: mode为1时LED灯亮  mode为0时LED灯灭， 共阴极
* 出口参数: 无
****************************************************************************/
void LedOnOrOff(uint8 mode)
{
    LED1 = mode;
    LED2 = mode;
    LED3 = mode;
    LED4 = mode;
}
/****************************************************************************
* 名    称: InitLed()
* 功    能: 设置LED灯相应的IO口
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void InitLed(void)
{
    P1DIR |= 0x13;      //P1.0、P1.1、P1.4定义为输出
    P0DIR |= 0x02;      //P0.1定义为输出
    LedOnOrOff(0);      //使所有LED灯默认为熄灭状态
}

void initDS18B20(void) {
    P0DIR |= 0x02;
}

void eggLeds(void) {
    LED1 = !LED1;         //流水灯，初始化时LED为熄灭执行后则点亮
    DelayMS(200);
    LED2 = !LED2;
    DelayMS(200);
    LED3 = !LED3;
    DelayMS(200);
    LED4 = !LED4;
    DelayMS(200);

    for (uint8 i=0; i<2; i++)   //所有灯闪烁2次
    {
       LedOnOrOff(0);    //关闭所有LED灯
       DelayMS(200);
       LedOnOrOff(1);    //打开所有LED灯
       DelayMS(200);
    }

    LedOnOrOff(0);       //使所有LED灯熄灭状态
    DelayMS(500);
}
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SensorTag_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SensorTag_Init( uint8 task_id )
{
  sensorTag_TaskID = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

  // Setup the GAP Peripheral Role Profile
  {
    // Device starts advertising upon initialization
    uint8 initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 1;
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, sizeof(attDeviceName), attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;

    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }


  // Add services
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  //IRTemp_AddService (GATT_ALL_SERVICES );         // IR Temperature Service
  //Accel_AddService (GATT_ALL_SERVICES );          // Accelerometer Service
  Mpu6050_AddService(GATT_ALL_SERVICES);
  Ds18b20_AddService(GATT_ALL_SERVICES);
  Humidity_AddService (GATT_ALL_SERVICES );       // Humidity Service
  //Magnetometer_AddService( GATT_ALL_SERVICES );   // Magnetometer Service
  //Barometer_AddService( GATT_ALL_SERVICES );      // Barometer Service
  //Gyro_AddService( GATT_ALL_SERVICES );           // Gyro Service
  //SK_AddService( GATT_ALL_SERVICES );             // Simple Keys Profile
  //Test_AddService( GATT_ALL_SERVICES );           // Test Profile
  CcService_AddService( GATT_ALL_SERVICES );      // Connection Control Service

#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the Seensor Profile Characteristic Values
  //resetCharacteristicValues();

  // Register for all key events - This app will handle all key events
  //RegisterForKeys( sensorTag_TaskID );

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // Initialise sensor drivers
  //HALIRTempInit();
  HalHumiInit();
  //HalMagInit();
  //HalAccInit();
  //HalBarInit();
  //HalGyroInit();
  //HalMPU6050initialize();

  // Register callbacks with profile
  //VOID IRTemp_RegisterAppCBs( &sensorTag_IrTempCBs );
  //VOID Magnetometer_RegisterAppCBs( &sensorTag_MagnetometerCBs );
  //VOID Accel_RegisterAppCBs( &sensorTag_AccelCBs );
  VOID Mpu6050_RegisterAppCBs(&sensorTag_Mpu6050CBs);
  VOID Ds18b20_RegisterAppCBs(&sensorTag_Ds18b20CBs);
  VOID Humidity_RegisterAppCBs( &sensorTag_HumidCBs );
  //VOID Barometer_RegisterAppCBs( &sensorTag_BarometerCBs );
  //VOID Gyro_RegisterAppCBs( &sensorTag_GyroCBs );
  //VOID Test_RegisterAppCBs( &sensorTag_TestCBs );
  VOID CcService_RegisterAppCBs( &sensorTag_ccCBs );
  VOID GAPRole_RegisterAppCBs( &paramUpdateCB );

  //InitLed();
  //initDS18B20();

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  // Setup a delayed profile startup
  osal_set_event( sensorTag_TaskID, ST_START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      SensorTag_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SensorTag_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( sensorTag_TaskID )) != NULL )
    {
      sensorTag_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Handle system reset (long press on side key)
  if ( events & ST_SYS_RESET_EVT )
  {
    if (sysResetRequest)
    {
      HAL_SYSTEM_RESET();
    }
    return ( events ^ ST_SYS_RESET_EVT );
  }

  if ( events & ST_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &sensorTag_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &sensorTag_BondMgrCBs );

    HALLM75ATempInit();
    EggLM75ATempInit();
    //osal_start_timerEx( sensorTag_TaskID, ST_LED_EVT, 500 );
    //osal_start_timerEx( sensorTag_TaskID, ST_MPU6050_SENSOR_EVT, sensorMpu6050Period );
    //osal_start_timerEx( sensorTag_TaskID, ST_LM75A_SENSOR_GPIO_EVT, 5000 );
    return ( events ^ ST_START_DEVICE_EVT );
  }
  if ( events & ST_DS18B20_CONTINUE_EVT )
  {
    //eggLeds();
    //osal_start_timerEx( sensorTag_TaskID, ST_LED_EVT, 500 );
    eggSerialAppSendNoti(gsendbuffer+17, 16);
    return ( events ^ ST_DS18B20_CONTINUE_EVT );
  }

  //////////////////////////
  //    IR TEMPERATURE    //
  //////////////////////////
  if ( events & ST_IRTEMPERATURE_READ_EVT )
  {
    if ( irTempEnabled )
    {
      if (HalIRTempStatus() == TMP006_DATA_READY)
      {
        readIrTempData();
        osal_start_timerEx( sensorTag_TaskID, ST_IRTEMPERATURE_READ_EVT, sensorTmpPeriod-TEMP_MEAS_DELAY );
      }
      else if (HalIRTempStatus() == TMP006_OFF)
      {
        HalIRTempTurnOn();
        osal_start_timerEx( sensorTag_TaskID, ST_IRTEMPERATURE_READ_EVT, TEMP_MEAS_DELAY );
      }
    }
    else
    {
      //Turn off Temperatur sensor
      VOID HalIRTempTurnOff();
      VOID resetCharacteristicValue(IRTEMPERATURE_SERV_UUID,SENSOR_DATA,0,IRTEMPERATURE_DATA_LEN);
      VOID resetCharacteristicValue(IRTEMPERATURE_SERV_UUID,SENSOR_CONF,ST_CFG_SENSOR_DISABLE,sizeof ( uint8 ));
    }

    return (events ^ ST_IRTEMPERATURE_READ_EVT);
  }

  //////////////////////////
  //    Accelerometer     //
  //////////////////////////
  if ( events & ST_ACCELEROMETER_SENSOR_EVT )
  {
    if(accConfig != ST_CFG_SENSOR_DISABLE)
    {
      readAccData();
      osal_start_timerEx( sensorTag_TaskID, ST_ACCELEROMETER_SENSOR_EVT, sensorAccPeriod );
    }
    else
    {
      VOID resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_DATA, 0, ACCELEROMETER_DATA_LEN );
      VOID resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ ST_ACCELEROMETER_SENSOR_EVT);
  }

  //////////////////////////
  //    MPU6050           //
  //////////////////////////
  if ( events & ST_MPU6050_SENSOR_EVT )
  {
    if(mpu6050Config != ST_CFG_SENSOR_DISABLE)
    {
        if (gEggState == EGG_STATE_MEASURE_HUMIDITY||
            gEggState == EGG_STATE_MEASURE_LM75A)
        {
            osal_start_timerEx( sensorTag_TaskID, ST_MPU6050_SENSOR_EVT, 1000 );
            return (events ^ ST_MPU6050_SENSOR_EVT);
        }
        
        gEggState = EGG_STATE_MEASURE_MPU6050;
        mpuIntStatus = HalMPU6050getIntStatus();
        fifoCount = HalMPU6050getFIFOCount();
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        //if (fifoCount == 1024) {
        // reset so we can continue cleanly
           HalMPU6050resetFIFO();
           osal_start_timerEx( sensorTag_TaskID, ST_MPU6050_SENSOR_EVT, 10 );
           return (events ^ ST_MPU6050_SENSOR_EVT);
        }
        if (mpuIntStatus & 0x02)
        {
            while (fifoCount < packetSize) 
            {
                fifoCount = HalMPU6050getFIFOCount();
            }
            HalMPU6050getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
            readMPU6050DmpData(fifoBuffer);
        }
        osal_start_timerEx( sensorTag_TaskID, ST_MPU6050_SENSOR_EVT, sensorMpu6050Period );
        gEggState = EGG_STATE_MEASURE_IDLE;
        HalMPU6050resetFIFO();
    }
    else
    {
        //TODO : sleep the MPU6050.

    }
    return (events ^ ST_MPU6050_SENSOR_EVT);
  }

  if (events & ST_MPU6050_DMP_INIT_EVT)
  {
    HalMPU6050dmpInitialize();
    HalMPU6050setDMPEnabled(true);
    osal_start_timerEx( sensorTag_TaskID, ST_MPU6050_SENSOR_EVT, 4000 );
    return (events ^ ST_MPU6050_DMP_INIT_EVT);
  }
  //////////////////////////
  //    DS18B20           //
  //////////////////////////
  if ( events & ST_DS18B20_SENSOR_EVT )
  {
    if (ds18b20Enabled == TRUE)
    {
        if (gEggState == EGG_STATE_MEASURE_HUMIDITY ||
            gEggState == EGG_STATE_MEASURE_MPU6050)
        {//Try again after 1000ms.
            osal_start_timerEx( sensorTag_TaskID, ST_DS18B20_SENSOR_EVT, 1000 );
            return (events ^ ST_DS18B20_SENSOR_EVT);
        }
        gEggState = EGG_STATE_MEASURE_LM75A;
        if (ds18b20State == 0)
        {
            readDs18b20WithState(0, flagRom);
        }
        else {
            readDs18b20WithState(1, flagRom++);
        }

        if (ds18b20State == 0)
        {
            osal_start_timerEx( sensorTag_TaskID, ST_DS18B20_SENSOR_EVT, 1000 );
            ds18b20State = 1;
        }
        else
        {
            if (flagRom >= 14)
            {
                gEggState = EGG_STATE_MEASURE_IDLE;
                flagRom = 0;
                osal_start_timerEx( sensorTag_TaskID, ST_DS18B20_SENSOR_EVT, sensorDs18b20Period );
            }
            else {
                osal_start_timerEx( sensorTag_TaskID, ST_DS18B20_SENSOR_EVT, 100 );
            }
            ds18b20State = 0;
        }
    }
    else
    {
        //TODO : sleep the DS18B20.

    }
    return (events ^ ST_DS18B20_SENSOR_EVT);
  }
  //////////////////////////
  //    LM75A             //
  //////////////////////////
  if (events & ST_LM75A_SENSOR_GPIO_EVT)
  {
    uint8 lm75abuffer[16] = {0};
    //HalLM75ATempTurnOn(1);
    //HalLM75ATempRead(1, lm75abuffer);
    EggReadAllLM75ATemp(lm75abuffer);
    //HalLM75ATempReadAll(lm75abuffer);
    //osal_memcpy(advertData+7, lm75abuffer, sizeof(lm75abuffer));
    //GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    //osal_start_timerEx( sensorTag_TaskID, ST_LM75A_SENSOR_GPIO_EVT, 4000 );
    #if 1
    osal_memset(gsendbuffer, 0xFF, sizeof(gsendbuffer));
    gsendbuffer[0] = 0xAA; // 0
    gsendbuffer[1] = 0xBB;
    gsendbuffer[2] = 0xBB; // 2
    osal_memcpy(gsendbuffer+3, lm75abuffer, sizeof(lm75abuffer));
    gsendbuffer[3+sizeof(lm75abuffer)] = 0x0D;
    gsendbuffer[3+sizeof(lm75abuffer)+1] = 0x0A;
    eggSerialAppSendNoti(gsendbuffer, 11);
    //ST_HAL_DELAY(625); //Delay 5ms
    ST_HAL_DELAY(1001); //Delay 8ms
    eggSerialAppSendNoti(gsendbuffer+11, 10);
    gEggState = EGG_STATE_MEASURE_IDLE;
    osal_start_timerEx( sensorTag_TaskID, ST_LM75A_SENSOR_EVT, 60000 );
    #endif
    return (events ^ ST_LM75A_SENSOR_GPIO_EVT);
  }
  
  if ( events & ST_LM75A_SENSOR_EVT )
  {
    if (gEggState == EGG_STATE_MEASURE_HUMIDITY ||
        gEggState == EGG_STATE_MEASURE_MPU6050)
    {   //Try again after 1500ms.
        osal_start_timerEx( sensorTag_TaskID, ST_LM75A_SENSOR_EVT, 1500 );
        return (events ^ ST_LM75A_SENSOR_EVT);
    }
    gEggState = EGG_STATE_MEASURE_LM75A;
    uint8 lm75abuffer[16] = {0};
    HalLM75ATempReadAll(lm75abuffer);
    osal_memset(gsendbuffer, 0xFF, sizeof(gsendbuffer));
    gsendbuffer[0] = 0xAA; // 0
    gsendbuffer[1] = 0xBB;
    gsendbuffer[2] = 0xBB; // 2
    osal_memcpy(gsendbuffer+3, lm75abuffer, sizeof(lm75abuffer));
    gsendbuffer[3+sizeof(lm75abuffer)] = 0x0D;
    gsendbuffer[3+sizeof(lm75abuffer)+1] = 0x0A;
    eggSerialAppSendNoti(gsendbuffer, 11);
    //ST_HAL_DELAY(625); //Delay 5ms
    ST_HAL_DELAY(1000); //Delay 8ms
    eggSerialAppSendNoti(gsendbuffer+11, 10);
    //gEggState = EGG_STATE_MEASURE_IDLE;
    osal_start_timerEx( sensorTag_TaskID, ST_LM75A_SENSOR_GPIO_EVT, 400 );
    return (events ^ ST_LM75A_SENSOR_EVT);
  }
  
  //////////////////////////
  //      Humidity        //
  //////////////////////////
  if ( events & ST_HUMIDITY_SENSOR_EVT )
  {
    if (humiEnabled)
    {
      if (gEggState == EGG_STATE_MEASURE_MPU6050 ||
          gEggState == EGG_STATE_MEASURE_LM75A)
      {
        osal_start_timerEx( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT, 1000 );//Try again after 1000ms.
        return (events ^ ST_HUMIDITY_SENSOR_EVT);
      }
      gEggState = EGG_STATE_MEASURE_HUMIDITY;
      bool returnValue = HalHumiExecMeasurementStep(humiState);
      /*if (!returnValue)
      {
        gEggState = EGG_STATE_MEASURE_IDLE;
        humiState = 0;
        osal_start_timerEx( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT, sensorHumPeriod );
        return (events ^ ST_HUMIDITY_SENSOR_EVT);
      }*/
      if (humiState == 2)
      {
        readHumData();
        humiState = 0;
        gEggState = EGG_STATE_MEASURE_IDLE;
        osal_start_timerEx( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT, sensorHumPeriod );
      }
      else
      {
        humiState++;
        osal_start_timerEx( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT, HUM_FSM_PERIOD );
      }
    }
    else
    {
      resetCharacteristicValue( HUMIDITY_SERV_UUID, SENSOR_DATA, 0, HUMIDITY_DATA_LEN);
      resetCharacteristicValue( HUMIDITY_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ ST_HUMIDITY_SENSOR_EVT);
  }

  //////////////////////////
  //      Magnetometer    //
  //////////////////////////
  if ( events & ST_MAGNETOMETER_SENSOR_EVT )
  {
    if(magEnabled)
    {
      if (HalMagStatus() == MAG3110_DATA_READY)
      {
        readMagData();
      }
      else if (HalMagStatus() == MAG3110_OFF)
      {
        HalMagTurnOn();
      }

      osal_start_timerEx( sensorTag_TaskID, ST_MAGNETOMETER_SENSOR_EVT, sensorMagPeriod );
    }
    else
    {
      HalMagTurnOff();
      resetCharacteristicValue( MAGNETOMETER_SERV_UUID, SENSOR_DATA, 0, MAGNETOMETER_DATA_LEN);
      resetCharacteristicValue( MAGNETOMETER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ ST_MAGNETOMETER_SENSOR_EVT);
  }

  //////////////////////////
  //        Barometer     //
  //////////////////////////
  if ( events & ST_BAROMETER_SENSOR_EVT )
  {
    if (barEnabled)
    {
      if (barBusy)
      {
        barBusy = FALSE;
        readBarData();
        osal_start_timerEx( sensorTag_TaskID, ST_BAROMETER_SENSOR_EVT, sensorBarPeriod );
      }
      else
      {
        barBusy = TRUE;
        HalBarStartMeasurement();
        osal_start_timerEx( sensorTag_TaskID, ST_BAROMETER_SENSOR_EVT, BAR_FSM_PERIOD );
      }
    }
    else
    {
      resetCharacteristicValue( BAROMETER_SERV_UUID, SENSOR_DATA, 0, BAROMETER_DATA_LEN);
      resetCharacteristicValue( BAROMETER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
      resetCharacteristicValue( BAROMETER_SERV_UUID, SENSOR_CALB, 0, BAROMETER_CALI_LEN);
    }

    return (events ^ ST_BAROMETER_SENSOR_EVT);
  }

  //////////////////////////
  //      Gyroscope       //
  //////////////////////////
  if ( events & ST_GYROSCOPE_SENSOR_EVT )
  {
    uint8 status;

    status = HalGyroStatus();

    if(gyroEnabled)
    {
      if (status == HAL_GYRO_STOPPED)
      {
        HalGyroSelectAxes(sensorGyroAxes);
        HalGyroTurnOn();
        osal_start_timerEx( sensorTag_TaskID, ST_GYROSCOPE_SENSOR_EVT, GYRO_STARTUP_TIME);
      }
      else
      {
        if(sensorGyroUpdateAxes)
        {
          HalGyroSelectAxes(sensorGyroAxes);
          sensorGyroUpdateAxes = FALSE;
        }

        if (status == HAL_GYRO_DATA_READY)
        {
          readGyroData();
          osal_start_timerEx( sensorTag_TaskID, ST_GYROSCOPE_SENSOR_EVT, sensorGyrPeriod - GYRO_STARTUP_TIME);
        }
        else
        {
          // Gyro needs to be activated;
          HalGyroWakeUp();
          osal_start_timerEx( sensorTag_TaskID, ST_GYROSCOPE_SENSOR_EVT, GYRO_STARTUP_TIME);
        }
      }
    }
    else
    {
      HalGyroTurnOff();
      resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_DATA, 0, GYROSCOPE_DATA_LEN);
      resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof( uint8 ));
    }

    return (events ^ ST_GYROSCOPE_SENSOR_EVT);
  }

  if ( events & ST_CONN_PARAM_UPDATE_EVT )
  {
    // Send param update.  If it fails, retry until successful.
    GAPRole_SendUpdateParam( DEFAULT_DESIRED_MIN_CONN_INTERVAL, DEFAULT_DESIRED_MAX_CONN_INTERVAL,
                             DEFAULT_DESIRED_SLAVE_LATENCY, DEFAULT_DESIRED_CONN_TIMEOUT,
                             GAPROLE_TERMINATE_LINK );
    return (events ^ ST_CONN_PARAM_UPDATE_EVT);
  }
#if defined ( PLUS_BROADCASTER )
  if ( events & ST_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ ST_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      sensorTag_test
 *
 * @brief   Run a self-test of the sensor TAG
 *
 * @param   none
 *
 * @return  bitmask of error flags
 */
uint16 sensorTag_test(void)
{
  selfTestResult = HalSensorTest();
  HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);

  // Write the self-test result to the test service
  Test_SetParameter( TEST_DATA_ATTR, TEST_DATA_LEN, &selfTestResult);

  return selfTestResult;
}

/*********************************************************************
* Private functions
*/


/*********************************************************************
 * @fn      sensorTag_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void sensorTag_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      sensorTag_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    default:
      // do nothing
      break;
  }
}

/*********************************************************************
 * @fn      sensorTag_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void sensorTag_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;
  VOID shift;  // Intentionally unreferenced parameter

  if (keys & HAL_KEY_SW_1)
  {
    // Reset the system if side key is pressed for more than 3 seconds
    sysResetRequest = TRUE;
    osal_start_timerEx( sensorTag_TaskID, ST_SYS_RESET_EVT, ST_SYS_RESET_DELAY );

    if (!testMode ) // Side key
    {
      // If device is not in a connection, pressing the side key should toggle
      //  advertising on and off
      if ( gapProfileState != GAPROLE_CONNECTED )
      {
        uint8 current_adv_enabled_status;
        uint8 new_adv_enabled_status;

        // Find the current GAP advertising status
        GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

        if( current_adv_enabled_status == FALSE )
        {
          new_adv_enabled_status = TRUE;
        }
        else
        {
          new_adv_enabled_status = FALSE;
        }

        // Change the GAP advertisement status to opposite of current status
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
      }

      if ( gapProfileState == GAPROLE_CONNECTED )
      {
        uint8 adv_enabled = TRUE;

        // Disconnect
        GAPRole_TerminateConnection();
        // Start advertising
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &adv_enabled );
      }
    }
    else
    {
      // Test mode
      if ( keys & HAL_KEY_SW_1 ) // Side key
      {
        SK_Keys |= SK_KEY_SIDE;
      }
    }
  }

  if ( keys & HAL_KEY_SW_2 )   // Carbon S2
  {
    SK_Keys |= SK_KEY_LEFT;
  }

  if ( keys & HAL_KEY_SW_3 )   // Carbon S3
  {
    SK_Keys |= SK_KEY_RIGHT;
  }

  if (!(keys & HAL_KEY_SW_1))
  {
    // Cancel system reset request
    sysResetRequest = FALSE;
  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}


/*********************************************************************
 * @fn      resetSensorSetup
 *
 * @brief   Turn off all sensors that are on
 *
 * @param   none
 *
 * @return  none
 */
static void resetSensorSetup (void)
{
/*
  if (HalIRTempStatus()!=TMP006_OFF || irTempEnabled)
  {
    HalIRTempTurnOff();
    irTempEnabled = FALSE;
  }

  if (accConfig != ST_CFG_SENSOR_DISABLE)
  {
    accConfig = ST_CFG_SENSOR_DISABLE;
  }
*/
  if (mpu6050Config != ST_CFG_SENSOR_DISABLE)
  {
    mpu6050Config = ST_CFG_SENSOR_DISABLE;
  }
/*
  if (HalMagStatus()!=MAG3110_OFF || magEnabled)
  {
    HalMagTurnOff();
    magEnabled = FALSE;
  }

  if (gyroEnabled)
  {
    HalGyroTurnOff();
    gyroEnabled = FALSE;
  }

  if (barEnabled)
  {
    HalBarInit();
    barEnabled = FALSE;
  }
*/
  if (humiEnabled)
  {
    HalHumiInit();
    humiEnabled = FALSE;
  }
  if (ds18b20Enabled)
  {
    ds18b20Enabled = FALSE;
  }
/*
  // Reset internal states
  sensorGyroAxes = 0;
  sensorGyroUpdateAxes = FALSE;
  testMode = FALSE;
*/
  // Reset all characteristics values
  //resetCharacteristicValues();
}


/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
    {
        // Set the system ID from the bd addr
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
        uint8 systemIdBak[DEVINFO_SYSTEM_ID_LEN];
        GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);
        osal_memcpy(systemIdBak, systemId, DEVINFO_SYSTEM_ID_LEN);

        // shift three bytes up
        systemId[7] = systemId[5];
        systemId[6] = systemId[4];
        systemId[5] = systemId[3];

        // set middle bytes to zero
        systemId[4] = 0;
        systemId[3] = 0;
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Set the serial number from the bd addr.
        uint8 serialNumber[DEVINFO_SERIAL_NUMBER_LEN+2] = "\0";
        uint8 aNumber;
        uint8 j = 0;
        for (int8 i = B_ADDR_LEN-1; i >= 0; i--)
        {
            aNumber = systemIdBak[i];
            if (aNumber < 10)
            {
                strcat((char*)serialNumber+j*2, (const char*)"0");
                _itoa(aNumber, serialNumber+j*2+1, 16);
            }
            else
            {
                _itoa(aNumber, serialNumber+j*2, 16);
            }

            /*if (osal_memcmp(&aNumber, &Zero, 1) == TRUE)
            {
                strcat((char*)serialNumber+j*2+1, (const char*)"0");
            }*/
            j++;
        }
        DevInfo_SetParameter(DEVINFO_SERIAL_NUMBER, DEVINFO_SERIAL_NUMBER_LEN, serialNumber);
    }
    break;

    case GAPROLE_ADVERTISING:
	    HalLedSet(HAL_LED_1, HAL_LED_MODE_ON );
	    break;

    case GAPROLE_CONNECTED:
        // Set timer to update connection parameters
        // 5 seconds should allow enough time for Service Discovery by the collector to finish
        osal_start_timerEx( sensorTag_TaskID, ST_CONN_PARAM_UPDATE_EVT, 6000);
        HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF );
        gEggState = EGG_STATE_MEASURE_IDLE;
        //mpu6050StarWhenConnected();
        //humidityStarWhenConnected();
        //ds18b20StarWhenConnected();
        lm75aStarWhenConnected();
        break;

    case GAPROLE_WAITING:
      // Link terminated intentionally: reset all sensors
      resetSensorSetup();
      break;

	  default:
	    break;
  }

  gapProfileState = newState;
}

/*********************************************************************
 * @fn      readAccData
 *
 * @brief   Read accelerometer data
 *
 * @param   none
 *
 * @return  none
 */
static void readAccData(void)
{
  uint8 aData[ACCELEROMETER_DATA_LEN];

  if (HalAccRead(aData))
  {
    Accel_SetParameter( SENSOR_DATA, ACCELEROMETER_DATA_LEN, aData);
  }
}

#if 0
static void readMPU6050DataAdv( void )
{
    int16 ax,ay,az,gx,gy,gz;
    uint8 *p = (uint8*)&ax;
    uint8 buffers[MPU6050_DATA_LEN];
    HalMPU6050getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    buffers[0] = *(p+1);
    buffers[1] = *p;
    p = (uint8*)&ay;
    buffers[2] = *(p+1);
    buffers[3] = *p;
    p = (uint8*)&az;
    buffers[4] = *(p+1);
    buffers[5] = *p;
    p = (uint8*)&gx;
    buffers[6] = *(p+1);
    buffers[7] = *p;
    p = (uint8*)&gy;
    buffers[8] = *(p+1);
    buffers[9] = *p;
    p = (uint8*)&gz;
    buffers[10] = *(p+1);
    buffers[11] = *p;
    //GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    Mpu6050_SetParameter(SENSOR_DATA, MPU6050_DATA_LEN, buffers);
    eggSerialAppSendNoti(buffers, MPU6050_DATA_LEN);
}
#else
static void readMPU6050DataAdv( void )
{
    int16 ax,ay,az,gx,gy,gz;
    uint8 buffers[12];
    HalMPU6050getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    buffers[1] = ax >> 8;
    buffers[0] = ax & 0xFF;
    buffers[3] = ay >> 8;
    buffers[2] = ay & 0xFF;
    buffers[5] = az >> 8;
    buffers[4] = az & 0xFF;
    buffers[7] = gx >> 8;
    buffers[6] = gx & 0xFF;
    buffers[9] = gy >> 8;
    buffers[8] = gy & 0xFF;
    buffers[11] = gz >> 8;
    buffers[10] = gz & 0xFF;

    uint8 sendbuffer[12+5];
    sendbuffer[0] = 0xAA;
    sendbuffer[1] = 0xBB;
    sendbuffer[2] = 0xAA;
    VOID osal_memcpy( sendbuffer+3, buffers, 12 );
    sendbuffer[15] = 0x0D;
    sendbuffer[16] = 0x0A;
    //Mpu6050_SetParameter(SENSOR_DATA, MPU6050_DATA_LEN, buffers);
    eggSerialAppSendNoti(sendbuffer, 12+5);
}

static void readMPU6050DmpData( uint8 *packet )
{
    int16 ax,ay,az;
    ax = 0;
    ay = 0;
    az = 0;
    uint8 buffers[MPU6050_DATA_LEN];
    int16_t qI[4];
    HalMPU6050getAcceleration(&ax, &ay, &az);
    HalMPU6050dmpGetQuaternion(qI, packet);
//    float w = (float)qI[0] / 16384.0f;
        
    buffers[1] = ax >> 8;
    buffers[0] = ax & 0xFF;
    buffers[3] = ay >> 8;
    buffers[2] = ay & 0xFF;
    buffers[5] = az >> 8;
    buffers[4] = az & 0xFF;
#if 1
    buffers[7] = qI[0] >> 8;
    buffers[6] = qI[0] & 0xFF;
    buffers[9] = qI[1] >> 8;
    buffers[8] = qI[1] & 0xFF;
    buffers[11] = qI[2] >> 8;
    buffers[10] = qI[2] & 0xFF;
    buffers[13] = qI[3] >> 8;
    buffers[12] = qI[3] & 0xFF;
#else
    buffers[6] = qI[0] >> 8;
    buffers[7] = qI[0] & 0xFF;
    buffers[8] = qI[1] >> 8;
    buffers[9] = qI[1] & 0xFF;
    buffers[10] = qI[2] >> 8;
    buffers[11] = qI[2] & 0xFF;
    buffers[12] = qI[3] >> 8;
    buffers[13] = qI[3] & 0xFF;
#endif
    uint8 sendbuffer[MPU6050_DATA_LEN+5];
    sendbuffer[0] = 0xAA;
    sendbuffer[1] = 0xBB;
    sendbuffer[2] = 0xAA;
    VOID osal_memcpy( sendbuffer+3, buffers, MPU6050_DATA_LEN );
    sendbuffer[17] = 0x0D;
    sendbuffer[18] = 0x0A;
    //Mpu6050_SetParameter(SENSOR_DATA, MPU6050_DATA_LEN, buffers);
    eggSerialAppSendNoti(sendbuffer, MPU6050_DATA_LEN+5);
}
#endif

static void readDs18b20Data( void )
{
  uint8 mData[DS18B20_DATA_LEN];


  float temp = DS18B20_ReadMain(mData, DS18B20_DATA_LEN);
  //LCD_WRITE_STRING_VALUE( "Temp Now: ", temp, 10, HAL_LCD_LINE_1 );
#if 0
  uint8 i;
  uint8 present = 0;
  uint8 data[12];
  uint8 addr[8];
  float celsius;
  if (OneWire_search(addr))
  {
    OneWire_reset_search();
    ST_HAL_DELAY(31250);// 250ms
    mData[0] = 0x88;
    mData[1] = 0x87;
    Ds18b20_SetParameter(SENSOR_DATA, DS18B20_DATA_LEN, mData);
    return;
  }

  OneWire_reset();
  OneWire_select(addr);
  OneWire_write(0x44, 1);
  ST_HAL_DELAY(125000);
  present = OneWire_reset();
  OneWire_select(addr);
  OneWire_write(0xBE, 0);
  for (int i = 0; i < 2; i++)
  {
    mData[i] = OneWire_read();
  }
  #endif
  Ds18b20_SetParameter(SENSOR_DATA, DS18B20_DATA_LEN, mData);
}

static void readDs18b20Data1( uint8* mData, uint8 flagrom)
{
    uint8 rom[8] = {0x28, 0x5C, 0x1F, 0x92, 0x04, 0x00, 0x00, 0x26};
    uint8 rom1[8] = {0x28, 0x12, 0x91, 0xA1, 0x05, 0x00, 0x00, 0x42};
    uint8 rom2[8] = {0x28, 0xDA, 0xA1, 0xA1, 0x05, 0x00, 0x00, 0xF8};
    uint8 rom3[8] = {0x28, 0x35, 0xAC, 0x31, 0x03, 0x00, 0x00, 0x29};
    uint8 rom4[8] = {0x28, 0xBD, 0xA4, 0xA1, 0x05, 0x00, 0x00, 0x6C};
    uint8 rom5[8] = {0x28, 0xEB, 0x8E, 0xA1, 0x05, 0x00, 0x00, 0xB5};
    uint8 rom6[8] = {0x28, 0x37, 0xEC, 0x31, 0x03, 0x00, 0x00, 0xAE};
    uint8 *pRom = rom;
    switch (flagrom) {
        case 0:
            pRom = rom;
            break;
        case 1:
            pRom = rom1;
            break;
        case 2:
            pRom = rom2;
            break;
        case 3:
            pRom = rom3;
            break;
        case 4:
            pRom = rom4;
            break;
        case 5:
            pRom = rom5;
            break;
        case 6:
            pRom = rom6;
            break;
        default:
            pRom = rom;
            break;
    }
    DS18B20_Init();
    DS18B20_select(pRom);
    DS18B20_Write(0xBE, 0);
    mData[0] = DS18B20_Read();
    mData[1] = DS18B20_Read();
    //mData[2] = DS18B20_Read();
    //mData[3] = DS18B20_Read();
    //mData[4] = DS18B20_Read();
    unsigned char tem_h,tem_l, a, b, flag;
    tem_l = mData[0];
    tem_h = mData[1];
    uint8 sendbuffer[6];
    sendbuffer[0] = 0xAA;
    sendbuffer[1] = 0xBB;
    sendbuffer[2] = 0xEE;
    sendbuffer[3] = flag;
    sendbuffer[4] = tem_l;
    sendbuffer[5] = tem_h;
    eggSerialAppSendNoti(sendbuffer, 6);
#if 0
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

    float ft = sensor_data_value[1] + ((float)sensor_data_value[0])/10.0;
    mData[2] = sensor_data_value[1];
    mData[3] = sensor_data_value[0];
#endif
    DS18B20_Init();             //复位18B20
    DS18B20_Write(0xCC, 1);     //发出跳过ROM匹配操作
    DS18B20_Write(0x44, 1);     //启动温度转换
}

void readDs18b20WithState(uint8 state, uint8 flagrom)
{
    uint8 rom[8] = {0x28, 0x5C, 0x1F, 0x92, 0x04, 0x00, 0x00, 0x26};
    uint8 rom1[8] = {0x28, 0x12, 0x91, 0xA1, 0x05, 0x00, 0x00, 0x42};
    uint8 rom2[8] = {0x28, 0xDA, 0xA1, 0xA1, 0x05, 0x00, 0x00, 0xF8};
    uint8 rom3[8] = {0x28, 0x35, 0xAC, 0x31, 0x03, 0x00, 0x00, 0x29};
    uint8 rom4[8] = {0x28, 0xBD, 0xA4, 0xA1, 0x05, 0x00, 0x00, 0x6C};
    uint8 rom5[8] = {0x28, 0xEB, 0x8E, 0xA1, 0x05, 0x00, 0x00, 0xB5};
    uint8 rom6[8] = {0x28, 0x37, 0xEC, 0x31, 0x03, 0x00, 0x00, 0xAE};

    uint8 rom7[8] = {0x28, 0x5C, 0x1F, 0x92, 0x04, 0x00, 0x00, 0x26};
    uint8 rom8[8] = {0x28, 0x12, 0x91, 0xA1, 0x05, 0x00, 0x00, 0x42};
    uint8 rom9[8] = {0x28, 0xDA, 0xA1, 0xA1, 0x05, 0x00, 0x00, 0xF8};
    uint8 rom10[8] = {0x28, 0x35, 0xAC, 0x31, 0x03, 0x00, 0x00, 0x29};
    uint8 rom11[8] = {0x28, 0xBD, 0xA4, 0xA1, 0x05, 0x00, 0x00, 0x6C};
    uint8 rom12[8] = {0x28, 0xEB, 0x8E, 0xA1, 0x05, 0x00, 0x00, 0xB5};
    uint8 rom13[8] = {0x28, 0x37, 0xEC, 0x31, 0x03, 0x00, 0x00, 0xAE};
    uint8 *pRom = rom;
    switch (flagrom) {
        case 0:
            pRom = rom;
            break;
        case 1:
            pRom = rom1;
            break;
        case 2:
            pRom = rom2;
            break;
        case 3:
            pRom = rom3;
            break;
        case 4:
            pRom = rom4;
            break;
        case 5:
            pRom = rom5;
            break;
        case 6:
            pRom = rom6;
            break;

        case 7:
            pRom = rom7;
            break;
        case 8:
            pRom = rom8;
            break;
        case 9:
            pRom = rom9;
            break;
        case 10:
            pRom = rom10;
            break;
        case 11:
            pRom = rom11;
            break;
        case 12:
            pRom = rom12;
            break;
        case 13:
            pRom = rom13;
            break;
        default:
            pRom = rom;
            break;
    }
    if (state == 0) {
        #if 0
        DS18B20_Init();
        DS18B20_select(pRom);
        DS18B20_Write(0x44, 1); //Start conversion, with parasite power on at the end.
        #else
        OneWire_reset();
        OneWire_select(pRom);
        OneWire_write(0x44, 1);
        #endif
    }
    else {
        #if 0
        DS18B20_Init();
        DS18B20_select(pRom);
        DS18B20_Write(0xBE, 0); // Read scratchpad.
        uint8 tem_h,tem_l;
        tem_l = DS18B20_Read();
        tem_h = DS18B20_Read();
        #else
        OneWire_reset();
        OneWire_select(pRom);
        OneWire_write(0xBE, 0);
        uint8 tem_h,tem_l;
        tem_l = OneWire_read();
        tem_h = OneWire_read();
        #endif
        if (flagrom == 0)
        {
            gsendbufferI = 0;
            osal_memset(gsendbuffer, 0xFF, sizeof(gsendbuffer));
            gsendbuffer[gsendbufferI++] = 0xAA; // 0
            gsendbuffer[gsendbufferI++] = 0xBB;
            gsendbuffer[gsendbufferI++] = 0xBB; // 2
            gsendbuffer[gsendbufferI++] = tem_l;
            gsendbuffer[gsendbufferI++] = tem_h; // 4
        }
        else
        {
            gsendbuffer[gsendbufferI++] = tem_l;
            gsendbuffer[gsendbufferI++] = tem_h;
        }
        if (flagrom == 13)
        {
            gsendbuffer[gsendbufferI++] = 0x0D;
            gsendbuffer[gsendbufferI++] = 0x0A;
            eggSerialAppSendNoti(gsendbuffer, 17);
            //ST_HAL_DELAY(625);
            ST_HAL_DELAY(1000);
            eggSerialAppSendNoti(gsendbuffer+17, 16);
            //Ds18b20_SetParameter(SENSOR_DATA, DS18B20_DATA_LEN, gsendbuffer+gsendbufferI-4);
            //osal_set_event( sensorTag_TaskID, ST_DS18B20_CONTINUE_EVT);
        }
        else
        {
            //Ds18b20_SetParameter(SENSOR_DATA, DS18B20_DATA_LEN, gsendbuffer+gsendbufferI-2);
        }
    }
}

void readDs18b20WithState1(uint8 state, uint8 flagrom)
{
    uint8 rom[8] = {0x28, 0x5C, 0x1F, 0x92, 0x04, 0x00, 0x00, 0x26};
    uint8 rom1[8] = {0x28, 0x12, 0x91, 0xA1, 0x05, 0x00, 0x00, 0x42};
    uint8 rom2[8] = {0x28, 0xDA, 0xA1, 0xA1, 0x05, 0x00, 0x00, 0xF8};
    uint8 rom3[8] = {0x28, 0x35, 0xAC, 0x31, 0x03, 0x00, 0x00, 0x29};
    uint8 rom4[8] = {0x28, 0xBD, 0xA4, 0xA1, 0x05, 0x00, 0x00, 0x6C};
    uint8 rom5[8] = {0x28, 0xEB, 0x8E, 0xA1, 0x05, 0x00, 0x00, 0xB5};
    uint8 rom6[8] = {0x28, 0x37, 0xEC, 0x31, 0x03, 0x00, 0x00, 0xAE};

    uint8 rom7[8] = {0x28, 0x5C, 0x1F, 0x92, 0x04, 0x00, 0x00, 0x26};
    uint8 rom8[8] = {0x28, 0x12, 0x91, 0xA1, 0x05, 0x00, 0x00, 0x42};
    uint8 rom9[8] = {0x28, 0xDA, 0xA1, 0xA1, 0x05, 0x00, 0x00, 0xF8};
    uint8 rom10[8] = {0x28, 0x35, 0xAC, 0x31, 0x03, 0x00, 0x00, 0x29};
    uint8 rom11[8] = {0x28, 0xBD, 0xA4, 0xA1, 0x05, 0x00, 0x00, 0x6C};
    uint8 rom12[8] = {0x28, 0xEB, 0x8E, 0xA1, 0x05, 0x00, 0x00, 0xB5};
    uint8 rom13[8] = {0x28, 0x37, 0xEC, 0x31, 0x03, 0x00, 0x00, 0xAE};
    uint8 *pRom = rom;
    switch (flagrom) {
        case 0:
            pRom = rom;
            break;
        case 1:
            pRom = rom1;
            break;
        case 2:
            pRom = rom2;
            break;
        case 3:
            pRom = rom3;
            break;
        case 4:
            pRom = rom4;
            break;
        case 5:
            pRom = rom5;
            break;
        case 6:
            pRom = rom6;
            break;

        case 7:
            pRom = rom7;
            break;
        case 8:
            pRom = rom8;
            break;
        case 9:
            pRom = rom9;
            break;
        case 10:
            pRom = rom10;
            break;
        case 11:
            pRom = rom11;
            break;
        case 12:
            pRom = rom12;
            break;
        case 13:
            pRom = rom13;
            break;
        default:
            pRom = rom;
            break;
    }
    if (state == 0) {
        #if 0
        DS18B20_Init();
        DS18B20_select(pRom);
        DS18B20_Write(0x44, 1); //Start conversion, with parasite power on at the end.
        #else
        OneWire_reset();
        OneWire_select(pRom);
        OneWire_write(0x44, 1);
        #endif
    }
    else {
        #if 0
        DS18B20_Init();
        DS18B20_select(pRom);
        DS18B20_Write(0xBE, 0); // Read scratchpad.
        uint8 tem_h,tem_l;
        tem_l = DS18B20_Read();
        tem_h = DS18B20_Read();
        #else
        OneWire_reset();
        OneWire_select(pRom);
        OneWire_write(0xBE, 0);
        uint8 tem_h,tem_l;
        tem_l = OneWire_read();
        tem_h = OneWire_read();
        #endif
        gsendbuffer[0] = 0xAA; // 0
        gsendbuffer[1] = 0xBB;
        gsendbuffer[2] = 0xBB; // 2
        gsendbuffer[3] = tem_l;
        gsendbuffer[4] = tem_h; // 4
        gsendbuffer[5] = 0xFF;
        gsendbuffer[6] = 0xFF;
        eggSerialAppSendNoti(gsendbuffer, 7);
    }
}
/*********************************************************************
 * @fn      readMagData
 *
 * @brief   Read magnetometer data
 *
 * @param   none
 *
 * @return  none
 */
static void readMagData( void )
{
  uint8 mData[MAGNETOMETER_DATA_LEN];

  if (HalMagRead(mData))
  {
    Magnetometer_SetParameter(SENSOR_DATA, MAGNETOMETER_DATA_LEN, mData);
  }
}

/*********************************************************************
 * @fn      readHumData
 *
 * @brief   Read humidity data
 *
 * @param   none
 *
 * @return  none
 */
static void readHumData(void)
{
  uint8 hData[HUMIDITY_DATA_LEN];
  uint8 buffers[7];
  if (HalHumiReadMeasurement(hData))
  {
    //Humidity_SetParameter( SENSOR_DATA, HUMIDITY_DATA_LEN, hData);

    buffers[0] = 0xAA;
    buffers[1] = 0xBB;
    buffers[2] = 0xCC;
    buffers[3] = hData[2];
    buffers[4] = hData[3];
    buffers[5] = 0x0D;
    buffers[6] = 0x0A;
    eggSerialAppSendNoti(buffers, 7);
  }
  else{
    buffers[0] = 0xAA;
    buffers[1] = 0xBB;
    buffers[2] = 0xCC;
    buffers[3] = 0x76;
    buffers[4] = 0x76;
    buffers[5] = 0x0D;
    buffers[6] = 0x0A;
    eggSerialAppSendNoti(buffers, 7);
  }
}

/*********************************************************************
 * @fn      readBarData
 *
 * @brief   Read barometer data
 *
 * @param   none
 *
 * @return  none
 */
static void readBarData( void )
{
  uint8 bData[BAROMETER_DATA_LEN];

  if (HalBarReadMeasurement(bData))
  {
    Barometer_SetParameter( SENSOR_DATA, BAROMETER_DATA_LEN, bData);
  }
}

/*********************************************************************
 * @fn      readBarCalibration
 *
 * @brief   Read barometer calibration
 *
 * @param   none
 *
 * @return  none
 */
static void readBarCalibration( void )
{
  uint8* cData = osal_mem_alloc(BAROMETER_CALI_LEN);

  if (cData != NULL )
  {
    HalBarReadCalibration(cData);
    Barometer_SetParameter( SENSOR_CALB, BAROMETER_CALI_LEN, cData);
    osal_mem_free(cData);
  }
}

/*********************************************************************
 * @fn      readIrTempData
 *
 * @brief   Read IR temperature data
 *
 * @param   none
 *
 * @return  none
 */
static void readIrTempData( void )
{
  uint8 tData[IRTEMPERATURE_DATA_LEN];

  if (HalIRTempRead(tData))
  {
    IRTemp_SetParameter( SENSOR_DATA, IRTEMPERATURE_DATA_LEN, tData);
  }
}

static void readIrTempDataAdv( void )
{
  uint8 tData[IRTEMPERATURE_DATA_LEN];

  if (HalIRTempRead(tData))
  {
    //IRTemp_SetParameter( SENSOR_DATA, IRTEMPERATURE_DATA_LEN, tData);

    advertData[7] = tData[0];
    advertData[8] = tData[1];
    advertData[9] = tData[2];
    advertData[10] = tData[3];
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
  }
}

/*********************************************************************
 * @fn      readGyroData
 *
 * @brief   Read gyroscope data
 *
 * @param   none
 *
 * @return  none
 */
static void readGyroData( void )
{
  uint8 gData[GYROSCOPE_DATA_LEN];

  if (HalGyroRead(gData))
  {
    Gyro_SetParameter( SENSOR_DATA, GYROSCOPE_DATA_LEN, gData);
  }
}

/*********************************************************************
 * @fn      barometerChangeCB
 *
 * @brief   Callback from Barometer Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void barometerChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch( paramID )
  {
    case SENSOR_CONF:
      Barometer_GetParameter( SENSOR_CONF, &newValue );

      switch ( newValue)
      {
      case ST_CFG_SENSOR_DISABLE:
        if (barEnabled)
        {
          barEnabled = FALSE;
          osal_set_event( sensorTag_TaskID, ST_BAROMETER_SENSOR_EVT);
        }
        break;

      case ST_CFG_SENSOR_ENABLE:
        if(!barEnabled)
        {
          barEnabled = TRUE;
          osal_set_event( sensorTag_TaskID, ST_BAROMETER_SENSOR_EVT);
        }
        break;

      case ST_CFG_CALIBRATE:
        readBarCalibration();
        break;

      default:
        break;
      }
      break;

  case SENSOR_PERI:
      Barometer_GetParameter( SENSOR_PERI, &newValue );
      sensorBarPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // should not get here!
      break;
  }
}

/*********************************************************************
 * @fn      irTempChangeCB
 *
 * @brief   Callback from IR Temperature Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void irTempChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID) {
  case SENSOR_CONF:
    IRTemp_GetParameter( SENSOR_CONF, &newValue );

    if ( newValue == ST_CFG_SENSOR_DISABLE)
    {
      // Put sensor to sleep
      if (irTempEnabled)
      {
        irTempEnabled = FALSE;
        osal_set_event( sensorTag_TaskID, ST_IRTEMPERATURE_READ_EVT);
      }
    }
    else if (newValue == ST_CFG_SENSOR_ENABLE)
    {
      if (!irTempEnabled)
      {
        irTempEnabled = TRUE;
        osal_set_event( sensorTag_TaskID,ST_IRTEMPERATURE_READ_EVT);
      }
    }
    break;

  case SENSOR_PERI:
    IRTemp_GetParameter( SENSOR_PERI, &newValue );
    sensorTmpPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
    break;

  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      accelChangeCB
 *
 * @brief   Callback from Acceleromter Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void accelChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Accel_GetParameter( SENSOR_CONF, &newValue );
      if ( newValue == ST_CFG_SENSOR_DISABLE)
      {
        // Put sensor to sleep
        if (accConfig != ST_CFG_SENSOR_DISABLE)
        {
          accConfig = ST_CFG_SENSOR_DISABLE;
          osal_set_event( sensorTag_TaskID, ST_ACCELEROMETER_SENSOR_EVT);
        }
      }
      else
      {
        if (accConfig == ST_CFG_SENSOR_DISABLE)
        {
          // Start scheduling only on change disabled -> enabled
          osal_set_event( sensorTag_TaskID, ST_ACCELEROMETER_SENSOR_EVT);
        }
        // Scheduled already, so just change range
        accConfig = newValue;
        HalAccSetRange(accConfig);
      }
      break;

    case SENSOR_PERI:
      Accel_GetParameter( SENSOR_PERI, &newValue );
      sensorAccPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // Should not get here
      break;
  }
}

static void mpu6050StarWhenConnected(void)
{
    if (mpu6050Config == ST_CFG_SENSOR_DISABLE)
    {
      // Start scheduling only on change disabled -> enabled
      //osal_set_event( sensorTag_TaskID, ST_MPU6050_SENSOR_EVT);
      //osal_start_timerEx( sensorTag_TaskID, ST_MPU6050_SENSOR_EVT, sensorMpu6050Period );
      // Scheduled already, so just change range
      mpu6050Config = ST_CFG_SENSOR_ENABLE;
      HalMPU6050initialize();

      osal_start_timerEx( sensorTag_TaskID, ST_MPU6050_DMP_INIT_EVT, 10000 );
    }
}

static void mpu6050ChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Mpu6050_GetParameter( SENSOR_CONF, &newValue );
      if ( newValue == ST_CFG_SENSOR_DISABLE)
      {
        // Put sensor to sleep
        if (mpu6050Config != ST_CFG_SENSOR_DISABLE)
        {
          mpu6050Config = ST_CFG_SENSOR_DISABLE;
          osal_set_event( sensorTag_TaskID, ST_MPU6050_SENSOR_EVT);
        }
      }
      else
      {
        if (mpu6050Config == ST_CFG_SENSOR_DISABLE)
        {
          // Start scheduling only on change disabled -> enabled
          osal_set_event( sensorTag_TaskID, ST_MPU6050_SENSOR_EVT);
        }
        // Scheduled already, so just change range
        mpu6050Config = newValue;
        HalMPU6050initialize();
      }
      break;

    case SENSOR_PERI:
      Mpu6050_GetParameter( SENSOR_PERI, &sensorMpu6050Period );
      break;

    default:
      // Should not get here
      break;
  }
}

static void lm75aStarWhenConnected(void)
{
    osal_start_timerEx( sensorTag_TaskID, ST_LM75A_SENSOR_EVT, 3999 );
}

static void ds18b20StarWhenConnected(void)
{
    if (!ds18b20Enabled)
    {
        ds18b20State = 0;
        ds18b20Enabled = TRUE;
        flagRom = 0;
        gsendbufferI = 0;
        //osal_set_event( sensorTag_TaskID, ST_DS18B20_SENSOR_EVT);
        osal_start_timerEx( sensorTag_TaskID, ST_DS18B20_SENSOR_EVT, 5000 );
    }
}

static void ds18b20ChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Ds18b20_GetParameter( SENSOR_CONF, &newValue );
      if ( newValue == ST_CFG_SENSOR_DISABLE )
      {
        if(ds18b20Enabled)
        {
          ds18b20Enabled = FALSE;
          osal_set_event( sensorTag_TaskID, ST_DS18B20_SENSOR_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!ds18b20Enabled)
        {
          ds18b20Enabled = TRUE;
          osal_set_event( sensorTag_TaskID, ST_DS18B20_SENSOR_EVT);
          //LCD_WRITE_STRING( "Let start DS18B20...", HAL_LCD_LINE_1 );
          //OneWire_reset_search();
        }
      }
      break;

    case SENSOR_PERI:
      Ds18b20_GetParameter( SENSOR_PERI, &sensorDs18b20Period );
      break;

    default:
      // Should not get here
      break;
  }
}

/*********************************************************************
 * @fn      magnetometerChangeCB
 *
 * @brief   Callback from Magnetometer Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void magnetometerChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Magnetometer_GetParameter( SENSOR_CONF, &newValue );

      if ( newValue == ST_CFG_SENSOR_DISABLE )
      {
        if(magEnabled)
        {
          magEnabled = FALSE;
          osal_set_event( sensorTag_TaskID, ST_MAGNETOMETER_SENSOR_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!magEnabled)
        {
          magEnabled = TRUE;
          osal_set_event( sensorTag_TaskID, ST_MAGNETOMETER_SENSOR_EVT);
        }
      }
      break;

    case SENSOR_PERI:
      Magnetometer_GetParameter( SENSOR_PERI, &newValue );
      sensorMagPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // Should not get here
      break;
  }
}

static void humidityStarWhenConnected(void)
{
  if (!humiEnabled)
  {
    humiEnabled = TRUE;
    humiState = 0;
    osal_set_event( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT);
  }
}
/*********************************************************************
 * @fn      humidityChangeCB
 *
 * @brief   Callback from Humidity Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void humidityChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch ( paramID)
  {
  case  SENSOR_CONF:
    Humidity_GetParameter( SENSOR_CONF, &newValue );

    if ( newValue == ST_CFG_SENSOR_DISABLE)
    {
      if (humiEnabled)
      {
        humiEnabled = FALSE;
        osal_set_event( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT);
      }
    }

    if ( newValue == ST_CFG_SENSOR_ENABLE )
    {
      if (!humiEnabled)
      {
        humiEnabled = TRUE;
        humiState = 0;
        osal_set_event( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT);
      }
    }
    break;

  case SENSOR_PERI:
    Humidity_GetParameter( SENSOR_PERI, &sensorHumPeriod );
    break;

  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      gyroChangeCB
 *
 * @brief   Callback from GyroProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void gyroChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID) {
  case SENSOR_CONF:
    Gyro_GetParameter( SENSOR_CONF, &newValue );

    if (newValue == 0)
    {
      // All three axes off, put sensor to sleep
      if (gyroEnabled)
      {
        gyroEnabled = FALSE;
        osal_set_event( sensorTag_TaskID, ST_GYROSCOPE_SENSOR_EVT);
      }
    }
    else
    {
      // Bitmap tells which axis to enable (bit 0: X, but 1: Y, but 2: Z)
      gyroEnabled = TRUE;
      sensorGyroAxes = newValue & 0x07;
      sensorGyroUpdateAxes = TRUE;
      osal_set_event( sensorTag_TaskID,  ST_GYROSCOPE_SENSOR_EVT);
    }
    break;

  case SENSOR_PERI:
    Gyro_GetParameter( SENSOR_PERI, &newValue );
    sensorGyrPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
    break;

  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      testChangeCB
 *
 * @brief   Callback from Test indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void testChangeCB( uint8 paramID )
{
  if( paramID == TEST_CONF_ATTR )
  {
    uint8 newValue;

    Test_GetParameter( TEST_CONF_ATTR, &newValue );

    if (newValue & TEST_MODE_ENABLE)
    {
      testMode = TRUE;
    }
    else
    {
      testMode = FALSE;
    }

    if (testMode)
    {
      // Test mode: possible to operate LEDs. Key hits will cause notifications,
      // side key does not influence connection state
      if (newValue & 0x01)
      {
        HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);
      }
      else
      {
        HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);
      }

      if (newValue & 0x02)
      {
        HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);
      }
      else
      {
        HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
      }
    }
    else
    {
      // Normal mode; make sure LEDs are reset and attribute cleared
      HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);
      HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
      newValue = 0x00;
      Test_SetParameter( TEST_CONF_ATTR, 1, &newValue );
    }
  }
}

/*********************************************************************
 * @fn      ccChangeCB
 *
 * @brief   Callback from Connection Control indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void ccChangeCB( uint8 paramID )
{

  // CCSERVICE_CHAR1: read & notify only

  // CCSERVICE_CHAR: requested connection parameters
  if( paramID == CCSERVICE_CHAR2 )
  {
    uint8 buf[CCSERVICE_CHAR2_LEN];
    uint16 minConnInterval;
    uint16 maxConnInterval;
    uint16 slaveLatency;
    uint16 timeoutMultiplier;

    CcService_GetParameter( CCSERVICE_CHAR2, buf );

    minConnInterval = BUILD_UINT16(buf[0],buf[1]);
    maxConnInterval = BUILD_UINT16(buf[2],buf[3]);
    slaveLatency = BUILD_UINT16(buf[4],buf[5]);
    timeoutMultiplier = BUILD_UINT16(buf[6],buf[7]);

    // Update connection parameters
    GAPRole_SendUpdateParam( minConnInterval, maxConnInterval, slaveLatency, timeoutMultiplier, GAPROLE_TERMINATE_LINK);
  }

  // CCSERVICE_CHAR3: Disconnect request
  if( paramID == CCSERVICE_CHAR3 )
  {
    // Any change in the value will terminate the connection
    GAPRole_TerminateConnection();
  }
}


/*********************************************************************
 * @fn      gapRolesParamUpdateCB
 *
 * @brief   Called when connection parameters are updates
 *
 * @param   connInterval - new connection interval
 *
 * @param   connSlaveLatency - new slave latency
 *
 * @param   connTimeout - new connection timeout
 *
 * @return  none
*/
static void gapRolesParamUpdateCB( uint16 connInterval, uint16 connSlaveLatency,uint16 connTimeout )
{
  uint8 buf[CCSERVICE_CHAR1_LEN];

  buf[0] = LO_UINT16(connInterval);
  buf[1] = HI_UINT16(connInterval);
  buf[2] = LO_UINT16(connSlaveLatency);
  buf[3] = HI_UINT16(connSlaveLatency);
  buf[4] = LO_UINT16(connTimeout);
  buf[5] = HI_UINT16(connTimeout);
  CcService_SetParameter(CCSERVICE_CHAR1,sizeof(buf),buf);
}


/*********************************************************************
 * @fn      resetCharacteristicValue
 *
 * @brief   Initialize a characteristic value to zero
 *
 * @param   servID - service ID (UUID)
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   vakue - value to initialise with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
static void resetCharacteristicValue(uint16 servUuid, uint8 paramID, uint8 value, uint8 paramLen)
{
  uint8* pData = osal_mem_alloc(paramLen);

  if (pData == NULL)
  {
    return;
  }

  osal_memset(pData,value,paramLen);

  switch(servUuid)
  {
    case IRTEMPERATURE_SERV_UUID:
      IRTemp_SetParameter( paramID, paramLen, pData);
      break;

    case ACCELEROMETER_SERV_UUID:
      Accel_SetParameter( paramID, paramLen, pData);
      break;

    case MAGNETOMETER_SERV_UUID:
      Magnetometer_SetParameter( paramID, paramLen, pData);
      break;

    case HUMIDITY_SERV_UUID:
      Humidity_SetParameter( paramID, paramLen, pData);
      break;

    case BAROMETER_SERV_UUID:
      Barometer_SetParameter( paramID, paramLen, pData);
      break;

    case GYROSCOPE_SERV_UUID:
      Gyro_SetParameter( paramID, paramLen, pData);
      break;

    default:
      // Should not get here
      break;
  }

  osal_mem_free(pData);
}

/*********************************************************************
 * @fn      resetCharacteristicValues
 *
 * @brief   Initialize all the characteristic values
 *
 * @return  none
 */
static void resetCharacteristicValues( void )
{
  resetCharacteristicValue( IRTEMPERATURE_SERV_UUID, SENSOR_DATA, 0, IRTEMPERATURE_DATA_LEN);
  resetCharacteristicValue( IRTEMPERATURE_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( IRTEMPERATURE_SERV_UUID, SENSOR_PERI, TEMP_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_DATA, 0, ACCELEROMETER_DATA_LEN );
  resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_PERI, ACC_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( HUMIDITY_SERV_UUID, SENSOR_DATA, 0, HUMIDITY_DATA_LEN);
  resetCharacteristicValue( HUMIDITY_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( HUMIDITY_SERV_UUID, SENSOR_PERI, HUM_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( MAGNETOMETER_SERV_UUID, SENSOR_DATA, 0, MAGNETOMETER_DATA_LEN);
  resetCharacteristicValue( MAGNETOMETER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( MAGNETOMETER_SERV_UUID, SENSOR_PERI, MAG_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( BAROMETER_SERV_UUID, SENSOR_DATA, 0, BAROMETER_DATA_LEN);
  resetCharacteristicValue( BAROMETER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( BAROMETER_SERV_UUID, SENSOR_PERI, BAR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_DATA, 0, GYROSCOPE_DATA_LEN);
  resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof( uint8 ));
  resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_PERI, GYRO_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
}


/*********************************************************************
*********************************************************************/
static void eggSerialAppSendNoti(uint8 *pBuffer,uint16 length)
{
  uint8 len;
  if (length > 20)
  {
    len = 20;
  }
  else
  {
    len = length;
  }
  static attHandleValueNoti_t pReport;
  pReport.handle=0x2E;
  pReport.len = len;
  osal_memcpy(pReport.value, pBuffer, len);
  GATT_Notification( 0, &pReport, FALSE );
}

