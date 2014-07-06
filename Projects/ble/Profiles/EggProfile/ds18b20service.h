/**************************************************************************************************
  Filename:       ds18b20service.h
  Revised:        $Date: 2014-07-06 12:50:31 +0800 (Sun, 6 Jul 2014) $
  Revision:       $Revision: 35100 $

  Description:    mpu6050 service definitions and prototypes

**************************************************************************************************/

#ifndef DS18B20SERVICE_H
#define DS18B20SERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "st_util.h"

/*********************************************************************
 * CONSTANTS
 */

// Service UUID
#define DS18B20_SERV_UUID         0xAA00  // F000AB00-0451-4000-B000-00000000-0000
#define DS18B20_DATA_UUID         0xAA01
#define DS18B20_CONF_UUID         0xAA02
#define DS18B20_PERI_UUID         0xAA03

// Sensor Profile Services bit fields
#define DS18B20_SERVICE           0x00000002

// Length of sensor data in bytes
#define DS18B20_DATA_LEN          2

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * API FUNCTIONS
 */


/*
 * Ds18b20_AddService- Initializes the Sensor GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
extern bStatus_t Ds18b20_AddService( uint32 services );

/*
 * Ds18b20_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Ds18b20_RegisterAppCBs( sensorCBs_t *appCallbacks );

/*
 * Ds18b20_SetParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Ds18b20_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Ds18b20_GetParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Ds18b20_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* DS18B20SERVICE_H */

