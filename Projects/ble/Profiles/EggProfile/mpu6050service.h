/**************************************************************************************************
  Filename:       mpu6050service.h
  Revised:        $Date: 2014-06-30 22:50:31 +0800 (Mon, 30 Jue 2014) $
  Revision:       $Revision: 35100 $

  Description:    mpu6050 service definitions and prototypes

**************************************************************************************************/

#ifndef MPU6050SERVICE_H
#define MPU6050SERVICE_H

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
#define MPU6050_SERV_UUID         0xAB00  // F000AB00-0451-4000-B000-00000000-0000
#define MPU6050_DATA_UUID         0xAB01
#define MPU6050_CONF_UUID         0xAB02
#define MPU6050_PERI_UUID         0xAB03

// Sensor Profile Services bit fields
#define MPU6050_SERVICE           0x00000002

// Length of sensor data in bytes
#define MPU6050_DATA_LEN          12

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
 * Mpu6050_AddService- Initializes the Sensor GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
extern bStatus_t Mpu6050_AddService( uint32 services );

/*
 * Mpu6050_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Mpu6050_RegisterAppCBs( sensorCBs_t *appCallbacks );

/*
 * Mpu6050_SetParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Mpu6050_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Mpu6050_GetParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Mpu6050_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MPU6050SERVICE_H */

