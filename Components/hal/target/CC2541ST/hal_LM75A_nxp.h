/**************************************************************************************************
  Filename:       hal_LM75A_nxp.h
  Revised:        $Date: 2014-10-29 14:06:46 +0800 (Wed, 29 Oct 2014) $
  Revision:       $Revision: 1 $

  Description:    Interface to the LM75A temperature sensor driver
**************************************************************************************************/

#ifndef HAL_LM75A_NXP_H
#define HAL_LM75A_NXP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "comdef.h"

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * TYPEDEFS
 */
typedef enum
{
  LM75A_OFF,               // LM75A Temperature Sleeping
  LM75A_IDLE,			   // LM75A Temperature On and Configured
  LM75A_DATA_READY         // LM75A Temperature On, Configured and Data is Ready	
} LM75ATemperature_States_t;


/*********************************************************************
 * FUNCTIONS
 */
void HALLM75ATempInit(void);
void HalLM75ATempTurnOn(void);
void HalLM75ATempTurnOff(void);
bool HalLM75ATempRead(uint8 *lm75aTempData);
bool HalLM75ATempTest(void);
IRTemperature_States_t HalLM75ATempStatus(void);


#ifdef __cplusplus
}
#endif

#endif /* HAL_IRTEMP_H */
