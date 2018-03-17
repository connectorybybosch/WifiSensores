/*
 * XDKAppInfo.h
 *
 *  Created on: Mar 15, 2018
 *      Author: TCA1MX
 */

#ifndef SOURCE_XDKAPPINFO_H_
#define SOURCE_XDKAPPINFO_H_

/* own header files*/
#include "XdkCommonInfo.h"

#define TASK_PRIO_MAIN_CMD_PROCESSOR                (UINT32_C(1))
#define TASK_STACK_SIZE_MAIN_CMD_PROCESSOR          (UINT16_C(700))
#define TASK_Q_LEN_MAIN_CMD_PROCESSOR                (UINT32_C(10))

/**
 * @brief BCDS_APP_MODULE_ID for Application C module of XDK
 * @info  usage:
 *      #undef BCDS_APP_MODULE_ID
 *      #define BCDS_APP_MODULE_ID BCDS_APP_MODULE_ID_xxx
 */
enum XDK_App_ModuleID_E
{
    XDK_APP_MODULE_ID_MAIN= XDK_COMMON_ID_OVERFLOW,
    XDK_APP_MODULE_ID_HTTP_EXAMPLE_CLIENT
/* Define next module ID here and assign a value to it! */
};



#endif /* SOURCE_XDKAPPINFO_H_ */
