/*
 * modsd.h
 *
 *  Created on: 2024Äê4ÔÂ6ÈÕ
 *      Author: lenovo
 */

#ifndef SRC_MODULES_MODSD_H_
#define SRC_MODULES_MODSD_H_

#include "../support/os_framework.h"
#include    "tx_api.h"
void sd_write_init(TX_BYTE_POOL *pmem);

extern OS_U8 WriteToSD(OS_U8 fileIndex, OS_U8 *buf, OS_U32 length);
void MountSD();
extern OS_U8 InitSD();

extern unsigned char SD_MountOK;
extern unsigned char SD_Enable;
#endif /* SRC_MODULES_MODSD_H_ */
