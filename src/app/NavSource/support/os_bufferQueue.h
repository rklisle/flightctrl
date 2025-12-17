#ifndef _OS_BUFFER_H_
#define _OS_BUFFER_H_

#include "os_framework.h"

#define BUFFER_NO_HAVE_DATA			(0)
#define BUFFER_HAVE_DATA			(1)

OS_S32 BufferInit();

OS_S32 PushBuffer(const OS_MEM* pmData, const OS_U16 u16Length);

OS_MEM* PopBuffer(OS_U16* pu16Length);

#endif
