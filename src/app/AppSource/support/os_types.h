/*
 * types.h
 *
 *  Created on: 2022Äê8ÔÂ7ÈÕ
 *      Author: Lenovo
 */

#ifndef SUPPORT_OS_TYPES_H_
#define SUPPORT_OS_TYPES_H_

typedef unsigned char		OS_U8;
typedef signed char			OS_S8;
typedef unsigned short		OS_U16;
typedef signed short		OS_S16;
typedef unsigned int		OS_U32;
typedef signed int			OS_S32;
typedef unsigned long long 	OS_U64;
typedef signed long long 	OS_S64;
typedef float				OS_FLOAT;
typedef double				OS_DOUBLE;
typedef void				OS_VOID;
typedef OS_U8				OS_MEM;
typedef unsigned char		OS_BOOL;



#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

#ifndef NULL
#define NULL (0)
#endif

#ifndef PTR_NULL
#define PTR_NULL ((void *)0)
#endif

#ifndef BOOL
#define BOOL OS_U8
#endif

#define OS_TRUE			(1)
#define OS_FALSE		(0)

#define OS_SUCCESS		(0)
#define OS_FAILURE		(-1)

#endif /* SUPPORT_OS_TYPES_H_ */
