/*****************************************************************************************
 *
 * FILE NAME          : MaxLinearDataTypes.h
 *
 * AUTHOR             : Mariusz Murawski
 * DATE CREATED       : Nov/30, 2011
 *
 * DESCRIPTION        : This file contains MaxLinear-defined data types.
 *                      Instead of using ANSI C data type directly in source code
 *                      All module should include this header file.
 *                      And conditional compilation switch is also declared
 *                      here.
 *
 *****************************************************************************************
 *                Copyright (c) 2006, MaxLinear, Inc.
 ****************************************************************************************/

#ifndef __MAXLINEAR_DATA_TYPES_H__
#define __MAXLINEAR_DATA_TYPES_H__

/*****************************************************************************************
    Include Header Files
    (No absolute paths - paths handled by make file)
*****************************************************************************************/
#ifndef _MSC_VER
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/types.h>

#else
#include <stdint.h>
#endif
#endif

/*****************************************************************************************
    Macros
*****************************************************************************************/

/*****************************************************************************************
    User-Defined Types (Typedefs)
*****************************************************************************************/

/* Basic datatypes are defined below.
 * If you do not wish to use them, or you already have
 * them defined, please add MXL_DO_NOT_DEFINE_BASIC_DATATYPES
 * to compilation defines
 */
#ifndef MXL_DO_NOT_DEFINE_BASIC_DATATYPES
#ifdef _MSC_VER
#ifndef __midl
typedef unsigned __int8     UINT8;
typedef unsigned __int16    UINT16;
typedef unsigned __int32    UINT32;
typedef unsigned __int64    UINT64;

typedef __int8              SINT8;
typedef __int16             SINT16;
#else
typedef small               SINT8;
typedef short               SINT16;
#endif

typedef __int32             SINT32;
typedef __int64             SINT64;
#else
typedef uint8_t             UINT8;
typedef uint16_t            UINT16;
typedef uint32_t            UINT32;
typedef uint64_t            UINT64;

typedef int8_t              SINT8;
typedef int16_t             SINT16;
typedef int32_t             SINT32;
typedef int64_t             SINT64;
#endif

typedef float               REAL32;
typedef double              REAL64;
#endif

#define MXL_MIN(x, y)                   (((x) < (y)) ? (x) : (y))
#define MXL_MAX(x, y)                   (((x) >= (y)) ? (x) : (y))
#define MXL_ABS(x)                      (((x) >= 0) ? (x) : -(x))

static inline UINT64 mxl_div_u64(UINT64 dividend, UINT32 divisor)
{
	return divisor ? div_u64(dividend, divisor) : 0;
}

static inline SINT64 mxl_div_s64(SINT64 dividend, SINT32 divisor)
{
	return divisor ? div_s64(dividend, divisor) : 0;
}

static inline UINT64 mxl_div_round_u64(UINT64 dividend, UINT32 divisor)
{
	return divisor ? div_u64(dividend + divisor / 2, divisor) : 0;
}

static inline SINT64 mxl_div_round_s64(SINT64 dividend, SINT32 divisor)
{
	SINT64 half;

	if (!divisor)
		return 0;

	half = divisor / 2;
	if (dividend < 0)
		half = -half;

	return div_s64(dividend + half, divisor);
}

#define u64mod(x, y) mxl_div_u64((UINT64)(x), (UINT32)(y))
#define s64mod(x, y) mxl_div_s64((SINT64)(x), (SINT32)(y))
#define MXL_DIV_ROUND(x, y) \
	mxl_div_round_u64((UINT64)(x), (UINT32)(y))
#define MXL_DIV_ROUND_S(x, y) \
	mxl_div_round_s64((SINT64)(x), (SINT32)(y))

#if (((!defined __MXL_WIN__) || (defined _WIN_CLI_)) && (!defined _KEEP_STDCALL_))
#undef  __stdcall
#define __stdcall
#endif // (((!defined __MXL_WIN__) || (defined _WIN_CLI_)) && (!defined _KEEP_STDCALL_))

typedef UINT32 (__stdcall *MXL_CALLBACK_FN_T)(UINT8 devId, UINT32 callbackType, void * callbackPayload);

typedef enum
{
  MXL_SUCCESS = 0,
  MXL_FAILURE = 1,
  MXL_INVALID_PARAMETER,
  MXL_NOT_INITIALIZED,
  MXL_ALREADY_INITIALIZED,
  MXL_NOT_SUPPORTED,
  MXL_NOT_READY,
  MXL_DATA_ERROR
} MXL_STATUS_E;

typedef enum
{
  MXL_DISABLE = 0,
  MXL_ENABLE  = 1,

  MXL_FALSE = 0,
  MXL_TRUE  = 1,

  MXL_INVALID = 0,
  MXL_VALID   = 1,

  MXL_NO      = 0,
  MXL_YES     = 1,

  MXL_OFF     = 0,
  MXL_ON      = 1
} MXL_BOOL_E;

// sample oemData type. Create your own
typedef struct
{
  UINT32         i2cAddress;
  UINT32         i2cIndex;
} oem_data_t;

/*****************************************************************************************
    Global Variable Declarations
*****************************************************************************************/

/*****************************************************************************************
    Prototypes
*****************************************************************************************/

#endif /* __MAXLINEAR_DATA_TYPES_H__ */
