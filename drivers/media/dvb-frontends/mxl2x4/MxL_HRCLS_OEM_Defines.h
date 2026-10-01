#ifndef __MXL_HRCLS_OEM_DEFINES_H__
#define __MXL_HRCLS_OEM_DEFINES_H__

#include <linux/printk.h>
#include <linux/string.h>

#define MXL_HRCLS_OEM_MAX_BLOCK_WRITE_LENGTH   256
#define MXL_HRCLS_OEM_MAX_BLOCK_READ_LENGTH    800

#define MXL_MODULE_DEBUG_LEVEL 0
#define MXL_MODULE_DEBUG_OPTIONS MXLDBG_ERROR
#define MXL_MODULE_DEBUG_FCT pr_debug

#define MxL_HRCLS_DEBUG   pr_debug
#define MxL_HRCLS_ERROR   pr_err
#define MxL_HRCLS_PRINT   pr_debug

#define not_MXL_HRCLS_WAKE_ON_WAN_ENABLED_
#define _MXL_HRCLS_LITTLE_ENDIAN_

#define	MXL254_I2C_INDEX	1

#endif /* __MXL_HRCLS_OEM_DEFINES_H__ */
