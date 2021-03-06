/*
 * Core Source for:
 * CY8CTMA4XX
 *
 * Copyright (C) 2012 Pantech, Inc.
 * 
 * used by touch_monitor, touch_fops
 */

typedef enum {
	APPLY_TOUCH_CONFIG = 501,
	DIAG_DEBUG = 502,
	RESET_TOUCH_CONFIG = 503,
	GET_TOUCH_CONFIG = 504,
	SET_TOUCH_CONFIG = 505,
	READ_ITO_TYPE = 506,
	GET_REFERENCE_DATA = 507,
	LOAD_PARAMETER_TABLE = 508,
    RESET_TOUCH_PARAMETER = 509,
	
	// functions for PST 
	PST_FAIL_COUNT = 601,
	PST_RESET1 = 602,
	PST_RESET2 = 603,
	PST_RESET3 = 604,

	TOUCH_IOCTL_READ_LASTKEY=1001,	
	TOUCH_IOCTL_DO_KEY,	
	TOUCH_IOCTL_RELEASE_KEY, 
	TOUCH_IOCTL_CLEAN,
    TOUCH_IOCTL_DEBUG,
	TOUCH_IOCTL_RESTART,
	TOUCH_IOCTL_PRESS_TOUCH,
	TOUCH_IOCTL_RELEASE_TOUCH,
	TOUCH_IOCTL_CHARGER_MODE,
	TOUCH_IOCTL_POWER_OFF,
	TOUCH_IOCTL_STYLUS_MODE,
    TOUCH_IOCTL_CALIBRATE,

	TOUCH_CALL_MODE_ENABLE = 1013,
	TOUCH_CALL_MODE_DISABLE = 1014,
	
	TOUCH_IOCTL_DELETE_ACTAREA = 2001,
	TOUCH_IOCTL_RECOVERY_ACTAREA,

	TOUCH_IOCTL_SENSOR_X = 2005,
	TOUCH_IOCTL_SENSOR_Y,
	TOUCH_IOCTL_CHECK_BASE,
	TOUCH_IOCTL_READ_IC_VERSION,
	TOUCH_IOCTL_READ_FW_VERSION,	
	TOUCH_IOCTL_START_UPDATE,
	TOUCH_IOCTL_SELF_TEST,

	TOUCH_IOCTL_INIT = 3001,	
	TOUCH_IOCTL_OFF  = 3002,

	TOUCH_CHARGE_MODE_CTL = 4001,
    
    TOUCH_IOCTL_DIAG_DEBUG_DELTA = 5010,
    TOUCH_IOCTL_DIAG_DEBUG_REF,
    TOUCH_IOCTL_DIAG_DEBUG_BASELINE,
    TOUCH_IOCTL_DIAG_DEBUG_OPERATEMODE,

	TOUCH_IOCTL_GET_CHARGER_MODE	= 6001,
	TOUCH_IOCTL_GET_TOUCH_ERR		= 6002,
	
} TOUCH_IOCTL_CMD;


