// definitions yoinked from adafruit bno055 library <3

/* PAGE0 REGISTER DEFINITION START*/
#define BNO_CHIP_ID_ADDR                0x00
#define BNO_ACC_REV_ID_ADDR             0x01
#define BNO_MAG_REV_ID_ADDR             0x02
#define BNO_GYR_REV_ID_ADDR             0x03
#define BNO_SW_REV_ID_LSB_ADDR			0x04
#define BNO_SW_REV_ID_MSB_ADDR			0x05
#define BNO_BL_Rev_ID_ADDR			    0X06
#define BNO_Page_ID_ADDR			    0X07

/* Accel data register*/
#define BNO_ACC_DATA_X_LSB_ADDR			0X08
#define BNO_ACC_DATA_X_MSB_ADDR			0X09
#define BNO_ACC_DATA_Y_LSB_ADDR			0X0A
#define BNO_ACC_DATA_Y_MSB_ADDR			0X0B
#define BNO_ACC_DATA_Z_LSB_ADDR			0X0C
#define BNO_ACC_DATA_Z_MSB_ADDR			0X0D

/*Mag data register*/
#define BNO_MAG_DATA_X_LSB_ADDR			0X0E
#define BNO_MAG_DATA_X_MSB_ADDR			0X0F
#define BNO_MAG_DATA_Y_LSB_ADDR			0X10
#define BNO_MAG_DATA_Y_MSB_ADDR			0X11
#define BNO_MAG_DATA_Z_LSB_ADDR			0X12
#define BNO_MAG_DATA_Z_MSB_ADDR			0X13

/*Gyro data registers*/
#define BNO_GYR_DATA_X_LSB_ADDR			0X14
#define BNO_GYR_DATA_X_MSB_ADDR			0X15
#define BNO_GYR_DATA_Y_LSB_ADDR			0X16
#define BNO_GYR_DATA_Y_MSB_ADDR			0X17
#define BNO_GYR_DATA_Z_LSB_ADDR			0X18
#define BNO_GYR_DATA_Z_MSB_ADDR			0X19

/*Euler data registers*/
#define BNO_EUL_HEADING_LSB_ADDR		0X1A
#define BNO_EUL_HEADING_MSB_ADDR		0X1B

#define BNO_EUL_ROLL_LSB_ADDR			0X1C
#define BNO_EUL_ROLL_MSB_ADDR			0X1D

#define BNO_EUL_PITCH_LSB_ADDR			0X1E
#define BNO_EUL_PITCH_MSB_ADDR			0X1F

/*Quaternion data registers*/
#define BNO_QUA_DATA_W_LSB_ADDR			0X20
#define BNO_QUA_DATA_W_MSB_ADDR			0X21
#define BNO_QUA_DATA_X_LSB_ADDR			0X22
#define BNO_QUA_DATA_X_MSB_ADDR			0X23
#define BNO_QUA_DATA_Y_LSB_ADDR			0X24
#define BNO_QUA_DATA_Y_MSB_ADDR			0X25
#define BNO_QUA_DATA_Z_LSB_ADDR			0X26
#define BNO_QUA_DATA_Z_MSB_ADDR			0X27

/* Linear acceleration data registers*/
#define BNO_LIA_DATA_X_LSB_ADDR			0X28
#define BNO_LIA_DATA_X_MSB_ADDR			0X29
#define BNO_LIA_DATA_Y_LSB_ADDR			0X2A
#define BNO_LIA_DATA_Y_MSB_ADDR			0X2B
#define BNO_LIA_DATA_Z_LSB_ADDR			0X2C
#define BNO_LIA_DATA_Z_MSB_ADDR			0X2D

/*Gravity data registers*/
#define BNO_GRV_DATA_X_LSB_ADDR			0X2E
#define BNO_GRV_DATA_X_MSB_ADDR			0X2F
#define BNO_GRV_DATA_Y_LSB_ADDR			0X30
#define BNO_GRV_DATA_Y_MSB_ADDR			0X31
#define BNO_GRV_DATA_Z_LSB_ADDR			0X32
#define BNO_GRV_DATA_Z_MSB_ADDR			0X33

/* Temperature data register*/

#define BNO_TEMP_ADDR				    0X34

/* Status registers*/
#define BNO_CALIB_STAT_ADDR			    0X35
#define BNO_ST_RESULT_ADDR			    0X36
#define BNO_INT_STA_ADDR			    0X37
#define BNO_SYS_STATUS_ADDR			    0X39
#define BNO_SYS_ERR_ADDR			    0X3A

/* Unit selection register*/
#define BNO_UNIT_SEL_ADDR		    	0X3B
#define BNO_DATA_SEL_ADDR		    	0X3C

/* Mode registers*/
#define BNO_OPR_MODE_ADDR		    	0X3D
#define BNO_PWR_MODE_ADDR		    	0X3E

#define BNO_SYS_TRIGGER_ADDR			0X3F
#define BNO_TEMP_SOURCE_ADDR			0X40
/* Axis remap registers*/
#define BNO_AXIS_MAP_CONFIG_ADDR		0X41
#define BNO_AXIS_MAP_SIGN_ADDR			0X42

/* SIC registers*/
#define BNO_SIC_MATRIX_0_LSB_ADDR		0X43
#define BNO_SIC_MATRIX_0_MSB_ADDR		0X44
#define BNO_SIC_MATRIX_1_LSB_ADDR		0X45
#define BNO_SIC_MATRIX_1_MSB_ADDR		0X46
#define BNO_SIC_MATRIX_2_LSB_ADDR		0X47
#define BNO_SIC_MATRIX_2_MSB_ADDR		0X48
#define BNO_SIC_MATRIX_3_LSB_ADDR		0X49
#define BNO_SIC_MATRIX_3_MSB_ADDR		0X4A
#define BNO_SIC_MATRIX_4_LSB_ADDR		0X4B
#define BNO_SIC_MATRIX_4_MSB_ADDR		0X4C
#define BNO_SIC_MATRIX_5_LSB_ADDR		0X4D
#define BNO_SIC_MATRIX_5_MSB_ADDR		0X4E
#define BNO_SIC_MATRIX_6_LSB_ADDR		0X4F
#define BNO_SIC_MATRIX_6_MSB_ADDR		0X50
#define BNO_SIC_MATRIX_7_LSB_ADDR		0X51
#define BNO_SIC_MATRIX_7_MSB_ADDR		0X52
#define BNO_SIC_MATRIX_8_LSB_ADDR		0X53
#define BNO_SIC_MATRIX_8_MSB_ADDR		0X54

/* Accelerometer Offset registers*/
#define ACC_OFFSET_X_LSB_ADDR			0X55
#define ACC_OFFSET_X_MSB_ADDR			0X56
#define ACC_OFFSET_Y_LSB_ADDR			0X57
#define ACC_OFFSET_Y_MSB_ADDR			0X58
#define ACC_OFFSET_Z_LSB_ADDR			0X59
#define ACC_OFFSET_Z_MSB_ADDR			0X5A

/* Magnetometer Offset registers*/
#define MAG_OFFSET_X_LSB_ADDR			0X5B
#define MAG_OFFSET_X_MSB_ADDR			0X5C
#define MAG_OFFSET_Y_LSB_ADDR			0X5D
#define MAG_OFFSET_Y_MSB_ADDR			0X5E
#define MAG_OFFSET_Z_LSB_ADDR			0X5F
#define MAG_OFFSET_Z_MSB_ADDR			0X60

/* Gyroscope Offset registers*/
#define GYRO_OFFSET_X_LSB_ADDR			0X61
#define GYRO_OFFSET_X_MSB_ADDR			0X62
#define GYRO_OFFSET_Y_LSB_ADDR			0X63
#define GYRO_OFFSET_Y_MSB_ADDR			0X64
#define GYRO_OFFSET_Z_LSB_ADDR			0X65
#define GYRO_OFFSET_Z_MSB_ADDR			0X66

/* PAGE0 REGISTERS DEFINITION END*/

/* PAGE1 REGISTERS DEFINITION START*/
/* Configuration registers*/
#define ACC_CONFIG_ADDR					0X08
#define MAG_CONFIG_ADDR					0X09
#define GYRO_CONFIG_ADDR				0X0A
#define GYRO_MODE_CONFIG_ADDR			0X0B
#define ACC_SLEEP_CONFIG_ADDR			0X0C
#define GYR_SLEEP_CONFIG_ADDR			0X0D
#define MAG_SLEEP_CONFIG_ADDR			0x0E

/* Interrupt registers*/
#define INT_MSK_ADDR					0X0F
#define INT_ADDR						0X10
#define ACC_AM_THRES_ADDR				0X11
#define ACC_INT_SETTINGS_ADDR			0X12
#define ACC_HG_DURATION_ADDR			0X13
#define ACC_HG_THRES_ADDR				0X14
#define ACC_NM_THRES_ADDR				0X15
#define ACC_NM_SET_ADDR					0X16
#define GYR_INT_SETING_ADDR				0X17
#define GYR_HR_X_SET_ADDR				0X18
#define GYR_DUR_X_ADDR					0X19
#define GYR_HR_Y_SET_ADDR				0X1A
#define GYR_DUR_Y_ADDR					0X1B
#define GYR_HR_Z_SET_ADDR				0X1C
#define GYR_DUR_Z_ADDR					0X1D
#define GYR_AM_THRES_ADDR				0X1E
#define GYR_AM_SET_ADDR					0X1F
/* PAGE1 REGISTERS DEFINITION END*/


typedef enum {
  			CONFIG = 0X00,
  			ACCONLY = 0X01,
  			MAGONLY = 0X02,
  			GYRONLY = 0X03,
  			ACCMAG = 0X04,
  			ACCGYRO = 0X05,
  			MAGGYRO = 0X06,
  			AMG = 0X07,
  			IMUPLUS = 0X08,
  			COMPASS = 0X09,
  			M4G = 0X0A,
  			NDOF_FMC_OFF = 0X0B,
  			NDOF = 0X0C
} opr_mode;

esp_err_t i2c_master_init(void);

uint8_t readRegister(uint8_t reg_addr);

esp_err_t writeRegister(uint8_t reg_addr, uint8_t data);

uint8_t bno_getMode();

void bno_setMode(opr_mode mode);

void bno_getCalib(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag);

void bno_init();