/*
 * bno055.h
 *
 *  Created on: Jun. 21, 2022
 *  Author: jingxuejiang
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#ifdef __cplusplus
extern "C" {
#endif
/*USER CODE START*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Extern variables ----------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
/********************************************************/
/**\name    I2C ADDRESS DEFINITION FOR BNO055           */
/********************************************************/
/*bno055 i2d address*/
#define BNO055_I2C_ADDR (0x28)

/********************************************************/
/**\name    PAGE0 REGISTER ADDRESS DEFINITION           */
/********************************************************/
/*Page id register*/
#define BNO055_PAGE_ID_ADDR (0x07)


/*Chip id registers*/
#define BNO055_CHIP_ID_ADDR (0x00)
#define BNO055_ACC_CHIP_ID_ADDR (0x01)
#define BNO055_MAG_CHIP_ID_ADDR (0x02)
#define BNO055_GYR_CHIP_ID_ADDR (0x03)

/*Angular velocity registers*/
#define BNO055_GYR_DATA_X_LSB_ADDR (0x14)
#define BNO055_GYR_DATA_X_MSB_ADDR (0x15)
#define BNO055_GYR_DATA_Y_LSB_ADDR (0x16)
#define BNO055_GYR_DATA_Y_MSB_ADDR (0x17)
#define BNO055_GYR_DATA_Z_LSB_ADDR (0x18)
#define BNO055_GYR_DATA_Z_MSB_ADDR (0x19)

/*Euler angle registers*/
#define BNO055_EUL_DATA_X_LSB_ADDR (0x1A)
#define BNO055_EUL_DATA_X_MSB_ADDR (0x1B)
#define BNO055_EUL_DATA_Y_LSB_ADDR (0x1C)
#define BNO055_EUL_DATA_Y_MSB_ADDR (0x1D)
#define BNO055_EUL_DATA_Z_LSB_ADDR (0x1E)
#define BNO055_EUL_DATA_Z_MSB_ADDR (0x1F)

/*Linear acceleration registers*/
#define BNO055_LIA_DATA_X_LSB_ADDR (0x28)
#define BNO055_LIA_DATA_X_MSB_ADDR (0x29)
#define BNO055_LIA_DATA_Y_LSB_ADDR (0x2A)
#define BNO055_LIA_DATA_Y_MSB_ADDR (0x2B)
#define BNO055_LIA_DATA_Z_LSB_ADDR (0x2C)
#define BNO055_LIA_DATA_Z_MSB_ADDR (0x2D)

/*Calibration status register*/
#define BNO055_CALIB_STATUS_ADDR (0x35)

/*Self test result register*/
#define BNO055_ST_RESULT_ADDR (0X36)

/*System status registers*/
#define BNO055_SYS_STATUS_ADDR (0x39)
#define BNO055_SYS_ERR_ADDR (0x3A)

/*Unit selection register*/
#define BNO055_UNIT_SEL_ADDR (0x3B)

/*Operation mode register*/
#define BNO055_OPR_MODE_ADDR (0x3D)

/*Axis remap register*/
#define BNO055_AXIS_MAP_CONFIG_ADDR (0x41)
#define BNO055_AXIS_MAP_SIGN_ADDR (0x42)

/*Accelerometer Offset registers*/
#define BNO055_ACC_OFFSET_X_LSB_ADDR (0x55)
#define BNO055_ACC_OFFSET_X_MSB_ADDR (0x56)
#define BNO055_ACC_OFFSET_Y_LSB_ADDR (0x57)
#define BNO055_ACC_OFFSET_Y_MSB_ADDR (0x58)
#define BNO055_ACC_OFFSET_Z_LSB_ADDR (0x59)
#define BNO055_ACC_OFFSET_Z_MSB_ADDR (0x5A)

/*Magnetometer Offset registers*/
#define BNO055_MAG_OFFSET_X_LSB_ADDR (0x5B)
#define BNO055_MAG_OFFSET_X_MSB_ADDR (0x5C)
#define BNO055_MAG_OFFSET_Y_LSB_ADDR (0x5D)
#define BNO055_MAG_OFFSET_Y_MSB_ADDR (0x5E)
#define BNO055_MAG_OFFSET_Z_LSB_ADDR (0x5F)
#define BNO055_MAG_OFFSET_Z_MSB_ADDR (0x60)

/*Gyroscope Offset registers*/
#define BNO055_GYR_OFFSET_X_LSB_ADDR (0x61)
#define BNO055_GYR_OFFSET_X_MSB_ADDR (0x62)
#define BNO055_GYR_OFFSET_Y_LSB_ADDR (0x63)
#define BNO055_GYR_OFFSET_Y_MSB_ADDR (0x64)
#define BNO055_GYR_OFFSET_Z_LSB_ADDR (0x65)
#define BNO055_GYR_OFFSET_Z_MSB_ADDR (0x66)

/*Radius register*/
#define BNO055_ACC_RADIUS_LSB_ADDR (0x67)
#define BNO055_ACC_RADIUS_MSB_ADDR (0x68)
#define BNO055_MAG_RADIUS_LSB_ADDR (0x69)
#define BNO055_MAG_RADIUS_MSB_ADDR (0x6A)

/*PAGE0 REGISTERS DEFINITION END*/

/********************************************************/
/**\    PAGE1 REGISTER ADDRESS DEFINITION               */
/********************************************************/
/*Implement according to your needs*/
/*PAGE1 REGISTERS DEFINITION END*/

/***************************************************/
/**\          CONSTANT DEFINITIONS                 */
/***************************************************/
/*Non-fusion operation modes*/
#define BNO055_OPRMODE_CONFIGMODE    (0x00U)	//default operation mode after power on
#define BNO055_OPRMODE_ACCONLY       (0x01U)
#define BNO055_OPRMODE_MAGONLY       (0x02U)
#define BNO055_OPRMODE_GYROONLY      (0x03U)
#define BNO055_OPRMODE_ACCMAG        (0x04U)
#define BNO055_OPRMODE_ACCGYRO       (0x05U)
#define BNO055_OPRMODE_MAGGYRO       (0x06U)
#define BNO055_OPRMODE_AMG           (0x07U)

/*Fushion modes*/
#define BNO055_OPRMODE_IMU           (0x08U)
#define BNO055_OPRMODE_COMPASS       (0x09U)
#define BNO055_OPRMODE_M4G           (0x0AU)
#define BNO055_OPRMODE_NDOF_FMC_OFF  (0x0BU)
#define BNO055_OPRMODE_NDOF          (0x0CU)

/*Axis remap configurations*/
#define BNO055_PLACEMENT_1247_AXIS   (0x24U)	//deafult axis placement is P1
#define BNO055_PLACEMENT_0356_AXIS   (0x21U)

/*Axis remap signs*/
#define BNO055_PLACEMENT0_SIGN  	 (0x04U)
#define BNO055_PLACEMENT1_SIGN       (0x00U)	//deafult axis placement is P1
#define BNO055_PLACEMENT2_SIGN       (0x06U)
#define BNO055_PLACEMENT3_SIGN       (0x02U)
#define BNO055_PLACEMENT4_SIGN       (0x03U)
#define BNO055_PLACEMENT5_SIGN       (0x01U)
#define BNO055_PLACEMENT6_SIGN       (0x07U)
#define BNO055_PLACEMENT7_SIGN       (0x05U)

/*Unit combinations*/
#define BNO055_DEFAULT_UNIT          (0x80U)	//Android, Celsius, Degrees, dps, m/s^2
#define BNO055_DEG_DPS_MG_UNIT       (0x81U)    //Android, Celsius, Degrees, dps, mg
#define BNO055_RAD_RPS_MSQ_UNIT      (0x86U)	//Android, Celsius, Radians, rps, m/s^2
#define BNO055_RAD_RPS_MG_UNIT       (0x87U)	//Android, Celsius, Radians, rps, mg

/*Calibration data enable or disable*/
#define BNO055_CALIBDATA_DISABLE     (0x00U)
#define BNO055_CALIBDATA_ENABLE      (0X01U)

/*Enable or disable unit selection*/
#define BNO055_UNITSELECT_DISABLE    (0x00U)
#define BNO055_UNITSELECT_ENABLE     (0X01U)

/*Enable or diable axis remap*/
#define BNO055_AXISREMAP_DISABLE	 (0x00U)
#define BNO055_AXISREMAP_ENABLE      (0x01U)

/*Page ID*/
#define BNO055_PAGE_ZERO             (0X00U)
#define BNO055_PAGE_ONE              (0X01U)

/***************************************************/
/**\             FUNCTION MACROS                   */
/***************************************************/
/*Function to shift and mask bits in registers*/
#define BNOO055_SHIFT_MASK_BITS(value,shiftpos,mask)  ((value >> shiftpos) & mask)
#define BNO055_SET_BITS(value,mask,setval,shiftpos)  ((value & mask) | (setval << shiftpos))
/* */
#define IS_OPR_MODE(MODE)	(((MODE) == BNO055_OPRMODE_CONFIGMODE) ||\
							 ((MODE) == BNO055_OPRMODE_ACCONLY)    ||\
							 ((MODE) == BNO055_OPRMODE_MAGONLY)    ||\
							 ((MODE) == BNO055_OPRMODE_GYROONLY)   ||\
							 ((MODE) == BNO055_OPRMODE_ACCMAG)     ||\
							 ((MODE) == BNO055_OPRMODE_ACCGYRO)    ||\
							 ((MODE) == BNO055_OPRMODE_MAGGYRO)    ||\
							 ((MODE) == BNO055_OPRMODE_AMG)        ||\
							 ((MODE) == BNO055_OPRMODE_IMU)        ||\
							 ((MODE) == BNO055_OPRMODE_COMPASS)    ||\
							 ((MODE) == BNO055_OPRMODE_M4G)        ||\
							 ((MODE) == BNO055_OPRMODE_NDOF_FMC_OFF) ||\
							 ((MODE) == BNO055_OPRMODE_NDOF))

/********************************************************/
/**\name    STRUCTURE DEFINITION           */
/********************************************************/
/*BNO055 Status*/
typedef enum
{
	BNO055_OK = 0x00U,
	BNO055_ERROR = 0x01U

}BNO055_StatusTypeDef;


/*Structure that stores the accelerometer offset and radius*/
typedef struct
{
	uint8_t Reg_acc_offset_x_lsb;
	uint8_t Reg_acc_offset_x_msb;
	uint8_t Reg_acc_offset_y_lsb;
	uint8_t Reg_acc_offset_y_msb;
	uint8_t Reg_acc_offset_z_lsb;
	uint8_t Reg_acc_offset_z_msb;

	uint8_t Reg_acc_radius_lsb;
	uint8_t Reg_acc_radius_msb;

}BNO055_CalibProfile_Acc;

/*Structure that stores the magnetometer offset and raidus*/
typedef struct
{
	uint8_t Reg_mag_offset_x_lsb;
	uint8_t Reg_mag_offset_x_msb;
	uint8_t Reg_mag_offset_y_lsb;
	uint8_t Reg_mag_offset_y_msb;
	uint8_t Reg_mag_offset_z_lsb;
	uint8_t Reg_mag_offset_z_msb;

	uint8_t Reg_mag_radius_lsb;
	uint8_t Reg_mag_radius_msb;

}BNO055_CalibProfile_Mag;

/*Strucutre that stores gyroscope offset*/

typedef struct
{
	uint8_t Reg_gyr_offset_x_lsb;
	uint8_t Reg_gyr_offset_x_msb;
	uint8_t Reg_gyr_offset_y_lsb;
	uint8_t Reg_gyr_offset_y_msb;
	uint8_t Reg_gyr_offset_z_lsb;
	uint8_t Reg_gyr_offset_z_msb;

}BNO055_CalibProfile_Gyro;


/*Structure that stores initialization values*/
typedef struct
{
	uint8_t DeviceAddress;
	uint8_t AccCalibDE;
	uint8_t MagCalibDE;
	uint8_t GyrCalibDE;
	uint8_t UnitSelecDE;
	uint8_t AxisRemapDE;

	BNO055_CalibProfile_Acc *Acc;
	BNO055_CalibProfile_Mag *Mag;
	BNO055_CalibProfile_Gyro *Gyro;
	uint8_t AxisPlacement;
	uint8_t UnitCombination;

}BNO055_InitTypeDef;

/*Structure that stores basic sensor information of BNO055 IMU sensor*/
typedef struct
{
	uint8_t PageID;

	uint8_t ChipID;
	uint8_t AccID;
	uint8_t MagID;
	uint8_t GyroID;

}BNO055_Info;

/*Structure that stores the result of the self test*/
typedef struct
{
	uint8_t McuTest;
	uint8_t GyroTest;
	uint8_t MagTest;
	uint8_t AccTest;

}BNO055_SelfTest_Result;

/*Structure that stores the calibration status of accelerometer, gyroscope and magnetometer */
typedef struct
{
	uint8_t SysCalibStatus;
	uint8_t GyroCalibStatus;
	uint8_t AccCalibStatus;
	uint8_t MagCalibStatus;

}BNO055_Calib_Status;

/*Structure that stores the data output units*/
typedef struct
{
	uint8_t RotationAngleConvention;	// 0: Windows 1: Android
	uint8_t TempUnit;				    // 0: Celcius 1: Fahrenheit
	uint8_t EulerUnit;					// 0: Degrees 1: Radians
	uint8_t AngularVelocityUnit;		// 0: Dps     1: Rps
	uint8_t AccelUnit;		            // 0: m/s^2   1: mg

}BNO055_OutUnit;


/*Structure to store the offset and radius data of all three sensors*/
typedef struct
{
	BNO055_CalibProfile_Gyro gyro;
	BNO055_CalibProfile_Mag mag;
	BNO055_CalibProfile_Acc acc;

}BNO055_Calibration_Profile;


/*Structure to store the angular velocity data from GYR_ DATA_ <axis> registers*/
typedef struct
{
	double x;
	double y;
	double z;

}BNO055_AngularVel_Data;

/*Structure to store the orientation data in Eluer angles format from EUL<dof> registers */
typedef struct
{
	double heading;
	double roll;
	double pitch;

}BNO055_Euler_Data;

/*Structure to store the linear acceleration data from LIA_ DATA_ <axis> registers*/
typedef struct
{
	double x;
	double y;
	double z;

}BNO055_LinearAccel_Data;

/********************************************************/
/**\             FUNCTION DECLARATION                   */
/********************************************************/
/*Write to register*/
BNO055_StatusTypeDef BNO055_Mem_Write(BNO055_InitTypeDef *bno,uint8_t RegAddress,uint8_t *pData,uint16_t Size);

/*Read from register*/
BNO055_StatusTypeDef BNO055_Mem_Read(BNO055_InitTypeDef *bno,uint8_t RegAddress,uint8_t *pData,uint16_t Size);

/*Initialize device address and calibration offsets*/
BNO055_StatusTypeDef BNO055_Init(BNO055_InitTypeDef *bno);

/*Select operation mode*/
BNO055_StatusTypeDef BNO055_OperationMode_Write(BNO055_InitTypeDef *bno,uint8_t oprmode);

/*Read operation mode*/
BNO055_StatusTypeDef BNO055_OperationMode_Read(BNO055_InitTypeDef *bno,uint8_t *oprmode);

/*Change output units*/
BNO055_StatusTypeDef BNO055_Unit_Eul_Write(BNO055_InitTypeDef *bno,uint8_t eulunit);

/*Read output units*/
BNO055_StatusTypeDef BNO055_Unit_Read(BNO055_InitTypeDef *bno,BNO055_OutUnit *unit);      //default value in unit_sel register is 0x80;

/*Read Calibration Status*/
BNO055_StatusTypeDef BNO055_CalibStat_Read(BNO055_InitTypeDef *bno,BNO055_Calib_Status *stat);

/*Read System error from SYS_STATUS register*/
BNO055_StatusTypeDef BNO055_SystemStat_Read(BNO055_InitTypeDef *bno,uint8_t *syserr);

/*Read self test result*/
BNO055_StatusTypeDef BNO055_SelfTest_Result_Read(BNO055_InitTypeDef *bno,uint8_t *st);

/*Read heading in degrees*/
BNO055_StatusTypeDef BNO055_Euler_Heading_Deg_Read(BNO055_InitTypeDef *bno,double *eul);

/*Read roll in degrees*/
BNO055_StatusTypeDef BNO055_Euler_Roll_Deg_Read(BNO055_InitTypeDef *bno,double *eul);

/*Read pitch in degrees*/
BNO055_StatusTypeDef BNO055_Euler_Pitch_Deg_Read(BNO055_InitTypeDef *bno,double *eul);

/*Read linear acceleration along x-axis in m/s^2*/
BNO055_StatusTypeDef BNO055_LinearAccel_X_MSQ_Read(BNO055_InitTypeDef *bno,double *lia);

/*Read linear acceleration along y-axis in m/s^2*/
BNO055_StatusTypeDef BNO055_LinearAccel_Y_MSQ_Read(BNO055_InitTypeDef *bno,double *lia);

/*Read linear acceleration along z-axis in m/s^2*/
BNO055_StatusTypeDef BNO055_LinearAccel_Z_MSQ_Read(BNO055_InitTypeDef *bno,double *lia);

/*Read Angular velocity along x-axis in degree per second*/
BNO055_StatusTypeDef BNO055_AngularRate_X_DPS_Read(BNO055_InitTypeDef *bno,double *vel);

/*Read Angular velocity along y-axis in degree per second*/
BNO055_StatusTypeDef BNO055_AngularRate_Y_DPS_Read(BNO055_InitTypeDef *bno,double *vel);

/*Read Angular velocity along z-axis in degree per second*/
BNO055_StatusTypeDef BNO055_AngularRate_Z_DPS_Read(BNO055_InitTypeDef *bno,double *vel);

/*Read accelerometer's calibration profile*/
BNO055_StatusTypeDef BNO055_CalibProfile_Acc_Read(BNO055_InitTypeDef *bno,BNO055_CalibProfile_Acc *profile);

/*Read gyroscope's calibration profile*/
BNO055_StatusTypeDef BNO055_CalibProfile_Gyro_Read(BNO055_InitTypeDef *bno,BNO055_CalibProfile_Gyro *profile);

/*Write accelerometer's calibration profile*/
BNO055_StatusTypeDef BNO055_CalibProfile_Acc_Write(BNO055_InitTypeDef *bno,BNO055_CalibProfile_Acc profile);









/*USER CODE END*/
#ifdef __cplusplus
}
#endif

#endif /* INC_BNO055_H_ */
