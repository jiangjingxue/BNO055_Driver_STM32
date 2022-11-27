/*
 * bno055.c
 *
 *  Created on: Jun. 24, 2022
 *      Author: jingxuejiang
 */
/* Includes ------------------------------------------------------------------*/
#include "bno055.h"

/* Structure definitions-------------------------------------------------------*/

/********************************************************/
/**\             FUNCTION DEFINITIONS                   */
/********************************************************/
/*Write to register*/
BNO055_StatusTypeDef BNO055_Mem_Write(BNO055_InitTypeDef *bno,uint8_t RegAddress,uint8_t *pData,uint16_t Size)
{

	if(HAL_I2C_Mem_Write(&hi2c1, ((bno->DeviceAddress)<< 1), RegAddress, 1, pData, Size, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	return BNO055_OK;
}

/*Read from register*/
BNO055_StatusTypeDef BNO055_Mem_Read(BNO055_InitTypeDef *bno,uint8_t RegAddress,uint8_t *pData,uint16_t Size)
{

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress)<< 1), RegAddress, 1, pData, Size, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	return BNO055_OK;
}

/*Initialize device address, calibration offsets and radius*/
BNO055_StatusTypeDef BNO055_Init(BNO055_InitTypeDef *bno)
{
	if(bno == NULL)
	{
		return BNO055_ERROR;
	}

	//Check if the device address is valid
	if(HAL_I2C_IsDeviceReady(&hi2c1, ((bno->DeviceAddress) << 1), 2, 10) != HAL_OK)
	{
		return BNO055_ERROR;
	}


	//Check if page id is zero;
	uint8_t pageid = 0;

	if((BNO055_Mem_Read(bno,BNO055_PAGE_ID_ADDR,&pageid,1) != BNO055_OK) || (pageid == BNO055_PAGE_ONE))
	{
		return BNO055_ERROR;
	}

	//Check if operation mode is in config mode;
	uint8_t oprmode = 0;

	if((BNO055_Mem_Read(bno,BNO055_OPR_MODE_ADDR,&oprmode,1) != BNO055_OK))
	{
		return BNO055_ERROR;
	}
	else
	{
		oprmode = BNOO055_SHIFT_MASK_BITS(oprmode,0,15);

		if(oprmode != BNO055_OPRMODE_CONFIGMODE)
		{
			return BNO055_ERROR;
		}
	}

	//Check if the user wants to write calibration data (offsets and radius)
	if(bno->AccCalibDE == BNO055_CALIBDATA_ENABLE)
	{
		if(bno->Acc == NULL)
		{
			return BNO055_ERROR;
		}

		//Write accelerometer calibration data to the registers
	}

	if(bno->MagCalibDE == BNO055_CALIBDATA_ENABLE)
	{
		if(bno->Mag == NULL)
		{
			return BNO055_ERROR;
		}

		//Write magnetometer calibration data to the registers
	}

	if(bno->GyrCalibDE == BNO055_CALIBDATA_ENABLE)
	{
		if(bno->Gyro == NULL)
		{
			return BNO055_ERROR;
		}

		//Write gyroscope calibration data to the registers
	}

	return BNO055_OK;
}

/**
  * @brief Function to select operation mode
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_OperationMode_Write(BNO055_InitTypeDef *bno,uint8_t oprmode)
{
	//parameter check
	if(IS_OPR_MODE(oprmode) == 0)
	{
		return BNO055_ERROR;
	}

	if(HAL_I2C_Mem_Write(&hi2c1, ((bno->DeviceAddress)<< 1), BNO055_OPR_MODE_ADDR, 1, &oprmode, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	HAL_Delay(19);

	return BNO055_OK;
}

/**
  * @brief Function to read operation mode
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_OperationMode_Read(BNO055_InitTypeDef *bno,uint8_t *oprmode)
{
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress)<< 1), BNO055_OPR_MODE_ADDR, 1, oprmode, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	//mask out the lower 4 bits.
	*oprmode = BNOO055_SHIFT_MASK_BITS(*oprmode,0,15);
	return BNO055_OK;

}

/**
  * @brief Function to read write euler angle units
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_Unit_Eul_Write(BNO055_InitTypeDef *bno,uint8_t eulunit)
{
	uint8_t unit = 0;

	//Save current units stored in unit_sel register
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_UNIT_SEL_ADDR, 1, &unit, 1, 20) != HAL_OK)
	{
		//use deault unit
		return BNO055_ERROR;
	}

	unit = BNO055_SET_BITS(unit,147,eulunit,2);    //the value of this variable should equal to 132 in decimal

	//Change euler unit while keeping the other units unchanged
	if(HAL_I2C_Mem_Write(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_UNIT_SEL_ADDR, 1, &unit, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	return BNO055_OK;
}

/**
  * @brief Function to read data output units
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_Unit_Read(BNO055_InitTypeDef *bno,BNO055_OutUnit *unit)
{
	//if page id is one, change it to zero

	uint8_t unitsel = 0;
	uint8_t outunit = 0;

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_UNIT_SEL_ADDR, 1, &unitsel, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	//Rotation angle convention: Windows or Andriod
	outunit = BNOO055_SHIFT_MASK_BITS(unitsel,7,1);

	unit->RotationAngleConvention = outunit;

	//Temp units: Celsius or Fahrenheit
	outunit = BNOO055_SHIFT_MASK_BITS(unitsel,4,1);

	unit->TempUnit = outunit;

	//Euler units: Degrees or radians
	outunit = BNOO055_SHIFT_MASK_BITS(unitsel,2,1);

	unit->EulerUnit = outunit;

	//Angular rate units: dps or rps
	outunit = BNOO055_SHIFT_MASK_BITS(unitsel,1,1);

	unit->AngularVelocityUnit = outunit;

	//acceleration units: m/s^2 or mg
	outunit = BNOO055_SHIFT_MASK_BITS(unitsel,0,1);

	unit->AccelUnit = outunit;

	return BNO055_OK;
}

/**
  * @brief Function to read calibration status
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_CalibStat_Read(BNO055_InitTypeDef *bno,BNO055_Calib_Status *stat)
{
	//if page id is one, set it to zero
	uint8_t data = 0;
	uint8_t calibstat = 0;

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_CALIB_STATUS_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	//sys status
	calibstat = BNOO055_SHIFT_MASK_BITS(data,6,3);

	stat->SysCalibStatus = calibstat;

	//gyro status
	calibstat = BNOO055_SHIFT_MASK_BITS(data,4,3);

	stat->GyroCalibStatus = calibstat;

	//acc status
	calibstat = BNOO055_SHIFT_MASK_BITS(data,2,3);

	stat->AccCalibStatus = calibstat;

	//mag status
	calibstat = BNOO055_SHIFT_MASK_BITS(data,0,3);

	stat->MagCalibStatus = calibstat;

	return BNO055_OK;

}

/**
  * @brief Function to read sytem error bit from system status register
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_SystemStat_Read(BNO055_InitTypeDef *bno,uint8_t *syserr)
{
	uint8_t data = 0;

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_SYS_STATUS_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	*syserr = data;

	return BNO055_OK;
}

/**
  * @brief Function to read self test result
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_SelfTest_Result_Read(BNO055_InitTypeDef *bno,uint8_t *st)
{
	uint8_t data = 0;

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_ST_RESULT_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	*st = BNOO055_SHIFT_MASK_BITS(data,0,15);	//st result should be 15 by default

	return BNO055_OK;

}

/**
  * @brief Function to read euler angle headings measured in degrees
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_Euler_Heading_Deg_Read(BNO055_InitTypeDef *bno,double *eul)
{
	uint8_t data[2]= {0,0};
	int16_t eul16 = 0;

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_EUL_DATA_X_LSB_ADDR, 1, data, 2, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	eul16 = ((int16_t)data[1] << 8) | data[0];

	*eul = (double)(eul16 / 16.0);

	return BNO055_OK;

}

/**
  * @brief Function to read roll measured in degrees
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_Euler_Roll_Deg_Read(BNO055_InitTypeDef *bno,double *eul)
{
	uint8_t data[2]= {0,0};
	int16_t eul16 = 0;

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_EUL_DATA_Y_LSB_ADDR, 1, data, 2, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	eul16 = ((int16_t)data[1] << 8) | data[0];

	*eul = (double)(eul16 / 16.0);

	return BNO055_OK;

}

/**
  * @brief Function to read pitch measured in degrees
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_Euler_Pitch_Deg_Read(BNO055_InitTypeDef *bno,double *eul)
{
	uint8_t data[2]= {0,0};
	int16_t eul16 = 0;

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_EUL_DATA_Z_LSB_ADDR, 1, data, 2, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	eul16 = ((int16_t)data[1] << 8) | data[0];

	*eul = (double)(eul16 / 16.0);

	return BNO055_OK;
}

/**
  * @brief Function to read linear acceleration along x-axis in m/s^2
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_LinearAccel_X_MSQ_Read(BNO055_InitTypeDef *bno,double *lia)
{
	uint8_t data[2] = {0,0};
	int16_t lia16 = 0;

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_LIA_DATA_X_LSB_ADDR, 1, data, 2, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	lia16 = ((int16_t)data[1] << 8) | data[0];

	*lia = (double)(lia16 / 100.0);

	return BNO055_OK;

}

/**
  * @brief Function to read linear acceleration along y-axis in m/s^2
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_LinearAccel_Y_MSQ_Read(BNO055_InitTypeDef *bno,double *lia)
{
	uint8_t data[2] = {0,0};
	int16_t lia16 = 0;

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_LIA_DATA_Y_LSB_ADDR, 1, data, 2, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	lia16 = ((int16_t)data[1] << 8) | data[0];

	*lia = (double)(lia16 / 100.0);

	return BNO055_OK;
}

/**
  * @brief Function to read linear acceleration along z-axis in m/s^2
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_LinearAccel_Z_MSQ_Read(BNO055_InitTypeDef *bno,double *lia)
{
	uint8_t data[2] = {0,0};
	int16_t lia16 = 0;

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_LIA_DATA_Z_LSB_ADDR, 1, data, 2, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	lia16 = ((int16_t)data[1] << 8) | data[0];

	*lia = (double)(lia16 / 100.0);

	return BNO055_OK;
}

/**
  * @brief Function to read angular rate along x-axis in dps
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_AngularRate_X_DPS_Read(BNO055_InitTypeDef *bno,double *vel)
{
	uint8_t data[2] = {0,0};
	int16_t vel16 = 0;

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_GYR_DATA_X_LSB_ADDR, 1, data, 2, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	vel16 = ((int16_t)data[1] << 8) | data[0];

	*vel = (double)(vel16 / 16.0);

	return BNO055_OK;
}

/**
  * @brief Function to read angular rate along y axis in dps
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_AngularRate_Y_DPS_Read(BNO055_InitTypeDef *bno,double *vel)
{
	uint8_t data[2] = {0,0};
	int16_t vel16 = 0;

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_GYR_DATA_Y_LSB_ADDR, 1, data, 2, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	vel16 = ((int16_t)data[1] << 8) | data[0];

	*vel = (double)(vel16 / 16.0);

	return BNO055_OK;
}

/**
  * @brief Function to read angular rate along z axis in dps
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_AngularRate_Z_DPS_Read(BNO055_InitTypeDef *bno,double *vel)
{
	uint8_t data[2] = {0,0};
	int16_t vel16 = 0;

	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_GYR_DATA_Z_LSB_ADDR, 1, data, 2, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	vel16 = ((int16_t)data[1] << 8) | data[0];

	*vel = (double)(vel16 / 16.0);

	return BNO055_OK;
}

/**
  * @brief Function to read accelerometer's calibration profile;
  * @param None
  * @retval None
  */
BNO055_StatusTypeDef BNO055_CalibProfile_Acc_Read(BNO055_InitTypeDef *bno,BNO055_CalibProfile_Acc *profile)
{
	uint8_t data;

	if(profile == NULL)
	{
		return BNO055_ERROR;
	}

	//read the content of the offset_x_lsb register and store it in the struct
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_ACC_OFFSET_X_LSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_acc_offset_x_lsb = data;

	//read the content of the offset_x_msb register and store it in the struct
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_ACC_OFFSET_X_MSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_acc_offset_x_msb = data;

	//read the content of the offset_y_lsb register and store it in the struct
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_ACC_OFFSET_Y_LSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_acc_offset_y_lsb = data;

	//read the content of the offset_y_msb register and store it in the struct
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_ACC_OFFSET_Y_MSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_acc_offset_y_msb = data;

	//read the content of the offset_z_lsb register and store it in the struct
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_ACC_OFFSET_Z_LSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_acc_offset_z_lsb = data;

	//read the content of the offset_z_msb register and store it in the struct
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_ACC_OFFSET_Z_MSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_acc_offset_z_msb = data;

	//read the content of the ACC_RADIUS_LSB register and stroe it in the struct
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_ACC_RADIUS_LSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_acc_radius_lsb = data;

	//read the content of the ACC_RADIUS_MSB register and stroe it in the struct
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_ACC_RADIUS_MSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_acc_radius_msb = data;

	return BNO055_OK;

}

BNO055_StatusTypeDef BNO055_CalibProfile_Gyro_Read(BNO055_InitTypeDef *bno,BNO055_CalibProfile_Gyro *profile)
{
	uint8_t data;

	if(profile == NULL)
	{
		return BNO055_ERROR;
	}

	//GYR_OFFSET_X_LSB 0x61
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_GYR_OFFSET_X_LSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_gyr_offset_x_lsb = data;

	//GYR_OFFSET_X_MSB 0x62
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_GYR_OFFSET_X_MSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_gyr_offset_x_msb = data;

	//GYR_OFFSET_Y_LSB 0x63
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_GYR_OFFSET_Y_LSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_gyr_offset_y_lsb = data;

	//GYR_OFFSET_Y_MSB 0x64
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_GYR_OFFSET_Y_MSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_gyr_offset_y_msb = data;

	//GYR_OFFSET_Z_LSB 0x65
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_GYR_OFFSET_Z_LSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_gyr_offset_z_lsb = data;

	//GYR_OFFSET_Z_MSB 0x66
	if(HAL_I2C_Mem_Read(&hi2c1, ((bno->DeviceAddress) << 1), BNO055_GYR_OFFSET_Z_MSB_ADDR, 1, &data, 1, 20) != HAL_OK)
	{
		return BNO055_ERROR;
	}

	profile->Reg_gyr_offset_z_msb = data;

	return BNO055_OK;

}
