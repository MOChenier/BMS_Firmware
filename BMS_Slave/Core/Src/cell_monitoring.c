/*
 * cell_monitoring.c
 *
 *  Created on: Mar 22, 2025
 *      Author: Alexis G.
 */


#include "cell_monitoring.h"

//extern osSemaphoreId_t registers_pull;

// IMPROTED

RegisterGroup Registers;

Temps_t CAN_Temps;

const unsigned int OVPThreshold = 4300;
const unsigned int UVPThreshold = 2500;
const unsigned char SCDDelay = SCD_DELAY_100us;
const unsigned char SCDThresh = SCD_THRESH_89mV_44mV;
const unsigned char OCDDelay = OCD_DELAY_320ms;
const unsigned char OCDThresh = OCD_THRESH_22mV_11mV;
const unsigned char OVDelay = OV_DELAY_2s;
const unsigned char UVDelay = UV_DELAY_8s;

unsigned int CellVoltage[15];
float Gain = 0;
int iGain = 0;

void Cell_Motoring_Task(){

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	vTaskDelay(1000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	vTaskDelay(1000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	vTaskDelay(1000);

	HAL_StatusTypeDef init_status = HAL_OK;
	HAL_StatusTypeDef import_status = HAL_OK;

	//init_status = TEST_I2C();

	init_status |= InitialisebqMaximo();

	for(;;){

	    if (HAL_I2C_IsDeviceReady(&hi2c2, BQ76940_ADDR, 3, 1000) == HAL_OK)
	    {

	    	//if (osSemaphoreAcquire(reg_pullHandle, osWaitForever) == 0){

				import_status |= UpdateVoltageFromBqMaximo();
				build_can_temps();

	    	    //osSemaphoreRelease(reg_memHandle);

		    	import_status |= UpdateTempertureFromBqMaximo();
				// Red LED
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
				vTaskDelay(1000);
	    	//}

	    }
		else
		{
			// Yellow LED
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
			vTaskDelay(1000);
	    }

	    vTaskDelay(100);
	}
}

void Ass_CanTempByte(){

}


HAL_StatusTypeDef GetADCGainOffset()
{
    HAL_StatusTypeDef WriteStatus = HAL_OK;

    WriteStatus = HAL_I2C_Mem_Read(&hi2c2, BQMAXIMO, ADCGAIN1, I2C_MEMADD_SIZE_8BIT, &(Registers.ADCGain1.ADCGain1Byte), 1, HAL_MAX_DELAY);
    WriteStatus = HAL_I2C_Mem_Read(&hi2c2, BQMAXIMO, ADCGAIN2, I2C_MEMADD_SIZE_8BIT, &(Registers.ADCGain2.ADCGain2Byte), 1, HAL_MAX_DELAY);
    WriteStatus|= HAL_I2C_Mem_Read(&hi2c2, BQMAXIMO, ADCOFFSET, I2C_MEMADD_SIZE_8BIT, &(Registers.ADCOffset), 1, HAL_MAX_DELAY);

	return WriteStatus;
}

HAL_StatusTypeDef ConfigureBqMaximo()
{
    HAL_StatusTypeDef ConfigStatus = HAL_OK;
	unsigned char bqMaximoProtectionConfig[5];

	//ConfigStatus = I2CWriteBlock(BQMAXIMO, PROTECT1, &(Registers.Protect1.Protect1Byte), 5);
	ConfigStatus = HAL_I2C_Mem_Write(&hi2c2, BQ76940_ADDR, PROTECT1, I2C_MEMADD_SIZE_8BIT, &(Registers.Protect1.Protect1Byte), 1, HAL_MAX_DELAY);
	ConfigStatus = HAL_I2C_Mem_Write(&hi2c2, BQ76940_ADDR, PROTECT2, I2C_MEMADD_SIZE_8BIT, &(Registers.Protect2.Protect2Byte), 1, HAL_MAX_DELAY);
	ConfigStatus = HAL_I2C_Mem_Write(&hi2c2, BQ76940_ADDR, PROTECT3, I2C_MEMADD_SIZE_8BIT, &(Registers.Protect3.Protect3Byte), 1, HAL_MAX_DELAY);
	ConfigStatus = HAL_I2C_Mem_Write(&hi2c2, BQ76940_ADDR, OV_TRIP, I2C_MEMADD_SIZE_8BIT, &(Registers.OVTrip), 1, HAL_MAX_DELAY);
	ConfigStatus = HAL_I2C_Mem_Write(&hi2c2, BQ76940_ADDR, UV_TRIP, I2C_MEMADD_SIZE_8BIT, &(Registers.UVTrip), 1, HAL_MAX_DELAY);

	//ConfigStatus = I2CReadBlock(BQMAXIMO, PROTECT1, bqMaximoProtectionConfig, 5);
	ConfigStatus = HAL_I2C_Mem_Read(&hi2c2, BQMAXIMO, PROTECT1, I2C_MEMADD_SIZE_8BIT, bqMaximoProtectionConfig, 5, HAL_MAX_DELAY);

	if(bqMaximoProtectionConfig[0] != Registers.Protect1.Protect1Byte
			|| bqMaximoProtectionConfig[1] != Registers.Protect2.Protect2Byte
			|| bqMaximoProtectionConfig[2] != Registers.Protect3.Protect3Byte
			|| bqMaximoProtectionConfig[3] != Registers.OVTrip
			|| bqMaximoProtectionConfig[4] != Registers.UVTrip)
	{
		ConfigStatus = HAL_ERROR;
	}

	return ConfigStatus;
}

HAL_StatusTypeDef InitialisebqMaximo()
{
    HAL_StatusTypeDef WriteStatus = HAL_OK;

	Registers.Protect1.Protect1Bit.SCD_DELAY = SCDDelay;
	Registers.Protect1.Protect1Bit.SCD_THRESH = SCDThresh;
	Registers.Protect2.Protect2Bit.OCD_DELAY = OCDDelay;
	Registers.Protect2.Protect2Bit.OCD_THRESH = OCDThresh;
	Registers.Protect3.Protect3Bit.OV_DELAY = OVDelay;
	Registers.Protect3.Protect3Bit.UV_DELAY = UVDelay;

	WriteStatus |= GetADCGainOffset();

	Gain = (365 + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((Registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5)) / 1000.0;
	iGain = 365 + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((Registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5);

    Registers.OVTrip = (unsigned char)((((unsigned short)((OVPThreshold - Registers.ADCOffset)/Gain + 0.5) - OV_THRESH_BASE) >> 4) & 0xFF);
    Registers.UVTrip = (unsigned char)((((unsigned short)((UVPThreshold - Registers.ADCOffset)/Gain + 0.5) - UV_THRESH_BASE) >> 4) & 0xFF);

    WriteStatus |= ConfigureBqMaximo();

    return WriteStatus;
}

HAL_StatusTypeDef UpdateVoltageFromBqMaximo()
{
    HAL_StatusTypeDef ReadStatus = HAL_OK;
	int i = 0;
	unsigned char *pRawADCData = NULL;
	unsigned int iTemp = 0;
	unsigned long lTemp = 0;

	ReadStatus |= HAL_I2C_Mem_Read(&hi2c2, BQ76940_ADDR, VC1_HI_BYTE, 1, &(Registers.VCell1.VCell1Byte.VC1_HI), 33, HAL_MAX_DELAY);

	pRawADCData = &Registers.VCell1.VCell1Byte.VC1_HI;
	for (i = 0; i < 15; i++)
	{
		iTemp = (unsigned int)(*pRawADCData << 8) + *(pRawADCData + 1);
		lTemp = ((unsigned long)iTemp * iGain)/1000;
		lTemp += Registers.ADCOffset;
		CellVoltage[i] = lTemp;
		pRawADCData += 2;
	}

	return ReadStatus;
}

HAL_StatusTypeDef UpdateTempertureFromBqMaximo()
{
    HAL_StatusTypeDef ReadStatus = HAL_OK;
	int i = 0;
	unsigned char *pRawADCData = NULL;
	unsigned int iTemp = 0;
	unsigned long lTemp = 0;

	ReadStatus |= HAL_I2C_Mem_Read(&hi2c2, BQ76940_ADDR, TS1_HI_ADDR, 1, &(Registers.TS1.TS1Byte), 6, HAL_MAX_DELAY);

	pRawADCData = &Registers.TS1.TS1Byte.TS1_HI;
	for (i = 0; i < 3; i++)
	{
		iTemp = (unsigned int)(*pRawADCData << 8) + *(pRawADCData + 1);
		lTemp = ((unsigned long)iTemp * iGain)/1000;
		lTemp += Registers.ADCOffset;
		CellVoltage[i] = lTemp;
		pRawADCData += 2;
	}

	return ReadStatus;
}

// ChatGPT is my homie
uint16_t ConvertTempToUint16(uint8_t msb, uint8_t lsb) {
    uint16_t adc_code = ((uint16_t)msb << 8) | lsb;
    float voltage = (adc_code / 32768.0f) * 1.8f;
    float temp_celsius = (voltage - 0.5f) / 0.01f;

    // Optional: clip range to safe max
    if (temp_celsius < 0) temp_celsius = 0;
    if (temp_celsius > 6553.5f) temp_celsius = 6553.5f;

    // Multiply by 10 to store 0.1°C resolution
    return (uint16_t)(temp_celsius * 10.0f + 0.5f);  // rounded
}

void build_can_temps(){

	CAN_Temps.Temp1 = ConvertTempToUint16(Registers.TS1.TS1Byte.TS1_HI , Registers.TS1.TS1Byte.TS1_LO);
	CAN_Temps.Temp2 = ConvertTempToUint16(Registers.TS2.TS2Byte.TS2_HI , Registers.TS2.TS2Byte.TS2_LO);
	CAN_Temps.Temp3 = ConvertTempToUint16(Registers.TS3.TS3Byte.TS3_HI , Registers.TS3.TS3Byte.TS3_LO);

}


