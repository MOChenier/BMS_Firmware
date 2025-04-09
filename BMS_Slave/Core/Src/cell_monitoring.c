/*
 * cell_monitoring.c
 *
 *  Created on: Mar 22, 2025
 *      Author: Alexis G.
 */


#include "cell_monitoring.h"

uint16_t cell_voltages[NB_CELLS];
uint16_t pack_temps[NB_TEMP_SENS];

// IMPROTED

RegisterGroup Registers;

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

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	vTaskDelay(20);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	vTaskDelay(20);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	vTaskDelay(20);

	//vTaskDelay(1000);
    int init_status = InitialisebqMaximo();
	int Result;
	//vTaskDelay(1000);
	uint8_t data = 0;

	for(;;){

	    if (HAL_I2C_IsDeviceReady(&hi2c2, BQ76940_ADDR, 3, HAL_MAX_DELAY) == HAL_OK)
	    {
//	    	if (HAL_I2C_Mem_Read(&hi2c2, BQ76940_ADDR, 0x00, 1, &data, 1, 100) == HAL_OK){
			Result = UpdateVoltageFromBqMaximo();
			// Red LED
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
			vTaskDelay(1000);

//	    	}

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

// I2C Read from the BQ76940
HAL_StatusTypeDef BQ76940_ReadRegister(uint8_t reg, uint8_t *data) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(&hi2c2, BQ76940_ADDR, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(&hi2c2, BQ76940_ADDR, data, 1, HAL_MAX_DELAY);
}


unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key)
{
	unsigned char i;
	unsigned char crc_var=0;
	while(len--!=0)
	{
		for(i=0x80; i!=0; i/=2)
		{
			if((crc_var & 0x80) != 0)
			{
				crc_var *= 2;
				crc_var ^= key;
			}
			else
				crc_var *= 2;

			if((*ptr & i)!=0)
				crc_var ^= key;
		}
		ptr++;
	}
	return(crc_var);
}


int ReadRegisterWithCRC(uint8_t I2CSlaveAddress, uint8_t Register, uint8_t *Data)
{
    HAL_StatusTypeDef WriteStatus = HAL_OK;
    HAL_StatusTypeDef ReadStatus = HAL_OK;
    uint8_t ReadData[2];            // [0] = actual data, [1] = CRC from slave
    uint8_t CRCInput[2];            // For manual CRC check
    uint8_t CRC_val = 0;

    // Step 1: Send the register address to read from
    WriteStatus = HAL_I2C_Master_Transmit(&hi2c2,
                                          I2CSlaveAddress << 1,
                                          &Register,
                                          1,
										  HAL_MAX_DELAY);

    // Step 2: Read 2 bytes (data + CRC)
    ReadStatus = HAL_I2C_Master_Receive(&hi2c2,
                                        (I2CSlaveAddress << 1) | 0x01,
                                        ReadData,
                                        2,
										HAL_MAX_DELAY);

    // Step 3: Error check
    if (WriteStatus != HAL_OK || ReadStatus != HAL_OK)
    {
        return -1;
    }

    // Step 4: CRC check
    CRCInput[0] = (I2CSlaveAddress << 1) | 0x01; // Read address with R/W bit set
    CRCInput[1] = ReadData[0];                  // Only the data byte

    CRC_val = CRC8(CRCInput, 2, CRC_KEY);       // Your CRC8 function

    if (CRC_val != ReadData[1])
    {
        return -1; // CRC mismatch
    }

    *Data = ReadData[0]; // Output the valid data
    return 0;
}
int I2CReadBlockWithCRC(uint8_t I2CSlaveAddress, uint8_t Register, uint8_t *Buffer, uint8_t Length)
{
    HAL_StatusTypeDef WriteStatus = HAL_OK;
    HAL_StatusTypeDef ReadStatus = HAL_OK;

    uint8_t TargetRegister = Register;
    uint8_t ReadData[64];  // Max expected Length = 32 (adjust size as needed)

    if (Length > 32)  // Prevent buffer overrun
        return -1;

    // Send the register address
    WriteStatus = HAL_I2C_Master_Transmit(&hi2c2, I2CSlaveAddress << 1, &TargetRegister, 1, HAL_MAX_DELAY);
    if (WriteStatus != HAL_OK)
        return -1;

    // Read Length bytes (without CRC)
    ReadStatus = HAL_I2C_Master_Receive(&hi2c2, (I2CSlaveAddress << 1) | 0x01, ReadData, Length, HAL_MAX_DELAY);
    if (ReadStatus != HAL_OK)
        return -1;

    // Copy received data to buffer
    for (int i = 0; i < Length; i++)
    {
        Buffer[i] = ReadData[i];
    }

    return 0;
	/*
    HAL_StatusTypeDef WriteStatus = HAL_OK;
    HAL_StatusTypeDef ReadStatus = HAL_OK;

    uint8_t TargetRegister = Register;
    uint8_t ReadData[64];  // 2 * max expected Length (adjust size as needed)
    uint8_t CRC_val = 0;
    uint8_t CRCInput[2];
    int i;

    if (Length > 32)  // Prevent buffer overrun
        return -1;

    // Send the register address
    WriteStatus = HAL_I2C_Master_Transmit(&hi2c2, I2CSlaveAddress << 1, &TargetRegister, 1, HAL_MAX_DELAY);
    if (WriteStatus != HAL_OK)
        return -1;

    // Read 2 * Length bytes (data + CRC for each byte)
    ReadStatus = HAL_I2C_Master_Receive(&hi2c2, (I2CSlaveAddress << 1) | 0x01, ReadData, 2 * Length, HAL_MAX_DELAY);
    if (ReadStatus != HAL_OK)
        return -1;

    // First byte uses special CRC input
    CRCInput[0] = (I2CSlaveAddress << 1) | 0x01;
    CRCInput[1] = ReadData[0];
    CRC_val = CRC8(CRCInput, 2, CRC_KEY);

    if (CRC_val != ReadData[1])
        return -1;

    Buffer[0] = ReadData[0];

    // Remaining bytes
    for (i = 1; i < Length; i++)
    {
        uint8_t data = ReadData[2 * i];
        uint8_t received_crc = ReadData[2 * i + 1];

        CRC_val = CRC8(&data, 1, CRC_KEY);
        if (CRC_val != received_crc)
            return -1;

        Buffer[i] = data;
    }

    return 0;
    */
}
int I2CWriteBlockWithCRC(uint8_t I2CSlaveAddress, uint8_t StartAddress, uint8_t *Buffer, uint8_t Length)
{

    HAL_StatusTypeDef WriteStatus = HAL_OK;

    uint8_t BufferToSend[32 + 1]; // Max Length = 32 → (1 + 32) = 33 bytes
    uint8_t i;

    if (Length > 32)  // Prevent buffer overflow
        return -1;

    // First byte: Start address
    BufferToSend[0] = StartAddress;

    // Copy data bytes into BufferToSend
    for (i = 0; i < Length; i++)
    {
        BufferToSend[i + 1] = Buffer[i];
    }

    // Send data starting from StartAddress
    // Total size = 1 (Start Address) + Length (Data Bytes)
    WriteStatus = HAL_I2C_Master_Transmit(&hi2c2, I2CSlaveAddress << 1, BufferToSend, 1 + Length, HAL_MAX_DELAY);

    if (WriteStatus != HAL_OK)
        return -1;

    return 0;

	/*
    HAL_StatusTypeDef WriteStatus = HAL_OK;

    // 2*Length (data + CRC for each byte) + 1 for address
    uint8_t BufferCRC[66]; // Max Length = 32 → (2*32 + 2) = 66
    uint8_t i;
    uint8_t *Pointer = BufferCRC;

    if (Length > 32)  // Prevent buffer overflow
        return -1;

    // First byte: Start address
    *Pointer++ = StartAddress;

    // Compute first data byte and its CRC
    *Pointer = Buffer[0];
    Pointer++;

    BufferCRC[0] = I2CSlaveAddress << 1;  // Used only for CRC input
    BufferCRC[1] = StartAddress;
    BufferCRC[2] = Buffer[0];
    *Pointer = CRC8(BufferCRC, 3, CRC_KEY);
    Pointer++;

    // Remaining bytes
    for (i = 1; i < Length; i++)
    {
        *Pointer = Buffer[i];
        Pointer++;
        *Pointer = CRC8(&Buffer[i], 1, CRC_KEY);
        Pointer++;
    }

    // Send data starting from StartAddress (already stored in BufferCRC[0])
    // Total size = 1 + (2 * Length)
    WriteStatus = HAL_I2C_Master_Transmit(&hi2c2, I2CSlaveAddress << 1, BufferCRC, 1 + (2 * Length), HAL_MAX_DELAY);

    if (WriteStatus != HAL_OK)
        return -1;

    return 0;

   */
}


HAL_StatusTypeDef GetADCGainOffset()
{
    HAL_StatusTypeDef WriteStatus = HAL_OK;


    WriteStatus = HAL_I2C_Mem_Read(&hi2c2, BQMAXIMO, ADCGAIN1, I2C_MEMADD_SIZE_8BIT, &(Registers.ADCGain1.ADCGain1Byte), 1, HAL_MAX_DELAY);
    WriteStatus = HAL_I2C_Mem_Read(&hi2c2, BQMAXIMO, ADCGAIN2, I2C_MEMADD_SIZE_8BIT, &(Registers.ADCGain2.ADCGain2Byte), 1, HAL_MAX_DELAY);
    WriteStatus = HAL_I2C_Mem_Read(&hi2c2, BQMAXIMO, ADCOFFSET, I2C_MEMADD_SIZE_8BIT, &(Registers.ADCOffset), 1, HAL_MAX_DELAY);

	//result = ReadRegisterWithCRC(BQMAXIMO, ADCGAIN1, &(Registers.ADCGain1.ADCGain1Byte));
	//result = ReadRegisterWithCRC(BQMAXIMO, ADCGAIN2, &(Registers.ADCGain2.ADCGain2Byte));
	//result = ReadRegisterWithCRC(BQMAXIMO, ADCOFFSET, &(Registers.ADCOffset));

	return WriteStatus;
}

HAL_StatusTypeDef ConfigureBqMaximo()
{
    HAL_StatusTypeDef WriteStatus = HAL_OK;
	unsigned char bqMaximoProtectionConfig[5];

	WriteStatus = I2CWriteBlockWithCRC(BQMAXIMO, PROTECT1, &(Registers.Protect1.Protect1Byte), 5);

	WriteStatus = I2CReadBlockWithCRC(BQMAXIMO, PROTECT1, bqMaximoProtectionConfig, 5);

	if(bqMaximoProtectionConfig[0] != Registers.Protect1.Protect1Byte
			|| bqMaximoProtectionConfig[1] != Registers.Protect2.Protect2Byte
			|| bqMaximoProtectionConfig[2] != Registers.Protect3.Protect3Byte
			|| bqMaximoProtectionConfig[3] != Registers.OVTrip
			|| bqMaximoProtectionConfig[4] != Registers.UVTrip)
	{
		WriteStatus = HAL_ERROR;
	}

	return WriteStatus;
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

	WriteStatus = GetADCGainOffset();

	Gain = (365 + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((Registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5)) / 1000.0;
	iGain = 365 + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((Registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5);

    Registers.OVTrip = (unsigned char)((((unsigned short)((OVPThreshold - Registers.ADCOffset)/Gain + 0.5) - OV_THRESH_BASE) >> 4) & 0xFF);
    Registers.UVTrip = (unsigned char)((((unsigned short)((UVPThreshold - Registers.ADCOffset)/Gain + 0.5) - UV_THRESH_BASE) >> 4) & 0xFF);

    WriteStatus = ConfigureBqMaximo();

    return WriteStatus;
}

HAL_StatusTypeDef UpdateVoltageFromBqMaximo()
{
    HAL_StatusTypeDef WriteStatus = HAL_OK;
	int i = 0;
	unsigned char *pRawADCData = NULL;
	unsigned int iTemp = 0;
	unsigned long lTemp = 0;

	WriteStatus = I2CReadBlockWithCRC(BQMAXIMO, \
			VC1_HI_BYTE, \
			&(Registers.VCell1.VCell1Byte.VC1_HI), \
			30);

	pRawADCData = &Registers.VCell1.VCell1Byte.VC1_HI;
	for (i = 0; i < 15; i++)
	{
		iTemp = (unsigned int)(*pRawADCData << 8) + *(pRawADCData + 1);
		lTemp = ((unsigned long)iTemp * iGain)/1000;
		lTemp += Registers.ADCOffset;
		CellVoltage[i] = lTemp;
		pRawADCData += 2;
	}

	return WriteStatus;
}

