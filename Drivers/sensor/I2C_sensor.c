#include <stdio.h>
#include <stdint.h>
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_delay.h"
#include "log.h"
#include "I2C_sensor.h"
#include "I2C_A.h"
#include "bsp.h"
#include "sht3x.h"

static float gxht30_temp_record=10,gxht30_hum_record=40;
extern bool bh1750flags;
extern bool bme280flags;
extern bool iic_noack;

#define BME280_ADDR_SDO_LOW   0x76
#define BME280_ADDR_SDO_HIGH  0x77
#define BME280_REG_CALIB00    0x88
#define BME280_REG_ID         0xD0
#define BME280_REG_RESET      0xE0
#define BME280_REG_CALIB26    0xE1
#define BME280_REG_CTRL_HUM   0xF2
#define BME280_REG_STATUS     0xF3
#define BME280_REG_CTRL_MEAS  0xF4
#define BME280_REG_CONFIG     0xF5
#define BME280_REG_DATA       0xF7
#define BME280_CHIP_ID        0x60
/* Same T/P registers as the BME280; no humidity registers. */
#define BMP280_CHIP_ID        0x58
#define BME280_RESET_WORD     0xB6
#define BME280_STATUS_MEASURING 0x08
#define BME280_STATUS_IM_UPDATE 0x01
/* ctrl_hum takes effect only on the next ctrl_meas write. */
#define BME280_CTRL_HUM_X1    0x01
/* osrs_t=×1, osrs_p=×1, forced mode. */
#define BME280_CTRL_MEAS_FORCED_X1 0x25

static uint8_t bme280_addr;
static uint8_t bme280_has_hum;
static int32_t bme280_t_fine;
static uint16_t bme280_dig_T1, bme280_dig_P1;
static int16_t bme280_dig_T2, bme280_dig_T3;
static int16_t bme280_dig_P2, bme280_dig_P3, bme280_dig_P4, bme280_dig_P5;
static int16_t bme280_dig_P6, bme280_dig_P7, bme280_dig_P8, bme280_dig_P9;
static uint8_t bme280_dig_H1, bme280_dig_H3;
static int16_t bme280_dig_H2, bme280_dig_H4, bme280_dig_H5;
static int8_t bme280_dig_H6;

static uint16_t bme280_u16(const uint8_t *p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t bme280_s16(const uint8_t *p)
{
	return (int16_t)bme280_u16(p);
}

/* dig_H4/dig_H5 are 12-bit signed values split across 0xE4..0xE6. */
static int16_t bme280_dig_h4(uint8_t e4, uint8_t e5)
{
	return (int16_t)(((int16_t)(int8_t)e4 << 4) | (e5 & 0x0F));
}

static int16_t bme280_dig_h5(uint8_t e5, uint8_t e6)
{
	return (int16_t)(((int16_t)(int8_t)e6 << 4) | (e5 >> 4));
}

/* 0.01 °C. Sets t_fine, which P and H depend on. */
static int32_t bme280_compensate_t(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t)bme280_dig_T1 << 1))) * ((int32_t)bme280_dig_T2)) >> 11;
	var2 = ((((((adc_T >> 4) - ((int32_t)bme280_dig_T1)) * ((adc_T >> 4) - ((int32_t)bme280_dig_T1))) >> 12) *
		((int32_t)bme280_dig_T3)) >> 14);
	bme280_t_fine = var1 + var2;
	T = (bme280_t_fine * 5 + 128) >> 8;
	return T;
}

/* Pa in Q24.8; 0 = divide by zero. */
static uint32_t bme280_compensate_p(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)bme280_t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bme280_dig_P6;
	var2 = var2 + ((var1 * (int64_t)bme280_dig_P5) << 17);
	var2 = var2 + (((int64_t)bme280_dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)bme280_dig_P3) >> 8) + ((var1 * (int64_t)bme280_dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)bme280_dig_P1)) >> 33;
	if(var1 == 0)
	{
		return 0;
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)bme280_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)bme280_dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)bme280_dig_P7) << 4);
	return (uint32_t)p;
}

/* %RH in Q22.10. */
static uint32_t bme280_compensate_h(int32_t adc_H)
{
	int32_t v_x1_u32r;
	v_x1_u32r = (bme280_t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme280_dig_H4) << 20) - (((int32_t)bme280_dig_H5) *
		v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
		((int32_t)bme280_dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)bme280_dig_H3)) >> 11) +
		((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)bme280_dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
		((int32_t)bme280_dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r >> 12);
}

static void bme280_load_cal(const uint8_t *cal, const uint8_t *h)
{
	bme280_dig_T1 = bme280_u16(cal + 0);
	bme280_dig_T2 = bme280_s16(cal + 2);
	bme280_dig_T3 = bme280_s16(cal + 4);
	bme280_dig_P1 = bme280_u16(cal + 6);
	bme280_dig_P2 = bme280_s16(cal + 8);
	bme280_dig_P3 = bme280_s16(cal + 10);
	bme280_dig_P4 = bme280_s16(cal + 12);
	bme280_dig_P5 = bme280_s16(cal + 14);
	bme280_dig_P6 = bme280_s16(cal + 16);
	bme280_dig_P7 = bme280_s16(cal + 18);
	bme280_dig_P8 = bme280_s16(cal + 20);
	bme280_dig_P9 = bme280_s16(cal + 22);
	bme280_dig_H1 = cal[25];
	bme280_dig_H2 = bme280_s16(h);
	bme280_dig_H3 = h[2];
	bme280_dig_H4 = bme280_dig_h4(h[3], h[4]);
	bme280_dig_H5 = bme280_dig_h5(h[4], h[5]);
	bme280_dig_H6 = (int8_t)h[6];
}

uint8_t check_bme280_connect(void)
{
	uint8_t addrs[2] = {BME280_ADDR_SDO_LOW, BME280_ADDR_SDO_HIGH};
	uint8_t n;
	uint8_t id = 0;
	uint8_t reset_word = BME280_RESET_WORD;
	uint8_t cfg = 0x00;
	uint8_t status = BME280_STATUS_IM_UPDATE;
	uint8_t cal[26];
	uint8_t hcal[7] = {0};
	uint8_t tries;

	bme280_addr = 0;
	bme280_has_hum = 0;
	I2C_GPIO_MODE_Config();
	for(n = 0; n < 2; n++)
	{
		id = 0;
		if(I2C_Read_reg_Len(addrs[n], BME280_REG_ID, 1, &id) != 0)
		{
			continue;
		}
		LOG_PRINTF(LL_DEBUG,"\n\rI2C 0x%02X chip ID 0x%02X\r\n", addrs[n], id);
		if(id == BME280_CHIP_ID || id == BMP280_CHIP_ID)
		{
			bme280_addr = addrs[n];
			bme280_has_hum = (id == BME280_CHIP_ID) ? 1 : 0;
			break;
		}
	}
	if(bme280_addr == 0)
	{
		return 0;
	}

	if(I2C_Write_reg_Len(bme280_addr, BME280_REG_RESET, 1, &reset_word) != 0)
	{
		bme280_addr = 0;
		return 0;
	}
	for(tries = 0; tries < 10 && (status & BME280_STATUS_IM_UPDATE); tries++)
	{
		delay_ms(2);
		if(I2C_Read_reg_Len(bme280_addr, BME280_REG_STATUS, 1, &status) != 0)
		{
			bme280_addr = 0;
			return 0;
		}
	}
	if(status & BME280_STATUS_IM_UPDATE)
	{
		bme280_addr = 0;
		return 0;
	}
	if(I2C_Read_reg_Len(bme280_addr, BME280_REG_CALIB00, 26, cal) != 0
		|| (bme280_has_hum && I2C_Read_reg_Len(bme280_addr, BME280_REG_CALIB26, 7, hcal) != 0))
	{
		bme280_addr = 0;
		return 0;
	}
	bme280_load_cal(cal, hcal);
	/* config is only writable in sleep mode, which reset leaves us in. */
	if(I2C_Write_reg_Len(bme280_addr, BME280_REG_CONFIG, 1, &cfg) != 0)
	{
		bme280_addr = 0;
		return 0;
	}
	return 1;
}

void BME280_Read(sensor_t *sensor_data)
{
	uint8_t hum_cmd = BME280_CTRL_HUM_X1;
	uint8_t meas_cmd = BME280_CTRL_MEAS_FORCED_X1;
	uint8_t raw[8];
	uint8_t status;
	uint8_t tries;
	int32_t adc_P, adc_T, adc_H;
	int32_t temp_centi;
	uint32_t press_q24_8;
	uint32_t hum_q22_10;

	sensor_data->temp_sht = 3276.7f;
	sensor_data->hum_sht = 6553.5f;
	sensor_data->press_hpa = -1.0f;
	if(bme280_addr == 0)
	{
		return;
	}

	I2C_GPIO_MODE_Config();
	if(bme280_has_hum && I2C_Write_reg_Len(bme280_addr, BME280_REG_CTRL_HUM, 1, &hum_cmd) != 0)
	{
		return;
	}
	if(I2C_Write_reg_Len(bme280_addr, BME280_REG_CTRL_MEAS, 1, &meas_cmd) != 0)
	{
		return;
	}

	/* Max measurement time at ×1 oversampling is 9.3 ms. */
	delay_ms(10);
	status = BME280_STATUS_MEASURING;
	for(tries = 0; tries < 10 && (status & BME280_STATUS_MEASURING); tries++)
	{
		delay_ms(1);
		if(I2C_Read_reg_Len(bme280_addr, BME280_REG_STATUS, 1, &status) != 0)
		{
			return;
		}
	}
	if(status & BME280_STATUS_MEASURING)
	{
		return;
	}
	if(I2C_Read_reg_Len(bme280_addr, BME280_REG_DATA, bme280_has_hum ? 8 : 6, raw) != 0)
	{
		return;
	}

	adc_P = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | ((uint32_t)raw[2] >> 4));
	adc_T = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | ((uint32_t)raw[5] >> 4));
	/* 0x80000 / 0x8000 mean the measurement was skipped. */
	if(adc_T == 0x80000 || adc_P == 0x80000)
	{
		return;
	}

	temp_centi = bme280_compensate_t(adc_T);
	press_q24_8 = bme280_compensate_p(adc_P);
	sensor_data->temp_sht = (float)temp_centi / 100.0f;
	if(bme280_has_hum)
	{
		adc_H = (int32_t)(((uint32_t)raw[6] << 8) | raw[7]);
		if(adc_H != 0x8000)
		{
			hum_q22_10 = bme280_compensate_h(adc_H);
			sensor_data->hum_sht = (float)hum_q22_10 / 1024.0f;
		}
	}
	if(press_q24_8 == 0)
	{
		sensor_data->press_hpa = -1.0f;
	}
	else
	{
		sensor_data->press_hpa = ((float)press_q24_8 / 256.0f) / 100.0f;
	}
}
	
uint8_t SHT31_CheckSum_CRC8(uint8_t* Result,uint8_t num) 
{
	uint8_t data[2];
	uint8_t index=0;
	index=num;
	
	data[0] = Result[index];
	data[1] = Result[index+1];

	uint32_t POLYNOMIAL = 0x131;
	uint8_t crc = 0xFF;
	uint8_t bit = 0;
	uint8_t byteCtr = 0;

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < 2; ++byteCtr)
	{
		crc ^= (data[byteCtr]);
		for (bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
			else crc = (crc << 1);
		}
	}
	if (crc == Result[index+2]) {
		return 1;
	}
	else {
		return 0;
	}
}

void SHT31_Read(sht3x_data_t *sht3x_data)
{
	etError   error;
  uint8_t times = 0;
	
	do
	{
		times++;
	  error = SHT3X_GetTempAndHumiClkStretch(&sht3x_data->temp_sht, &sht3x_data->hum_sht, REPEATAB_HIGH, 50);
	}while(times < 4 && error != NO_ERROR);

		if(error != NO_ERROR || sht3x_data->temp_sht-gxht30_temp_record>30 || sht3x_data->temp_sht-gxht30_temp_record<-30
			|| sht3x_data->hum_sht<20 || sht3x_data->hum_sht>=100 || sht3x_data->hum_sht-gxht30_hum_record>20 || sht3x_data->hum_sht-gxht30_hum_record<-20)
		{
			error = SHT3X_SoftReset();
			times = 0;
			delay_ms(50);		
			do
			{
				times++;
				error = SHT3X_GetTempAndHumiClkStretch(&sht3x_data->temp_sht, &sht3x_data->hum_sht, REPEATAB_HIGH, 50);
			}while(times < 4 && error != NO_ERROR);
	  }
	
  if(error == NO_ERROR)
	{
		if(sht3x_data->temp_sht>125)
		{
			sht3x_data->temp_sht=125;
			sht3x_data->temp_sht=gxht30_temp_record;
		}
		else if(sht3x_data->temp_sht<-40)
		{
			sht3x_data->temp_sht=-40;
			sht3x_data->temp_sht=gxht30_temp_record;
		}
		
		if(sht3x_data->hum_sht>100)
		{
			sht3x_data->hum_sht=100;
			sht3x_data->hum_sht=gxht30_hum_record;
		}
		else if(sht3x_data->hum_sht<0)
		{
			sht3x_data->hum_sht=0;
			sht3x_data->hum_sht=gxht30_hum_record;
		}
		else if(sht3x_data->hum_sht==100)
		{
			sht3x_data->hum_sht=gxht30_hum_record;
		}
		
		gxht30_temp_record=sht3x_data->temp_sht;
		gxht30_hum_record=sht3x_data->hum_sht;
	}
	else
	{
		sht3x_data->temp_sht=3276.7;
		sht3x_data->hum_sht=6553.5;
	}
}

uint8_t check_sht31_connect(void)
{
	uint8_t rxdata[3] = {0xFF, 0xFF, 0xFF};
	uint8_t data[2] = {0xF3, 0x2D};		
	I2C_GPIO_MODE_Config();
	I2C_Write_Len(0x44,0x01,2,data);
	delay_ms(50);
	if(I2C_Read_Len(0x44,0x01,3,rxdata)==0 && SHT31_CheckSum_CRC8(rxdata,0)==1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t SHT20_CheckSum_CRC8(uint8_t* Result) 
{
	uint8_t data[2];
	data[0] = Result[0];
	data[1] = Result[1];

	uint32_t POLYNOMIAL = 0x131;
	uint8_t crc = 0;
	uint8_t bit = 0;
	uint8_t byteCtr = 0;

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < 2; ++byteCtr)
	{
		crc ^= (data[byteCtr]);
		for (bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
			else crc = (crc << 1);
		}
	}
	if (crc == Result[2]) {
		return 1;
	}
	else {
		return 0;
	}
}

float SHT20_RH(void)
{
	uint8_t txdata[1]={0xf5};//Humidity measurement
  uint8_t rxdata[3];
	uint16_t AD_code;
	float hum;
	uint8_t check_number=0;	
  bool read_status=1;	

	I2C_GPIO_MODE_Config();
	
	do
	{
		read_status=1;
		check_number++;
	  if(I2C_Write_Len(0x40,0x01,1,txdata)==1)
		{
			read_status=0;
			delay_ms(20);
		}
	}while(read_status==0&&check_number<4);
	
	if(read_status==1)
	{
		delay_ms(30);
	  check_number=0;
		do
		{
			read_status=1;
			check_number++;
			if(I2C_Read_Len(0x40,0x01,3,rxdata)==1)
			{
				read_status=0;
				delay_ms(20);
			}
			
			if(SHT20_CheckSum_CRC8(rxdata)==0)
			{
				read_status=0;
				delay_ms(20);
			}
		}while(read_status==0&&check_number<4);
	}

	if(read_status==1)
	{	
		AD_code=(rxdata[0]<<8)+rxdata[1];
		AD_code &=~0x0003;
		hum=(AD_code*125.0/65536.0)-6.0;
		
		if(hum>100)	
		{
			hum=100;
		}
		else if(hum<0)
		{		
		  hum=0;
		}		
	}
	else
	{
		hum=6553.5;
	}	
  return hum;	
}

float SHT20_RT(void)
{
	uint8_t txdata[1]={0xf3};//Humidity measurement
  uint8_t rxdata[3];
	uint16_t AD_code;
	float tem;
	uint8_t check_number=0;	
  bool read_status=1;	

	I2C_GPIO_MODE_Config();
	
	do
	{
		read_status=1;
		check_number++;
	  if(I2C_Write_Len(0x40,0x01,1,txdata)==1)
		{
			read_status=0;
			delay_ms(20);
		}
	}while(read_status==0&&check_number<4);
	
	if(read_status==1)
	{
		delay_ms(90);
	  check_number=0;
		do
		{
			read_status=1;
			check_number++;
			if(I2C_Read_Len(0x40,0x01,3,rxdata)==1)
			{
				read_status=0;
				delay_ms(20);
			}
			
			if(SHT20_CheckSum_CRC8(rxdata)==0)
			{
				read_status=0;
				delay_ms(20);
			}
		}while(read_status==0&&check_number<4);
	}
	
	if(read_status==1)
	{	
		AD_code=(rxdata[0]<<8)+rxdata[1];
		AD_code &=~0x0003;
		tem=(AD_code*175.72/65536.0)-46.85;
			
		if(tem>125)
		{
			tem=125;
		}
		else if(tem<-40)
		{
			tem=-40;
		}		
	}
	else
	{
		tem=3276.7;
	}	
  return tem;
}

uint8_t check_sht20_connect(void)
{
	/* Not zero-filled: the SHT20 CRC of 00 00 is 00. */
	uint8_t rxdata[3] = {0xFF, 0xFF, 0xFF};
	uint8_t data[1] = {0xF3};		
	I2C_GPIO_MODE_Config();
	I2C_Write_Len(0x40,0x01,1,data);
	delay_ms(90);
	if(I2C_Read_Len(0x40,0x01,3,rxdata)==0 && SHT20_CheckSum_CRC8(rxdata)==1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t check_sht45_connect(void)
{
	uint8_t rxdata[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	uint8_t reset_cmd[1] = {0x94};
	uint8_t cmd[1] = {0x89};
	I2C_GPIO_MODE_Config();
	I2C_Write_Len(0x44, 0x01, 1, reset_cmd);
	delay_ms(2);
	I2C_Write_Len(0x44, 0x01, 1, cmd);
	delay_ms(2);
	if(I2C_Read_Len(0x44, 0x01, 6, rxdata) == 0
		&& SHT31_CheckSum_CRC8(rxdata, 0) == 1 && SHT31_CheckSum_CRC8(rxdata, 3) == 1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void SHT45_Read(sensor_t *sensor_data)
{
	uint8_t cmd[1] = {0xFD};
	uint8_t rxdata[6];
	uint8_t times = 0;
	uint8_t success = 0;

	do
	{
		times++;
		I2C_GPIO_MODE_Config();
		I2C_Write_Len(0x44, 0x01, 1, cmd);
		delay_ms(10);
		I2C_Read_Len(0x44, 0x01, 6, rxdata);

		if(SHT31_CheckSum_CRC8(rxdata, 0) == 1 && SHT31_CheckSum_CRC8(rxdata, 3) == 1)
		{
			success = 1;
			break;
		}
		delay_ms(10);
	} while(times < 4);

	if(success)
	{
		uint16_t rawT = ((uint16_t)rxdata[0] << 8) | rxdata[1];
		uint16_t rawRH = ((uint16_t)rxdata[3] << 8) | rxdata[4];

		sensor_data->temp_sht = -45.0f + 175.0f * rawT / 65535.0f;
		sensor_data->hum_sht = -6.0f + 125.0f * rawRH / 65535.0f;

		if(sensor_data->temp_sht > 125.0f) sensor_data->temp_sht = 125.0f;
		else if(sensor_data->temp_sht < -40.0f) sensor_data->temp_sht = -40.0f;

		if(sensor_data->hum_sht > 100.0f) sensor_data->hum_sht = 100.0f;
		else if(sensor_data->hum_sht < 0.0f) sensor_data->hum_sht = 0.0f;
	}
	else
	{
		sensor_data->temp_sht = 3276.7f;
		sensor_data->hum_sht = 6553.5f;
	}
}

uint16_t bh1750_read(void)
{
	uint8_t rxdata[2];	
	uint16_t AD_code;
	uint16_t luminance;	
	
	I2C_GPIO_MODE_Config();
	
	delay_ms(10);//Required	
	I2C_Write_Byte(0x23,0x01); 
	I2C_Write_Byte(0x23,0x20);  
  delay_ms(200);	
  I2C_Read_Len(0x23,0x00,2,rxdata);
	I2C_Write_Byte(0x23,0x07);	

	if(iic_noack==1)
	{
		luminance=0;	
	}
	else
	{
		AD_code=(rxdata[0]<<8)+rxdata[1];
		luminance=AD_code/1.2;
  }
	
	iic_noack=0;
	
	return luminance;
}

void I2C_read_data(sensor_t *sensor_data,uint8_t flag_temp, uint8_t message)
{	
	I2C_GPIO_MODE_Config();
	if(flag_temp==0)
	{
		sensor_data->temp_sht=6553.5;
		sensor_data->hum_sht=6553.5;
	} 
	else if(flag_temp==1)
	{
		sensor_data->temp_sht=SHT20_RT();
		sensor_data->hum_sht=SHT20_RH(); 
		if(message==1)
		{
			LOG_PRINTF(LL_DEBUG,"SHT2x_temp:%.1f,SHT2x_hum:%.1f\r\n",sensor_data->temp_sht,sensor_data->hum_sht);
			delay_ms(20);
		}
	}
	else if(flag_temp==2)
	{			       	
		sht3x_data_t get_sht3x_data;
		SHT31_Read(&get_sht3x_data);				
		sensor_data->temp_sht=get_sht3x_data.temp_sht;
		sensor_data->hum_sht=get_sht3x_data.hum_sht;	
		if(message==1)
		{
			LOG_PRINTF(LL_DEBUG,"SHT3x_temp:%.1f,SHT3x_hum:%.1f\r\n",sensor_data->temp_sht,sensor_data->hum_sht);
			delay_ms(20);
		}
	}
	else if(flag_temp==3)
	{		
    bh1750flags=1;		
		sensor_data->illuminance=bh1750_read();		
		if(message==1)
		{			
			LOG_PRINTF(LL_DEBUG,"BH1750_lum:%d\r\n",sensor_data->illuminance);	
      delay_ms(20);			
		}				
	}
	else if(flag_temp==4)
	{
		SHT45_Read(sensor_data);
		if(message==1)
		{
			LOG_PRINTF(LL_DEBUG,"SHT4x_temp:%.1f,SHT4x_hum:%.1f\r\n",sensor_data->temp_sht,sensor_data->hum_sht);
			delay_ms(20);
		}
	}
	else if(flag_temp==5)
	{
		bme280flags=1;
		BME280_Read(sensor_data);
		if(message==1)
		{
			LOG_PRINTF(LL_DEBUG,"BME280_temp:%.1f,BME280_hum:%.1f,BME280_press:%.1f\r\n",
				sensor_data->temp_sht,sensor_data->hum_sht,sensor_data->press_hpa);
			delay_ms(20);
		}
	}
  I2C_GPIO_MODE_ANALOG();	
}

void LidarLite_init(void)
{
   uint8_t sigCountMax[1]={0x80};
   uint8_t acqConfigReg[1]={0x08};
   uint8_t refCountMax[1]={0x05};
   uint8_t thresholdBypass[1]={0x00};	
   waitbusy(1); 
	 I2C_Write_reg_Len(0x62,0x02,1,sigCountMax);
   waitbusy(1); 
	 I2C_Write_reg_Len(0x62,0x04,1,acqConfigReg);
   waitbusy(1);
   I2C_Write_reg_Len(0x62,0x12,1,refCountMax);	 
   waitbusy(1); 
	 I2C_Write_reg_Len(0x62,0x1c,1,thresholdBypass);	 
   waitbusy(1);	
}

uint16_t LidarLite(void)
{
	uint8_t dataByte[1]={0x04};
  uint8_t rxdata1[1];
	uint8_t rxdata2[1];
	uint16_t distance;
  waitbusy(1);
  I2C_Write_reg_Len(0x62,0x00,1,dataByte);
  if(waitbusy(2)<999)
	{		
		I2C_Read_reg_Len(0x62,0x0f,1,rxdata1);
		I2C_Read_reg_Len(0x62,0x10,1,rxdata2);
		distance=(rxdata1[0]<<8)+rxdata2[0];
		if(distance>4000)
		{
			distance=0;			
			return distance;			
		}
		else
		{
			return distance*10;	
		}			
	}
	else
	{
	 distance=65535;
	 return distance;			
	}	
}

uint16_t waitbusy(uint8_t mode)
{
  uint16_t busyCounter = 0;
	uint8_t busy[1]={0x01};
  while (busy[0])      
  {
   if((mode==1)&&(busyCounter > 99))
   {
		return busyCounter;			 
   }
	 
   if((mode==2)&&(busyCounter > 999))
   {
		return busyCounter;			 
   }	 
	 I2C_Read_reg_Len(0x62,0x01,1,busy);
	 busy[0] &=0x01;
	 busyCounter++;
  }
	return busyCounter;	
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
