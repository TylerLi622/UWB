/*! ----------------------------------------------------------------------------
 *  @file    instance_calib.c
 *  @brief   DecaWave application level configuration and calibration functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "compiler.h"
#include "deca_device_api.h"
#include "instance.h"
#include "port.h"

// -------------------------------------------------------------------------------------------------------------------

//The table below specifies the default TX spectrum configuration parameters... this has been tuned for DW EVK hardware units
//the table is set for smart power - see below in the instance_config function how this is used when not using smart power
const tx_struct txSpectrumConfig[8] =
{
	//Channel 0 ----- this is just a place holder so the next array element is channel 1
    {
            0x0,   //0
            {
                    0x0, //0
                    0x0 //0
            }
    },
    //Channel 1
    {
			0xc9,   //PG_DELAY
            {
                    0x15355575, //16M prf power
                    0x07274767 //64M prf power
            }

    },
    //Channel 2
    {
            0xc2,   //PG_DELAY
            {
                    0x15355575, //16M prf power
                    0x07274767 //64M prf power
            }
    },
    //Channel 3
    {
            0xc5,   //PG_DELAY
            {
                    0x0f2f4f6f, //16M prf power
                    0x2b4b6b8b //64M prf power
            }
    },
    //Channel 4
    {
            0x95,   //PG_DELAY
            {
                    0x1f1f3f5f, //16M prf power
                    0x3a5a7a9a //64M prf power
            }
    },
    //Channel 5
    {
            0xc0,   //PG_DELAY
            {
                    0x0E082848, //16M prf power
                    0x25456585 //64M prf power
            }
    },
    //Channel 6 ----- this is just a place holder so the next array element is channel 7
    {
            0x0,   //0
            {
                    0x0, //0
                    0x0 //0
            }
    },
    //Channel 7
    {
            0x93,   //PG_DELAY
            {
                    0x32527292, //16M prf power
                    0x5171B1d1 //64M prf power
            }
    }
};

//these are default antenna delays for EVB1000, these can be used if there is no calibration data in the DW1000,
//or instead of the calibration data
const uint16 rfDelays[2] = {
		(uint16) ((DWT_PRF_16M_RFDLY/ 2.0) * 1e-9 / DWT_TIME_UNITS),//PRF 16
		(uint16) ((DWT_PRF_64M_RFDLY/ 2.0) * 1e-9 / DWT_TIME_UNITS)
};


// -------------------------------------------------------------------------------------------------------------------
extern instance_data_t instance_data[NUM_INST] ;

// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------

int powertest(void)
{
	dwt_config_t    configData ;
	dwt_txconfig_t  configTx ;
	uint8 pow;
	uint8 msg[127] = "The quick brown fox jumps over the lazy dog. The quick brown fox jumps over the lazy dog. The quick brown fox jumps over the l"; 
	
	//NOTE: SPI frequency must be < 3MHz
	//reduce the SPI speed before switching to XTAL
	//
	//	reset device 
	//
	dwt_softreset();

	//
	//	configure channel paramters
	//
    configData.chan = 2 ;
    configData.rxCode =  9 ;
	configData.txCode = 9 ;
    configData.prf = DWT_PRF_64M ;
    configData.dataRate = DWT_BR_110K ;
    configData.txPreambLength = DWT_PLEN_2048 ;
    configData.rxPAC = DWT_PAC64 ;
    configData.nsSFD = 1 ;
	configData.smartPowerEn = 0;

	dwt_configure(&configData, DWT_LOADANTDLY | DWT_LOADXTALTRIM) ;

	configTx.PGdly = txSpectrumConfig[configData.chan].PGdelay ;

	if(configData.smartPowerEn == 0)
	{
		pow = txSpectrumConfig[configData.chan].txPwr[configData.prf - DWT_PRF_16M] & 0xFF ;
		configTx.power = (pow | (pow << 8) | (pow << 16) | (pow << 24));
	}
	else
	{
		configTx.power = txSpectrumConfig[configData.chan].txPwr[configData.prf - DWT_PRF_16M];
	}

	dwt_configuretxrf(&configTx);

	// the value here 0x1000 gives a period of 32.82 µs
	//this is setting 0x1000 as frame period (125MHz clock cycles) (time from Tx en - to next - Tx en)
	dwt_configcontinuousframemode(0x1000);

	dwt_writetxdata(127, (uint8 *)  msg, 0) ;
	dwt_writetxfctrl(127, 0);

    //to start the first frame - set TXSTRT
	dwt_starttx(DWT_START_TX_IMMEDIATE); 

	//measure the power 
	//Spectrum Analyser set:
	//FREQ to be channel default e.g. 3.9936 GHz for channel 2
	//SPAN to 1GHz
	//SWEEP TIME 1s
	//RBW and VBW 1MHz
	//measure channel power

	return DWT_SUCCESS ;
}

int instance_startcwmode(int chan)
{
	//NOTE: SPI frequency must be < 3MHz
	//reduce the SPI speed before switching to XTAL
	SPI_ConfigFastRate(SPI_BaudRatePrescaler_32); //reduce SPI to < 3MHz

	dwt_configcwmode(chan);

	//measure the frequency
	//Spectrum Analyser set:
	//FREQ to be channel default e.g. 3.9936 GHz for channel 2
	//SPAN to 10MHz
	//PEAK SEARCH

	return  DWT_SUCCESS ;
}

int instance_starttxtest(int framePeriod)
{
	//define some test data for the tx buffer
	uint8 msg[127] = "The quick brown fox jumps over the lazy dog. The quick brown fox jumps over the lazy dog. The quick brown fox jumps over the l"; 
	
	//NOTE: SPI frequency must be < 3MHz

	// the value here 0x1000 gives a period of 32.82 µs
	//this is setting 0x1000 as frame period (125MHz clock cycles) (time from Tx en - to next - Tx en)
	dwt_configcontinuousframemode(framePeriod);

	dwt_writetxdata(127, (uint8 *)  msg, 0) ;
	dwt_writetxfctrl(127, 0);

    //to start the first frame - set TXSTRT
	dwt_starttx(DWT_START_TX_IMMEDIATE); 

	//measure the power 
	//Spectrum Analyser set:
	//FREQ to be channel default e.g. 3.9936 GHz for channel 2
	//SPAN to 1GHz
	//SWEEP TIME 1s
	//RBW and VBW 1MHz
	//measure channel power

	return DWT_SUCCESS ;
}

void xtalcalibration(void)
{
	int i;
	uint8 chan = 2 ;
	uint8 prf = DWT_PRF_16M ;
	dwt_txconfig_t  configTx ;
	uint8 pow ;
	
	//NOTE: SPI frequency must be < 3MHz
	//reduce the SPI speed before switching to XTAL
	//
	//	reset device 
	//
	dwt_softreset();

	//
	//	configure TX channel parameters
	//

	configTx.PGdly = txSpectrumConfig[chan].PGdelay ;

	//Assume smart power is disabled - not relevant for XTAL trimming as CW mode
    pow = txSpectrumConfig[chan].txPwr[prf - DWT_PRF_16M] & 0xFF ;
	configTx.power = (pow | (pow << 8) | (pow << 16) | (pow << 24));

	dwt_configuretxrf(&configTx);

	dwt_configcwmode(chan);

	for(i=0; i<=0x1F; i++)
	{
		dwt_xtaltrim(i);
		//measure the frequency
		//Spectrum Analyser set:
		//FREQ to be channel default e.g. 3.9936 GHz for channel 2
		//SPAN to 10MHz
		//PEAK SEARCH
	}

	return;
}

/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
