/*! ----------------------------------------------------------------------------
 *  @file    instance_common.c
 *  @brief   DecaWave application level common instance functions
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
#include "port.h"
#include "deca_device_api.h"
#include "deca_spi.h"


#include "instance.h"


// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------

double inst_idist = 0;
double inst_idistraw = 0;
double inst_adist = 0;
double inst_ldist = 0;

instance_data_t instance_data[NUM_INST] ;
uint8 anchorNum = 1;


instance_localdata_t instance_localdata[NUM_INST] ;

//int eventOutcount = 0;
//int eventIncount = 0;

// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------
// convert microseconds to device time
uint64 convertmicrosectodevicetimeu (double microsecu)
{
    uint64 dt;
    long double dtime;

    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6 ;

    dt =  (uint64) (dtime) ;

    return dt;
}

// -------------------------------------------------------------------------------------------------------------------
// convert microseconds to device time
uint32 convertmicrosectodevicetimeu32 (double microsecu)
{
    uint32 dt;
    long double dtime;

    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6 ;

    dt =  (uint32) (dtime) ;

    return dt;
}


double convertdevicetimetosec(int32 dt)
{
    double f = 0;

    f =  dt * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

    return f ;
}

double convertdevicetimetosec8(uint8* dt)
{
    double f = 0;

    uint32 lo = 0;
    int8 hi = 0;

    memcpy(&lo, dt, 4);
    hi = dt[4] ;

    f = ((hi * 65536.00 * 65536.00) + lo) * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

    return f ;
}


void reportTOF(instance_data_t *inst)
{
        double distance ;
        double distance_to_correct ;
        double tof ;
        double ltave;
        int64 tofi ;

        // check for negative results and accept them making them proper negative integers
        tofi = inst->tof ;                          // make it signed
        if (tofi > 0x007FFFFFFFFF)                          // MP counter is 40 bits,  close up TOF may be negative
        {
            tofi -= 0x010000000000 ;                       // subtract fill 40 bit range to make it negative
        }

        // convert to seconds (as floating point)
        tof = convertdevicetimetosec(tofi);          //this is divided by 4 to get single time of flight
        inst_idistraw = distance = tof * SPEED_OF_LIGHT;

#if (CORRECT_RANGE_BIAS == 1)
        //for the 6.81Mb data rate we assume gating gain of 6dB is used,
        //thus a different range bias needs to be applied
        //if(inst->configData.dataRate == DWT_BR_6M8)
        if(inst->configData.smartPowerEn)
        {
        	//1.31 for channel 2 and 1.51 for channel 5
        	if(inst->configData.chan == 5)
        	{
        		distance_to_correct = distance/1.51;
        	}
        	else //channel 2
        	{
        		distance_to_correct = distance/1.31;
			}
        }
        else
        {
        	distance_to_correct = distance;
        }
        distance = distance - dwt_getrangebias(inst->configData.chan, (float) distance_to_correct, inst->configData.prf);
#endif

        if ((distance < 0) || (distance > 20000.000))    // discount any items with error
		{
            return;
		}

        inst_idist = distance;

        inst->longTermRangeSum+= distance ;
        inst->longTermRangeCount++ ;                          // for computing a long term average
        ltave = inst->longTermRangeSum / inst->longTermRangeCount ;

        inst_ldist = ltave ;

        inst->adist[inst->tofindex++] = distance;

        if(distance < inst->idistmin)
            inst->idistmin = distance;

        if(distance > inst->idistmax)
            inst->idistmax = distance;

        if(inst->tofindex == RTD_MED_SZ) inst->tofindex = 0;

        if(inst->tofcount == RTD_MED_SZ)
        {
            int i;
            double avg;

            avg = 0;
            for(i = 0; i < inst->tofcount; i++)
            {
                avg += inst->adist[i];
            }
            avg /= inst->tofcount;

            inst_adist = avg ;

        }
        else
            inst->tofcount++;
    return ;
}// end of reportTOF

// -------------------------------------------------------------------------------------------------------------------
//
// function to select the destination address (e.g. the address of the next anchor to poll)
//
// -------------------------------------------------------------------------------------------------------------------
//
int instaddtagtolist(instance_data_t *inst, uint8 *tagAddr)
{
    uint8 i;
    uint8 blank[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    inst->blinkRXcount++ ;

    //add the new Tag to the list, if not already there and there is space
    for(i=0; i<TAG_LIST_SIZE; i++)
    {
        if(memcmp(&inst->tagList[i][0], &tagAddr[0], 8) != 0)
        {
            if(memcmp(&inst->tagList[i][0], &blank[0], 8) == 0) //blank entry
            {
                memcpy(&inst->tagList[i][0], &tagAddr[0], 8) ;
                inst->tagListLen = i + 1 ;
                break;
            }
        }
        else
        {
            break; //we already have this Tag in the list
        }
    }

    return 0;
}


// -------------------------------------------------------------------------------------------------------------------
#if (NUM_INST != 1)
#error These functions assume one instance only
#else

void instcleartaglist(void)
{
    int instance = 0 ;
    uint8 blank[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    instance_data[instance].tagListLen = 0 ;
    instance_data[instance].blinkRXcount = 0 ;
    instance_data[instance].tagToRangeWith = 0;

    memcpy(&instance_data[instance].tagList[0][0], &blank[0], 8);
}


uint32 instgetblinkrxcount(void)
{
    int instance = 0 ;
    return instance_data[instance].blinkRXcount ;
}




// -------------------------------------------------------------------------------------------------------------------
// Set this instance role as the Tag, Anchor or Listener
void instancesetrole(int inst_mode)
{
    // assume instance 0, for this
    instance_data[0].mode =  inst_mode;                   // set the role
}

int instancegetrole(void)
{
    return instance_data[0].mode;
}

int instancenewrange(void)
{
    if(instance_data[0].newrange)
    {
        instance_data[0].newrange = 0;
        return 1;
    }

    return 0;
}

int instancenorange(void)
{
	int x = 0;
    if(instance_data[0].norange)
    {
        x = instance_data[0].norange ;
        instance_data[0].norange = 0;
    }

    return x;
}

int instancenewrangeancadd(void)
{
    return instance_data[0].newrangeancaddress;
}

int instancenewrangetagadd(void)
{
    return instance_data[0].newrangetagaddress;
}

int instanceanchorwaiting(void)
{
	return instance_data[0].canprintinfo;
}

int instancesleeping(void)
{
	if(instance_data[0].canprintinfo == 1)
	{
		instance_data[0].canprintinfo = 0; //clear flag
		return 1;
	}

	return 0 ;
}
// -------------------------------------------------------------------------------------------------------------------
// function to clear counts/averages/range values
//
void instanceclearcounts(void)
{
    int instance = 0 ;

    instance_data[instance].rxTimeouts = 0 ;

    instance_data[instance].frame_sn = 0;

    dwt_configeventcounters(1); //enable and clear - NOTE: the counters are not preserved when in DEEP SLEEP

    instance_data[instance].frame_sn = 0;

    instance_data[instance].tofcount = 0 ;
    instance_data[instance].tofindex = 0 ;

    instance_data[instance].txmsgcount = 0;
    instance_data[instance].rxmsgcount = 0;
    instance_data[instance].lateTX = 0;
    instance_data[instance].lateRX = 0;

    instance_data[instance].longTermRangeSum  = 0;
    instance_data[instance].longTermRangeCount  = 0;

    instance_data[instance].idistmax = 0;
    instance_data[instance].idistmin = 1000;

    instcleartaglist();

} // end instanceclearcounts()


// -------------------------------------------------------------------------------------------------------------------
// function to initialise instance structures
//
// Returns 0 on success and -1 on error
int instance_init(void)
{
    int instance = 0 ;
    int result;
    //uint16 temp = 0;

    instance_data[instance].mode =  ANCHOR;                                // assume listener,

    instance_data[instance].instToSleep = 0;

    instance_data[instance].tofindex = 0;
    instance_data[instance].tofcount = 0;
    instance_data[instance].tof = 0;

    // Reset the IC (might be needed if not getting here from POWER ON)
    // ARM code: Remove soft reset here as using hard reset in the inittestapplication() in the main.c file
    //dwt_softreset();


	//we can enable any configuration loding from OTP/ROM on initialisation
    result = dwt_initialise(DWT_LOADUCODE | DWT_LOADLDOTUNE | DWT_LOADTXCONFIG | DWT_LOADANTDLY| DWT_LOADXTALTRIM) ;

    //temp = dwt_readtempvbat();
	//  temp formula is: 1.13 * reading - 113.0
	// Temperature (°C )= (SAR_LTEMP - (OTP_READ(Vtemp @ 23 °C )) x 1.14) + 23
	//  volt formula is: 0.0057 * reading + 2.3
	// Voltage (volts) = (SAR_LVBAT- (OTP_READ(Vmeas @ 3.3 V )) /173) + 3.3
	//printf("Vbat = %d (0x%02x) %1.2fV\tVtemp = %d  (0x%02x) %2.2fC\n",temp&0xFF,temp&0xFF, ((0.0057 * (temp&0xFF)) + 2.3), (temp>>8)&0xFF,(temp>>8)&0xFF, ((1.13 * ((temp>>8)&0xFF)) - 113.0));

    //this is platform dependant - only program if DW EVK/EVB
    dwt_setleds(3) ; //configure the GPIOs which control the leds on EVBs

    if (DWT_SUCCESS != result)
    {
        return (-1) ;   // device initialise has failed
    }


    instanceclearcounts() ;

    instance_data[instance].sleep_en = 1;
    instance_data[instance].panid = 0xdeca ;
    instance_data[instance].wait4ack = 0;
    instance_data[instance].stoptimer = 0;
    instance_data[instance].instancetimer_en = 0;

    instance_clearevents();

    dwt_geteui(instance_data[instance].eui64);

    instance_localdata[instance].testapprun_fn = NULL;
    instance_data[instance].canprintinfo = 0;

    instance_data[instance].clockOffset = 0;
    instance_data[instance].monitor = 0;
    return 0 ;
}

// -------------------------------------------------------------------------------------------------------------------
//
// Return the Device ID register value, enables higher level validation of physical device presence
//

uint32 instancereaddeviceid(void)
{
    return dwt_readdevid() ;
}

#if (REG_DUMP == 1)
void dwt_dumpregisters(char *str, size_t strSize)
{
    uint32 reg = 0;
    uint8 buff[5];
    int i;
    int cnt ;

#if (0)
    //first print all single registers
    for(i=0; i<0x3F; i++)
    {
        dwt_readfromdevice(i, 0, 5, buff) ;
        str += cnt = sprintf(str,"reg[%02X]=%02X%02X%02X%02X%02X",i,buff[4], buff[3], buff[2], buff[1], buff[0] ) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x20
    for(i=0; i<=32; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x20,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x20,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x21
    for(i=0; i<=44; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x21,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x21,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x23
    for(i=0; i<=0x20; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x23,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x23,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }
#else
    //reg 0x24
    for(i=0; i<=12; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x24,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x24,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x27
    for(i=0; i<=44; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x27,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x27,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x28
    for(i=0; i<=64; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x28,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x28,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2A
    for(i=0; i<20; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2A,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2A,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2B
    for(i=0; i<24; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2B,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2B,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2f
    for(i=0; i<40; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2f,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2f,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x31
    for(i=0; i<84; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x31,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x31,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x36 = PMSC_ID
    for(i=0; i<=48; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x36,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x36,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }
#endif
}
#endif

// -------------------------------------------------------------------------------------------------------------------
//
// function to allow application configuration be passed into instance and affect underlying device opetation
//
void instance_config(instanceConfig_t *config)
{
    int instance = 0 ;
    int use_otpdata = DWT_LOADANTDLY | DWT_LOADXTALTRIM;
    uint32 power = 0;

    instance_data[instance].configData.chan = config->channelNumber ;
    instance_data[instance].configData.rxCode =  config->preambleCode ;
    instance_data[instance].configData.txCode = config->preambleCode ;
    instance_data[instance].configData.prf = config->pulseRepFreq ;
    instance_data[instance].configData.dataRate = config->dataRate ;
    instance_data[instance].configData.txPreambLength = config->preambleLen ;
    instance_data[instance].configData.rxPAC = config->pacSize ;
    instance_data[instance].configData.nsSFD = config->nsSFD ;
    instance_data[instance].configData.phrMode = DWT_PHRMODE_STD ;
    instance_data[instance].configData.sfdTO = config->sfdTO;

    //configure the channel parameters
    dwt_configure(&instance_data[instance].configData, use_otpdata) ;

    instance_data[instance].configTX.PGdly = txSpectrumConfig[config->channelNumber].PGdelay ;

    //firstly check if there are calibrated TX power value in the DW1000 OTP
    power = dwt_getotptxpower(config->pulseRepFreq, instance_data[instance].configData.chan);

    if((power == 0x0) || (power == 0xFFFFFFFF)) //if there are no calibrated values... need to use defaults
    {
        power = txSpectrumConfig[config->channelNumber].txPwr[config->pulseRepFreq- DWT_PRF_16M];
    }

    instance_data[instance].configTX.power = power;

    //configure the tx spectrum parameters (power and PG delay)
    dwt_configuretxrf(&instance_data[instance].configTX);

    instance_data[instance].antennaDelayChanged = 0;

    //check if to use the antenna delay calibration values as read from the OTP
    if((use_otpdata & DWT_LOADANTDLY) == 0)
    {
    	instance_data[instance].rxantennaDelay = instance_data[instance].txantennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
        // -------------------------------------------------------------------------------------------------------------------
        // set the antenna delay, we assume that the RX is the same as TX.
        dwt_setrxantennadelay(instance_data[instance].txantennaDelay);
        dwt_settxantennadelay(instance_data[instance].txantennaDelay);
    }
    else
    {
        //get the antenna delay that was read from the OTP calibration area
    	instance_data[instance].rxantennaDelay = instance_data[instance].txantennaDelay = dwt_readantennadelay(config->pulseRepFreq) >> 1;

        // if nothing was actually programmed then set a reasonable value anyway
        if (instance_data[instance].txantennaDelay == 0)
        {
        	instance_data[instance].rxantennaDelay = instance_data[instance].txantennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
            // -------------------------------------------------------------------------------------------------------------------
            // set the antenna delay, we assume that the RX is the same as TX.
            dwt_setrxantennadelay(instance_data[instance].txantennaDelay);
            dwt_settxantennadelay(instance_data[instance].txantennaDelay);
        }


    }

    if(config->preambleLen == DWT_PLEN_64) //if preamble length is 64
	{
    	SPI_ConfigFastRate(SPI_BaudRatePrescaler_32); //reduce SPI to < 3MHz

		dwt_loadopsettabfromotp(0);

		SPI_ConfigFastRate(SPI_BaudRatePrescaler_4); //increase SPI to max
    }

#if (REG_DUMP == 1)
#define REG_BUF_SIZE    (100*30)
{
	char regDumpBuffer[REG_BUF_SIZE] ;
	dwt_dumpregisters(regDumpBuffer, REG_BUF_SIZE);

	{
		char* buff[REG_BUF_SIZE+10];
		int n = sprintf((char*)&buff[0], "%s", regDumpBuffer);
		send_usbmessage(&regDumpBuffer[0], n);
	}
}
#endif
}

// -------------------------------------------------------------------------------------------------------------------
// function to set the tag sleep time (in ms)
//
void instancesettagsleepdelay(int sleepdelay, int blinksleepdelay) //sleep in ms
{
    int instance = 0 ;
    instance_data[instance].tagSleepTime_ms = sleepdelay ;
    instance_data[instance].tagBlinkSleepTime_ms = blinksleepdelay ;
}



int instance_get_dly(void) //get antenna delay
{
    int x = instance_data[0].txantennaDelay;

    return (x);
}


// -------------------------------------------------------------------------------------------------------------------
double instance_get_ldist(void) //get long term average range
{
    double x = inst_ldist;

    return (x);
}

int instance_get_lcount(void) //get count of ranges used for calculation of lt avg
{
    int x = instance_data[0].longTermRangeCount;

    return (x);
}

double instance_get_idist(void) //get instantaneous range
{
    double x = inst_idist;

    return (x);
}

double instance_get_idistraw(void) //get instantaneous range
{
    double x = inst_idistraw;

    return (x);
}

int instance_get_rxf(void) //get number of Rxed frames
{
    int x = instance_data[0].rxmsgcount;

    return (x);
}

int instance_get_txf(void) //get number of Txed frames
{
    int x = instance_data[0].txmsgcount;

    return (x);
}

int instance_get_txl(void) //get number of late Tx frames
{
    int x = instance_data[0].lateTX;

    return (x);
}

int instance_get_rxl(void) //get number of late Tx frames
{
    int x = instance_data[0].lateRX;

    return (x);
}

double instance_get_adist(void) //get average range
{
    double x = inst_adist;

    return (x);
}

int instance_get_respPSC(void)
{
	int x = instance_data[0].respPSC;

	instance_data[0].respPSC = 0;

	return x;
}



void inst_processrxtimeout(instance_data_t *inst)
{

	//inst->responseTimeouts ++ ;
    inst->rxTimeouts ++ ;
    inst->done = INST_NOT_DONE_YET;

    if(inst->mode == ANCHOR) //we did not receive the final/ACK - wait for next poll
    {
		//only enable receiver when not using double buffering
		inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
		dwt_setrxtimeout(0);
    }
	else //if(inst->mode == TAG)
    {
		// initiate the re-transmission of the poll that was not responded to
		inst->testAppState = TA_TXE_WAIT ;

		if(inst->mode == TAG)
		{
			if(inst->previousState == TA_TXFINAL_WAIT_SEND) //no report
			{
				inst->norange = 2;
			}
			if(inst->previousState == TA_TXPOLL_WAIT_SEND) //no response
			{
				inst->norange = 1;
			}
			inst->nextState = TA_TXPOLL_WAIT_SEND ;

		}
		else //TAG_TDOA
		{
			inst->nextState = TA_TXBLINK_WAIT_SEND ;
		}

    }

    //timeout - disable the radio (if using SW timeout the rx will not be off)
    dwt_forcetrxoff() ;
}


void instance_txcallback(const dwt_callback_data_t *txd)
{
	int instance = 0;
	uint8 txTimeStamp[5] = {0, 0, 0, 0, 0};

	uint8 txevent = txd->event;
	event_data_t dw_event;

	if(txevent == DWT_SIG_TX_DONE)
	{
		//uint64 txtimestamp = 0;

		//NOTE - we can only get TX good (done) while here
		//dwt_readtxtimestamp((uint8*) &instance_data[instance].txu.txTimeStamp);

		dwt_readtxtimestamp(txTimeStamp) ;
		dw_event.timeStamp32l = (uint32)txTimeStamp[0] + ((uint32)txTimeStamp[1] << 8) + ((uint32)txTimeStamp[2] << 16) + ((uint32)txTimeStamp[3] << 24);
		dw_event.timeStamp = txTimeStamp[4];
	    dw_event.timeStamp <<= 32;
		dw_event.timeStamp += dw_event.timeStamp32l;
		dw_event.timeStamp32h = ((uint32)txTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);

		instance_data[instance].stoptimer = 0;

		dw_event.rxLength = 0;
		dw_event.type2 = dw_event.type = DWT_SIG_TX_DONE ;

		instance_putevent(dw_event);

#if (DEEP_SLEEP == 1)
        if (instance_data[instance].sleep_en)
            instance_data[instance].txmsgcount++;
#endif
		//printf("TX time %f ecount %d\n",convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp), instance_data[instance].dweventCnt);
		//printf("TX Timestamp: %4.15e\n", convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp));

	}
	else if(txevent == DWT_SIG_TX_AA_DONE)
	{
		//auto ACK confirmation
		dw_event.rxLength = 0;
		dw_event.type2 = dw_event.type = DWT_SIG_TX_AA_DONE ;

		instance_putevent(dw_event);

		//printf("TX AA time %f ecount %d\n",convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp), instance_data[instance].dweventCnt);
	}

	instance_data[instance].monitor = 0;
}

void instance_rxcallback(const dwt_callback_data_t *rxd)
{
	int instance = 0;
	uint8 rxTimeStamp[5]  = {0, 0, 0, 0, 0};
	uint8 srcAddr_index = 0;
    uint8 rxd_event = 0;
	uint8 fcode_index  = 0;
	event_data_t dw_event;

	//if we got a frame with a good CRC - RX OK
    if(rxd->event == DWT_SIG_RX_OKAY)
	{


        rxd_event = DWT_SIG_RX_OKAY;

		dw_event.rxLength = rxd->datalength;

		//need to process the frame control bytes to figure out what type of frame we have received
        switch(rxd->fctrl[0])
	    {
			//blink type frame
		    case 0xC5:
				if(rxd->datalength == 12)
				{
					rxd_event = SIG_RX_BLINK;
				}
				else if (rxd->datalength == 13){
					rxd_event = SIG_RX_BLINK;
				}
				else if(rxd->datalength == 18)//blink with Temperature and Battery level indication
				{
					rxd_event = SIG_RX_BLINK;
				}
				else
					rxd_event = SIG_RX_UNKNOWN;
					break;

			//ACK type frame - not supported in this SW - set as unknown (re-enable RX)
			case 0x02:
				rxd_event = SIG_RX_UNKNOWN;
			    break;

			//data type frames (with/without ACK request) - assume PIDC is on.
			case 0x41:
			case 0x61:
				//read the frame
				if(rxd->datalength > STANDARD_FRAME_SIZE)
					rxd_event = SIG_RX_UNKNOWN;

				//need to check the destination/source address mode
				if((rxd->fctrl[1] & 0xCC) == 0x88) //dest & src short (16 bits)
				{
					fcode_index = FRAME_CRTL_AND_ADDRESS_S; //function code is in first byte after source address
					srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_S;
				}
				else if((rxd->fctrl[1] & 0xCC) == 0xCC) //dest & src long (64 bits)  //decawave only used Long dest & src address
				{
					fcode_index = FRAME_CRTL_AND_ADDRESS_L; //function code is in first byte after source address
					srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_L;
				}
				else //using one short/one long
				{
					fcode_index = FRAME_CRTL_AND_ADDRESS_LS; //function code is in first byte after source address

					if(((rxd->fctrl[1] & 0xCC) == 0x8C)) //source short
					{
						srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_L;
					}
					else
					{
						srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_S;
					}
				}
				break;

			//any other frame types are not supported by this application
			default:
				rxd_event = SIG_RX_UNKNOWN;
				break;
		}


		//read rx timestamp
		if((rxd_event == SIG_RX_BLINK) || (rxd_event == DWT_SIG_RX_OKAY))
		{
			dwt_readrxtimestamp(rxTimeStamp) ;
			dw_event.timeStamp32l =  (uint32)rxTimeStamp[0] + ((uint32)rxTimeStamp[1] << 8) + ((uint32)rxTimeStamp[2] << 16) + ((uint32)rxTimeStamp[3] << 24);
			dw_event.timeStamp = rxTimeStamp[4];
			dw_event.timeStamp <<= 32;
			dw_event.timeStamp += dw_event.timeStamp32l;
			dw_event.timeStamp32h = ((uint32)rxTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);

			dwt_readrxdata((uint8 *)&dw_event.msgu.frame[0], rxd->datalength, 0);  // Read Data Frame
		}

		dw_event.type2 = dw_event.type = rxd_event;

		//----------------------------------------------------------------------------------------------
		//TWR - here we chack if we need to respond to a TWR Poll or Response Messages
		//----------------------------------------------------------------------------------------------

		//Process good frames
		if(rxd_event == DWT_SIG_RX_OKAY)
		{
			//check if this is a TWR message (and also which one)
			if(instance_data[instance].tagListLen > 0)
			{
				switch(dw_event.msgu.frame[fcode_index])
				{

					case RTLS_DEMO_MSG_TAG_POLL:
					{
						uint16 frameLength = 0;

						instance_data[instance].tagPollRxTime = dw_event.timeStamp ; //Poll's Rx time

	#if (IMMEDIATE_RESPONSE == 0)
						instance_data[instance].delayedReplyTime = (instance_data[instance].tagPollRxTime + instance_data[instance].responseReplyDelay) >> 8 ;  // time we should send the response
    #else
                        instance_data[instance].delayedReplyTime = 0;
	#endif

    #if (USING_64BIT_ADDR == 1)
                        frameLength = ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
						memcpy(&instance_data[instance].msg.destAddr[0], &dw_event.msgu.frame[srcAddr_index], ADDR_BYTE_SIZE_L); //remember who to send the reply to (set destination address)
	#else
						frameLength = ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
						memcpy(&instance_data[instance].msg.destAddr[0], &dw_event.msgu.frame[srcAddr_index], ADDR_BYTE_SIZE_S); //remember who to send the reply to (set destination address)
	#endif
						// Write calculated TOF into response message
						memcpy(&(instance_data[instance].msg.messageData[TOFR]), &instance_data[instance].tof, 5);

						instance_data[instance].tof = 0; //clear ToF ..

						instance_data[instance].msg.seqNum = instance_data[instance].frame_sn++;

						//set the delayed rx on time (the final message will be sent after this delay)
						dwt_setrxaftertxdelay((uint32)instance_data[instance].txToRxDelayAnc_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

						//response is expected
						instance_data[instance].wait4ack = DWT_RESPONSE_EXPECTED;

						dwt_writetxfctrl(frameLength, 0);
						dwt_writetxdata(frameLength, (uint8 *)  &instance_data[instance].msg, 0) ;	// write the frame data

	#if (IMMEDIATE_RESPONSE == 1)
						dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	#else
						if(instancesendpacket(frameLength, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED, instance_data[instance].delayedReplyTime))
						{
							dw_event.type3 = DWT_SIG_TX_ERROR ;
							dwt_setrxaftertxdelay(0);
							instance_data[instance].wait4ack = 0; //clear the flag as the TX has failed the TRX is off
							instance_data[instance].lateTX++;

						}
						else
	#endif
						{
							dw_event.type3 = DWT_SIG_TX_PENDING ; // exit this interrupt and notify the application/instance that TX is in progress.
						}
					}
					break;

					case RTLS_DEMO_MSG_ANCH_RESP:
					{

					}
					break;

					default: //process rx frame
					break;
				}
			}

	    	instance_data[instance].stoptimer = 1;

	    	instance_putevent(dw_event);

			//printf("RX OK %d %x\n",instance_data[instance].testAppState, instance_data[instance].rxmsg.messageData[FCODE]);
			//printf("RX OK %d ", instance_data[instance].testAppState);
			//printf("RX time %f ecount %d\n",convertdevicetimetosecu(dw_event.timeStamp), instance_data[instance].dweventCnt);

#if (DEEP_SLEEP == 1)
            if (instance_data[instance].sleep_en)
                instance_data[instance].rxmsgcount++;
#endif
		}
		else if (rxd_event == SIG_RX_BLINK)
		{
			instance_putevent(dw_event);

#if (DEEP_SLEEP == 1)
            if (instance_data[instance].sleep_en)
                instance_data[instance].rxmsgcount++;
#endif
		}

		if (rxd_event == SIG_RX_UNKNOWN) //need to re-enable the rx
		{
			instancerxon(&instance_data[instance], 0, 0); //immediate enable
		}
	}
	else if (rxd->event == DWT_SIG_RX_TIMEOUT)
	{
		dw_event.type2 = dw_event.type = DWT_SIG_RX_TIMEOUT;
		dw_event.rxLength = 0;
		dw_event.timeStamp = 0;
		dw_event.timeStamp32l = 0;
		dw_event.timeStamp32h = 0;

		instance_putevent(dw_event);
		//printf("RX timeout while in %d\n", instance_data[instance].testAppState);
	}
	else //assume other events are errors
	{
		//printf("RX error %d \n", instance_data[instance].testAppState);
		//re-enable the receiver
		//for ranging application rx error frame is same as TO - as we are not going to get the expected frame
		if((instance_data[instance].mode == TAG) || (instance_data[instance].mode == TAG_TDOA))
		{
			dw_event.type = DWT_SIG_RX_TIMEOUT;
			dw_event.type2 = 0x40 | DWT_SIG_RX_TIMEOUT;
			dw_event.rxLength = 0;

			instance_putevent(dw_event);
		}
		else
		{
			instancerxon(&instance_data[instance], 0, 0); //immediate enable if anchor or listener
		}
	}
}


int instance_peekevent(void)
{
	int instance = 0;
    return instance_data[instance].dwevent[instance_data[instance].dweventPeek].type; //return the type of event that is in front of the queue
}

void instance_putevent(event_data_t newevent)
{
	int instance = 0;
	uint8 etype = newevent.type;

	newevent.type = 0;
	//newevent.eventtime = portGetTickCount();
	//newevent.gotit = newevent.eventtimeclr = 0;

	//copy event
	instance_data[instance].dwevent[instance_data[instance].dweventIdxIn] = newevent;

	//set type - this makes it a new event (making sure the event data is copied before event is set as new)
	//to make sure that the get event function does not get an incomplete event
	instance_data[instance].dwevent[instance_data[instance].dweventIdxIn].type = etype;

	instance_data[instance].dweventIdxIn++;

	if(MAX_EVENT_NUMBER == instance_data[instance].dweventIdxIn)
		instance_data[instance].dweventIdxIn = 0;

	//eventIncount++;

	//printf("put %d - in %d out %d @ %d\n", newevent.type, instance_data[instance].dweventCntIn, instance_data[instance].dweventCntOut, ptime);
}

event_data_t dw_event_g;

//#pragma GCC optimize ("O0")
event_data_t* instance_getevent(int x)
{
	int instance = 0;
	int indexOut = instance_data[instance].dweventIdxOut;

	//dw_event_g = instance_data[instance].dwevent[instance_data[instance].dweventCntOut]; //this holds any TX/RX events

	//memcpy(&dw_event_g, &instance_data[instance].dwevent[instance_data[instance].dweventCntOut], sizeof(event_data_t));

	if(instance_data[instance].dwevent[indexOut].type == 0) //exit with "no event"
	{
		dw_event_g.type = 0;
		dw_event_g.type2 = 0;
		return &dw_event_g;
	}

	//copy the event
	dw_event_g.type2 = instance_data[instance].dwevent[indexOut].type2 ;
	dw_event_g.type3 = instance_data[instance].dwevent[indexOut].type3 ;
	dw_event_g.rxLength = instance_data[instance].dwevent[indexOut].rxLength ;
	dw_event_g.timeStamp = instance_data[instance].dwevent[indexOut].timeStamp ;
	dw_event_g.timeStamp32l = instance_data[instance].dwevent[indexOut].timeStamp32l ;
	dw_event_g.timeStamp32h = instance_data[instance].dwevent[indexOut].timeStamp32h ;
	//dw_event_g.eventtime = instance_data[instance].dwevent[indexOut].eventtime ;
	//dw_event_g.eventtimeclr = instance_data[instance].dwevent[indexOut].eventtimeclr ;
	//dw_event_g.gotit = instance_data[instance].dwevent[indexOut].gotit ;

	memcpy(&dw_event_g.msgu, &instance_data[instance].dwevent[indexOut].msgu, sizeof(instance_data[instance].dwevent[indexOut].msgu));

	dw_event_g.type = instance_data[instance].dwevent[indexOut].type ;


	//instance_data[instance].dwevent[indexOut].gotit = x;

	//instance_data[instance].dwevent[indexOut].eventtimeclr = portGetTickCount();

	instance_data[instance].dwevent[indexOut].type = 0; //clear the event

	instance_data[instance].dweventIdxOut++;
	if(MAX_EVENT_NUMBER == instance_data[instance].dweventIdxOut) //wrap the counter
		instance_data[instance].dweventIdxOut = 0;

	instance_data[instance].dweventPeek = instance_data[instance].dweventIdxOut; //set the new peek value

	//if(dw_event.type) printf("get %d - in %d out %d @ %d\n", dw_event.type, instance_data[instance].dweventCntIn, instance_data[instance].dweventCntOut, ptime);

	//eventOutcount++;


	return &dw_event_g;
}

void instance_clearevents(void)
{
	int i = 0;
	int instance = 0;

	for(i=0; i<MAX_EVENT_NUMBER; i++)
	{
        memset(&instance_data[instance].dwevent[i], 0, sizeof(event_data_t));
	}

	instance_data[instance].dweventIdxIn = 0;
	instance_data[instance].dweventIdxOut = 0;
	instance_data[instance].dweventPeek = 0;

	//eventOutcount = 0;
	//eventIncount = 0;
}

void instance_setapprun(int (*apprun_fn)(instance_data_t *inst, int message))
{
	int instance = 0 ;
	instance_localdata[instance].testapprun_fn = apprun_fn;
}

// -------------------------------------------------------------------------------------------------------------------
int instance_run(void)
{
    int instance = 0 ;
    int done = INST_NOT_DONE_YET;
    int message = instance_peekevent(); //get any of the received events from ISR


	while(done == INST_NOT_DONE_YET)
	{
		//int state = instance_data[instance].testAppState;
		done = instance_localdata[instance].testapprun_fn(&instance_data[instance], message) ;                                               // run the communications application

		//we've processed message
		message = 0;
	}

    if(done == INST_DONE_WAIT_FOR_NEXT_EVENT_TO) //we are in RX and need to timeout (Tag needs to send another poll if no Rx frame)
    {
        if(instance_data[instance].mode == TAG) //Tag (is either in RX or sleeping)
        {
            instance_data[instance].instancetimer = instance_data[instance].instancetimer_saved + instance_data[instance].tagSleepTime_ms; //set timeout time
            instance_data[instance].instancetimer_en = 1; //start timer
        }
        if(instance_data[instance].mode == TAG_TDOA)
        {
            instance_data[instance].instancetimer += instance_data[instance].tagBlinkSleepTime_ms; //set timeout time
            instance_data[instance].instancetimer_en = 1; //start timer
        }
        instance_data[instance].stoptimer = 0 ; //clear the flag - timer can run if instancetimer_en set (set above)
        instance_data[instance].done = INST_NOT_DONE_YET;
    }

    //check if timer has expired
    if((instance_data[instance].instancetimer_en == 1) && (instance_data[instance].stoptimer == 0))
    {
        if(instance_data[instance].instancetimer < portGetTickCount())
        {
			event_data_t dw_event;
            instance_data[instance].instancetimer_en = 0;
			dw_event.rxLength = 0;
			dw_event.type = DWT_SIG_RX_TIMEOUT;
			dw_event.type2 = 0x80 | DWT_SIG_RX_TIMEOUT;
			//printf("PC timeout DWT_SIG_RX_TIMEOUT\n");
			instance_putevent(dw_event);
        }
    }

    return 0 ;
}


void instance_close(void)
{
    //wake up device from low power mode
    //NOTE - in the ARM  code just drop chip select for 200us
    port_SPIx_clear_chip_select();  //CS low
    Sleep(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
    port_SPIx_set_chip_select();  //CS high
    Sleep(5);
    dwt_entersleepaftertx(0); // clear the "enter deep sleep after tx" bit

    dwt_setinterrupt(0xFFFFFFFF, 0); //don't allow any interrupts

}


void instance_notify_DW1000_inIDLE(int idle)
{
	instance_data[0].dwIDLE = idle;
}


void instanceconfigantennadelays(uint16 tx, uint16 rx)
{
	instance_data[0].txantennaDelay = tx ;
	instance_data[0].rxantennaDelay = rx ;

	instance_data[0].antennaDelayChanged = 1;
}

void instancesetantennadelays(void)
{
	if(instance_data[0].antennaDelayChanged == 1)
	{
		dwt_setrxantennadelay(instance_data[0].rxantennaDelay);
		dwt_settxantennadelay(instance_data[0].txantennaDelay);

		instance_data[0].antennaDelayChanged = 0;
	}
}


uint16 instancetxantdly(void)
{
	return instance_data[0].txantennaDelay;
}

uint16 instancerxantdly(void)
{
	return instance_data[0].rxantennaDelay;
}

#endif


/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
