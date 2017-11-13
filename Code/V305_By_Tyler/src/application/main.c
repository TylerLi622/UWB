/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   main loop for the DecaRanging application
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
/* Includes */
#include "compiler.h"
#include "port.h"

#include "instance.h"


#include "deca_types.h"

#include "deca_spi.h"
#include "stdio.h"
#include "stdint.h"

extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(int, uint8*, int);
extern void send_usbmessage(uint8*, int);


#define DWINTERRUPT_EN (1)  //set to 1 when using DW interrupt, set to 0 to poll DW1000 IRQ line

#define SOFTWARE_VER_STRING    "Version 3.05    " //


#define SWS1_SRM_MODE 0x80  //slow ranging mode with PC - response is configurable and > 150 ms
#define SWS1_ANC_MODE 0x08  //anchor mode
#define SWS1_SHF_MODE 0x10  //short frame mode (6.81M) (switch S1-5)
#define SWS1_64M_MODE 0x20  // 64M PRF mode (switch S1-6)
#define SWS1_CH5_MODE 0x40  //channel 5 mode (switch S1-7)

int instance_anchaddr = 0; //0 = 0xDECA020000000001; 1 = 0xDECA020000000002; 2 = 0xDECA020000000003
//NOTE: switches TA_SW1_7 and TA_SW1_8 are used to set tag/anchor address
int dr_mode = 0;
//if instance_mode = TAG_TDOA then the device cannot be selected as anchor
int instance_mode = ANCHOR;
//int instance_mode = TAG;
//int instance_mode = TAG_TDOA;
//int instance_mode = LISTENER;


//NOTE: Set s1switch value to select work mode.
//NOTE: Will use TA_SW1_4 to set the working mode of Module.
//uint8 s1switch = 0;
uint8 s1switch = 0x25;  //mode = tag
//uint8 s1switch = 0x2D;  //mode = anchor node.




int chan, tagaddr, ancaddr;

#define LCD_BUFF_LEN (100)
uint8 dataseq[LCD_BUFF_LEN];
uint8 dataseq1[LCD_BUFF_LEN];

int ranging = 0;

typedef struct
{
    uint8 channel ;
    uint8 prf ;
    uint8 datarate ;
    uint8 preambleCode ;
    uint8 preambleLength ;
    uint8 pacSize ;
    uint8 nsSFD ;
    uint16 sfdTO ;
} chConfig_t ;


//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
chConfig_t chConfig[8] ={
                    //mode 1 - S1: 7 off, 6 off, 5 off
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 2
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 3
                    {
                        2,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 4
                    {
                        2,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 5
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 6
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 7
                    {
                        5,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 8
                    {
                        5,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    }
};


uint32 inittestapplication(uint8 s1switch);

// Restart and re-configure
void restartinstance(void)
{
    instance_close() ;                          //shut down instance, PHY, SPI close, etc.

    spi_peripheral_init();                      //re initialise SPI...

    inittestapplication(s1switch) ;                     //re-initialise instance/device
} // end restartinstance()

//NOTE: Channel 5 is not supported for the non-discovery mode
int decarangingmode(uint8 s1switch)
{
    int mode = 0;

    if(s1switch & SWS1_SHF_MODE)
    {
        mode = 1;
    }

    if(s1switch & SWS1_64M_MODE)
    {
        mode = mode + 2;
    }
    if(s1switch & SWS1_CH5_MODE)
    {
        mode = mode + 4;
    }

    return mode;
}

uint32 inittestapplication(uint8 s1switch)
{
    uint32 devID ;
    instanceConfig_t instConfig;
    int result;

    SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);  //max SPI before PLLs configured is ~4M

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instancereaddeviceid() ;
    if(DWT_DEVICE_ID != devID) //if the read of device ID fails, the DW1000 could be asleep
    {
        port_SPIx_clear_chip_select();  //CS low
        Sleep(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
        port_SPIx_set_chip_select();  //CS high
        Sleep(7);
        devID = instancereaddeviceid() ;
        // SPI not working or Unsupported Device ID
        if(DWT_DEVICE_ID != devID)
            return(-1) ;
        //clear the sleep bit - so that after the hard reset below the DW does not go into sleep
        dwt_softreset();
    }

    //reset the DW1000 by driving the RSTn line low
    reset_DW1000();

    result = instance_init() ;
    if (0 > result) return(-1) ; // Some failure has occurred

    SPI_ConfigFastRate(SPI_BaudRatePrescaler_4); //increase SPI to max
    devID = instancereaddeviceid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT MP device
    {
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }


    if(s1switch & SWS1_ANC_MODE)
    {
        instance_mode = ANCHOR;

        led_on(LED_PC6);

    }
    else
    {
        instance_mode = TAG;
        led_on(LED_PC7);
    }

    instancesetrole(instance_mode) ;     // Set this instance role

    instance_init_s(instance_mode);
    dr_mode = decarangingmode(s1switch);

    instConfig.channelNumber = chConfig[dr_mode].channel ;
    instConfig.preambleCode = chConfig[dr_mode].preambleCode ;
    instConfig.pulseRepFreq = chConfig[dr_mode].prf ;
    instConfig.pacSize = chConfig[dr_mode].pacSize ;
    instConfig.nsSFD = chConfig[dr_mode].nsSFD ;
    instConfig.sfdTO = chConfig[dr_mode].sfdTO ;
    instConfig.dataRate = chConfig[dr_mode].datarate ;
    instConfig.preambleLen = chConfig[dr_mode].preambleLength ;


    instance_config(&instConfig) ;                  // Set operating channel etc

    instancesettagsleepdelay(POLL_SLEEP_DELAY, BLINK_SLEEP_DELAY); //set the Tag sleep time

    instance_init_timings();

    return devID;
}
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
void process_dwRSTn_irq(void)
{
    instance_notify_DW1000_inIDLE(1);
}


#if (DWINTERRUPT_EN == 1)
void process_deca_irq(void)
{
    do{

        instance_process_irq(0);

    }while(port_CheckEXT_IRQ() == 1); //while IRQ line active (ARM can only do edge sensitive interrupts)

}
#else
void process_deca_irq(void)
{
  while(dwt_checkIRQ() == 1)
  {
        instance_process_irq(0);
    } //while IRQ line active

}
#endif

void initLCD(void)
{
    uint8 initseq[9] = { 0x39, 0x14, 0x55, 0x6D, 0x78, 0x38 /*0x3C*/, 0x0C, 0x01, 0x06 };
    uint8 command = 0x0;
    int j = 100000;

    writetoLCD( 9, 0,  initseq); //init seq
    while(j--);

    command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);
    command = 0x1 ;  //clear screen
    writetoLCD( 1, 0,  &command);
}

/*
 * @brief switch_mask  - bitmask of testing switches (currently 7 switches)
 *        switchbuff[] - switch name to test
 *        *switch_fn[]() - corresponded to switch test function
**/
#define switch_mask   (0x7F)

const uint8 switchbuf[]={0, TA_SW1_3 , TA_SW1_4 , TA_SW1_5 , TA_SW1_6 , TA_SW1_7 , TA_SW1_8 };
typedef int (* switch_handler_t)(uint16) ;
const switch_handler_t switch_fn[] ={ is_button_low, \
                                is_switch_on, is_switch_on, is_switch_on,\
                                is_switch_on, is_switch_on, is_switch_on };

/*
 * @fn test_application_run
 * @brief   test application for production pre-test procedure
**/
//void test_application_run(void)
//{
//    char  dataseq[2][40];
//    uint8 j, switchStateOn, switchStateOff;
//
//    switchStateOn=0;
//    switchStateOff=0;
//
//    led_on(LED_ALL);    // show all LED OK
//    Sleep(1000);
//
//    dataseq[0][0] = 0x1 ;  //clear screen
//    writetoLCD( 1, 0, (const uint8 *) &dataseq);
//    dataseq[0][0] = 0x2 ;  //return cursor home
//    writetoLCD( 1, 0, (const uint8 *) &dataseq);
//
///* testing SPI to DW1000*/
//    writetoLCD( 40, 1, (const uint8 *) "TESTING         ");
//    writetoLCD( 40, 1, (const uint8 *) "SPI, U2, S2, S3 ");
//    Sleep(1000);
//
//    if(inittestapplication(s1switch) == (uint32)-1)
//    {
//        writetoLCD( 40, 1, (const uint8 *) "SPI, U2, S2, S3 ");
//        writetoLCD( 40, 1, (const uint8 *) "-- TEST FAILS --");
//        while(1); //stop
//    }
//
//    writetoLCD( 40, 1, (const uint8 *) "SPI, U2, S2, S3 ");
//    writetoLCD( 40, 1, (const uint8 *) "    TEST OK     ");
//    Sleep(1000);
//
///* testing of switch S2 */
//    dataseq[0][0] = 0x1 ;  //clear screen
//    writetoLCD( 1, 0, (const uint8 *) &dataseq);
//
//    while( (switchStateOn & switchStateOff) != switch_mask )
//        {
//        memset(&dataseq, ' ', sizeof(dataseq));
//        strcpy(&dataseq[0][0], (const char *)"SWITCH");
//        strcpy(&dataseq[1][0], (const char *)"toggle");
////switch 7-1
//        for (j=0;j<sizeof(switchbuf);j++)
//        {
//            if( switch_fn[j](switchbuf[j]) ) //execute current switch switch_fn
//            {
//                dataseq[0][8+j]='O';
//                switchStateOn |= 0x01<<j;
//                switchStateOff &= ~(0x01<<j);//all switches finaly should be in off state
//            }else{
//                dataseq[1][8+j]='O';
//                switchStateOff |=0x01<<j;
//        }
//        }
//
//        writetoLCD(40, 1, (const uint8 *) &dataseq[0][0]);
//        writetoLCD(40, 1, (const uint8 *) &dataseq[1][0]);
//        Sleep(100);
//        }
//
//    led_off(LED_ALL);
//
//    writetoLCD( 40, 1, (const uint8 *) "  Preliminary   ");
//    writetoLCD( 40, 1, (const uint8 *) "   TEST OKAY    ");
//
//    while(1);
//    }


void setLCDline1(uint8 s1switch)
{
  uint8 command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);

  sprintf((char*)&dataseq[0], "DecaRanging  %02x", s1switch);
  writetoLCD( 40, 1, dataseq); //send some data

  sprintf((char*)&dataseq1[0], "                 ");
  writetoLCD( 16, 1, dataseq1); //send some data
}

/*
 * @fn      main()
 * @brief   main entry point
**/
int main(void)
{
    int i = 0;
    int toggle = 1;
    double range_result = 0;
    double avg_result = 0;

    led_off(LED_ALL); //turn off all the LEDs

    peripherals_init();

    spi_peripheral_init();

    Sleep(1000); //wait for LCD to power on

    initLCD();

    memset(dataseq, 0, LCD_BUFF_LEN);
    memcpy(dataseq, (const uint8 *) "DECAWAVE        ", 16);
    writetoLCD( 40, 1, dataseq); //send some data
    memcpy(dataseq, (const uint8 *) SOFTWARE_VER_STRING, 16); // Also set at line #26 (Should make this from single value !!!)
    writetoLCD( 16, 1, dataseq); //send some data

    Sleep(1000);
#ifdef USB_SUPPORT
    // enable the USB functionality
    usb_init();
    Sleep(1000);
#endif


//    s1switch = s1switch  | is_switch_on(TA_SW1_4) << 3;


    port_DisableEXT_IRQ(); //disable ScenSor IRQ until we configure the device

//    //test EVB1000 - used in EVK1000 production
//    if((is_button_low(0) == S1_SWITCH_ON) && (is_switch_on(TA_SW1_8) == S1_SWITCH_ON)) //using BOOT1 switch for test
//
//    {
//        test_application_run(); //does not return....
//    }
//    else
//    if(is_switch_on(TA_SW1_3) == S1_SWITCH_OFF)
//    {
//        int j = 1000000;
//        uint8 command;
//
//        memset(dataseq, 0, LCD_BUFF_LEN);
//
//        while(j--);
//        //command = 0x1 ;  //clear screen
//        //writetoLCD( 1, 0,  &command);
//        command = 0x2 ;  //return cursor home
//        writetoLCD( 1, 0,  &command);
//
//        memcpy(dataseq, (const uint8 *) "DECAWAVE   ", 12);
//        writetoLCD( 40, 1, dataseq); //send some data
//#ifdef USB_SUPPORT //this is set in the port.h file
//        memcpy(dataseq, (const uint8 *) "USB to SPI ", 12);
//#else
//#endif
//        writetoLCD( 16, 1, dataseq); //send some data
//
//        j = 1000000;
//
//        while(j--);
//
//        command = 0x2 ;  //return cursor home
//        writetoLCD( 1, 0,  &command);
//#ifdef USB_SUPPORT //this is set in the port.h file
//        // enable the USB functionality
//        //usb_init();
//
//        NVIC_DisableDECAIRQ();
//
//        // Do nothing in foreground -- allow USB application to run, I guess on the basis of USB interrupts?
//        while (1)       // loop forever
//        {
//            usb_run();
//        }
//#endif
//        return 1;
//    }
//    else //run DecaRanging application
//    {
        uint8 dataseq[LCD_BUFF_LEN];
        uint8 command = 0x0;

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
        memset(dataseq, ' ', LCD_BUFF_LEN);
        memcpy(dataseq, (const uint8 *) "DECAWAVE  RANGE", 15);
        writetoLCD( 15, 1, dataseq); //send some data

        led_off(LED_ALL);

#ifdef USB_SUPPORT //this is set in the port.h file
        usb_printconfig(16, (uint8 *)SOFTWARE_VER_STRING, s1switch);
#endif

        if(inittestapplication(s1switch) == (uint32)-1)
        {
            led_on(LED_ALL); //to display error....
            dataseq[0] = 0x2 ;  //return cursor home
            writetoLCD( 1, 0,  &dataseq[0]);
            memset(dataseq, ' ', LCD_BUFF_LEN);
            memcpy(dataseq, (const uint8 *) "ERROR   ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            memcpy(dataseq, (const uint8 *) "  INIT FAIL ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            return 0; //error
        }

        //sleep for 5 seconds displaying "Decawave"
        i=30;
        while(i--)
        {
            if (i & 1) led_off(LED_ALL);
            else    led_on(LED_ALL);

            Sleep(200);
        }
        i = 0;
        led_off(LED_ALL);
        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);

        memset(dataseq, ' ', LCD_BUFF_LEN);

        if(s1switch & SWS1_ANC_MODE)
        {
            instance_mode = ANCHOR;

            led_on(LED_PC6);
        }
        else
        {
            instance_mode = TAG;
            led_on(LED_PC7);
        }

        if(instance_mode == TAG)
        {
//            //if TA_SW1_2 is on use fast ranging (fast 2wr)
//            if(is_button_low(0) == S1_SWITCH_ON)
//            {
//                memcpy(&dataseq[2], (const uint8 *) " Fast Tag   ", 12);
//                writetoLCD( 40, 1, dataseq); //send some data
//                memcpy(&dataseq[2], (const uint8 *) "   Ranging  ", 12);
//                writetoLCD( 16, 1, dataseq); //send some data
//            }
//            else
//            {
                memcpy(&dataseq[2], (const uint8 *) " TAG BLINK  ", 12);

                writetoLCD( 40, 1, dataseq); //send some data
                sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
                writetoLCD( 16, 1, dataseq); //send some data
//            }
        }
        else
        {
            memcpy(&dataseq[2], (const uint8 *) "  AWAITING  ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            memcpy(&dataseq[2], (const uint8 *) "    POLL    ", 12);
            writetoLCD( 16, 1, dataseq); //send some data
        }

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
//    }
#if (DWINTERRUPT_EN == 1)
    port_EnableEXT_IRQ(); //enable ScenSor IRQ before starting
#endif

    memset(dataseq, ' ', LCD_BUFF_LEN);
    memset(dataseq1, ' ', LCD_BUFF_LEN);

#ifdef USART_SUPPORT
    printf2(" %s\n", SOFTWARE_VER_STRING);
#endif

    // main loop
    while(1)
    {
#if (DWINTERRUPT_EN == 0)
      process_deca_irq(); //poll DW1000 IRQ line when using polling of interrupt line
#endif
        instance_run();

        //if delayed TX scheduled but did not happen after expected time then it has failed... (has to be < slot period)
        //if anchor just go into RX and wait for next message from tags/anchors
        //if tag handle as a timeout
        if((instance_data[0].monitor == 1) && ((portGetTickCnt() - instance_data[0].timeofTx) > instance_data[0].finalReplyDelay_ms))
        {
              instance_data[0].wait4ack = 0;

              if(instance_mode == TAG)
              {
                inst_processrxtimeout(&instance_data[0]);
              }
              else //if(instance_mode == ANCHOR)
              {
                dwt_forcetrxoff();  //this will clear all events
                //enable the RX
                instance_data[0].testAppState = TA_RXE_WAIT ;
              }
              instance_data[0].monitor = 0;
        }

        if(instancenewrange())
        {
            int n, l = 0, /*txl = 0, rxl = 0,*/ aaddr, taddr, txa, rxa, rng, rng_raw;
            ranging = 1;
            //send the new range information to LCD and/or USB
            range_result = instance_get_idist();
            avg_result = instance_get_adist();
            //set_rangeresult(range_result);
            dataseq[0] = 0x2 ;  //return cursor home
            writetoLCD( 1, 0,  dataseq);

            memset(dataseq, ' ', LCD_BUFF_LEN);
            memset(dataseq1, ' ', LCD_BUFF_LEN);
            sprintf((char*)&dataseq[1], "LAST: %4.2f m", range_result);
            writetoLCD( 40, 1, dataseq); //send some data

            sprintf((char*)&dataseq1[1], "AVG8: %4.2f m", avg_result);

            writetoLCD( 16, 1, dataseq1); //send some data

            l = instance_get_lcount();
            //txl = instance_get_txl();
            //rxl = instance_get_rxl();
            aaddr = instancenewrangeancadd();
            taddr = instancenewrangetagadd();
            txa =  instancetxantdly();
            rxa =  instancerxantdly();
            rng = (int) (range_result*1000);
            rng_raw = (int) (instance_get_idistraw()*1000);

            if(instance_mode == TAG)
            {
                //n = sprintf((char*)&dataseq[0], "ia%04x t%04x %04x %04x %04x %04x %04x %02x %02x t", aaddr, taddr, rng, rng_raw, l, txa, rxa, txl, rxl);
              n = sprintf((char*)&dataseq[0], "ia%04x t%04x %08x %08x %04x %04x %04x t", aaddr, taddr, rng, rng_raw, l, txa, rxa);
            }
            else
            {
                //n = sprintf((char*)&dataseq[0], "ia%04x t%04x %04x %04x %04x %04x %04x %02x %02x a", aaddr, taddr, rng, rng_raw, l, txa, rxa, txl, rxl);
              //n = sprintf((char*)&dataseq[0], "ia%04x t%04x %08x %08x %04x %04x %04x %2.2f a", aaddr, taddr, rng, rng_raw, l, txa, rxa, instance_data[0].clockOffset);
              n = sprintf((char*)&dataseq[0], "ia%04x t%04x %08x %08x %04x %04x %04x a", aaddr, taddr, rng, rng_raw, l, txa, rxa);
            }
#ifdef USB_SUPPORT //this is set in the port.h file
            send_usbmessage(&dataseq[0], n);
#endif

#ifdef USART_SUPPORT
            {

              //printf2("R= %i mm\r\n",rng);
              printf2("R= %-3.2f m\r\n",range_result);
            }
#endif
        }

#ifdef USART_SUPPORT
        {
        int nrm = 0;
      if(nrm = instancenorange())
      {
        if(nrm == 1)
        {
          printf2("I= No Response\r\n");
        }
        else if(nrm == 2)
        {
          printf2("I= No Report\r\n");
        }
        else if(nrm == 3)
        {
          printf2("I= No Final\r\n");
        }
      }
        }
#endif

        if(ranging == 0)
        {
            if(instance_mode != ANCHOR)
            {
                if(instancesleeping())
                {
                    dataseq[0] = 0x2 ;  //return cursor home
                    writetoLCD( 1, 0,  dataseq);
                    if(toggle)
                    {
                        toggle = 0;
                        memcpy(&dataseq[0], (const uint8 *) "    AWAITING    ", 16);
                        writetoLCD( 40, 1, dataseq); //send some data
                        memcpy(&dataseq[0], (const uint8 *) "    RESPONSE    ", 16);
                        writetoLCD( 16, 1, dataseq); //send some data
                    }
                    else
                    {
                        toggle = 1;
                        memcpy(&dataseq[2], (const uint8 *) "   TAG BLINK    ", 16);

                        writetoLCD( 40, 1, dataseq); //send some data
                        sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
                        writetoLCD( 16, 1, dataseq); //send some data
                    }
                }

                if(instanceanchorwaiting() == 2)
                {
                    ranging = 1;
                    dataseq[0] = 0x2 ;  //return cursor home
                    writetoLCD( 1, 0,  dataseq);
                    memcpy(&dataseq[0], (const uint8 *) "    RANGING WITH", 16);
                    writetoLCD( 40, 1, dataseq); //send some data
                    sprintf((char*)&dataseq[0], "%016llX", instance_get_anchaddr());
                    writetoLCD( 16, 1, dataseq); //send some data
                }
            }
            else //if(instance_mode == ANCHOR)
            {
                if(instanceanchorwaiting())
                {
                    toggle+=2;

                    if(toggle > 300000)
                    {
                        dataseq[0] = 0x2 ;  //return cursor home
                        writetoLCD( 1, 0,  dataseq);
                        if(toggle & 0x1)
                        {
                            toggle = 0;
                            memcpy(&dataseq[0], (const uint8 *) "    AWAITING    ", 16);
                            writetoLCD( 40, 1, dataseq); //send some data
                            memcpy(&dataseq[0], (const uint8 *) "      POLL      ", 16);
                            writetoLCD( 16, 1, dataseq); //send some data
                        }
                        else
                        {
                            toggle = 1;
                            memcpy(&dataseq[0], (const uint8 *) " DISCOVERY MODE ", 16);
                            writetoLCD( 40, 1, dataseq); //send some data
                            sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
                            writetoLCD( 16, 1, dataseq); //send some data
                        }
                    }

                }
                else if(instanceanchorwaiting() == 2)
                {
                    dataseq[0] = 0x2 ;  //return cursor home
                    writetoLCD( 1, 0,  dataseq);
                    memcpy(&dataseq[0], (const uint8 *) "    RANGING WITH", 16);
                    writetoLCD( 40, 1, dataseq); //send some data
                    sprintf((char*)&dataseq[0], "%llX", instance_get_tagaddr());
                    writetoLCD( 16, 1, dataseq); //send some data
                }
            }
        }
#ifdef USB_SUPPORT //this is set in the port.h file
        usb_run();
#endif
    }


    return 0;
}
