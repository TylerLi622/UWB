/*! ----------------------------------------------------------------------------
 *  @file    instance.h
 *  @brief   DecaWave header for application level instance
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include "instance_sws.h"
#include "deca_types.h"
#include "deca_device_api.h"

/******************************************************************************************************************
********************* NOTES on DW (MP) features/options ***********************************************************
*******************************************************************************************************************/
#define DEEP_SLEEP (1) //To enable deep-sleep set this to 1
//DEEP_SLEEP mode can be used, for example, by a Tag instance to put the DW1000 into low-power deep-sleep mode, there are two cases:
// 1. when the Anchor is sending the range report back to the Tag, the Tag will enter sleep after a ranging exchange is finished
// once it receives a report or times out, before the next poll message is sent (before next ranging exchange is started).
// 2. when the Anchor is not sending the report the Tag will go automatically to sleep after the final message is sent


#define CORRECT_RANGE_BIAS  (1)     // Compensate for small bias due to uneven accumulator growth at close up high power

/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define NUM_INST            1
#define SPEED_OF_LIGHT      (299702547.0)     // in m/s in air
#define MASK_40BIT			(0x00FFFFFFFFFF)  // MP counter is 40 bits
#define MASK_TXDTS			(0x00FFFFFFFE00)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.


#define USING_64BIT_ADDR (1) //when set to 0 - the DecaRanging application will use 16-bit addresses

#define SIG_RX_BLINK			7		// Received ISO EUI 64 blink message
#define SIG_RX_UNKNOWN			99		// Received an unknown frame

// Existing frames type in ranging process.
enum
{
    BLINK = 0,
    RNG_INIT,
    POLL,
    RESP,
    FINAL,
    FRAME_TYPE_NB
};

//Fast 2WR function codes
#define RTLS_DEMO_MSG_RNG_INIT              (0x20)          // Ranging initiation message
#define RTLS_DEMO_MSG_TAG_POLL              (0x21)          // Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESP             (0x10)          // Anchor response to poll
#define RTLS_DEMO_MSG_TAG_FINAL             (0x29)          // Tag final massage back to Anchor (0x29 because of 5 byte timestamps needed for PC app)

//lengths including the Decaranging Message Function Code byte
#define TAG_POLL_MSG_LEN                    1				// FunctionCode(1),
#define ANCH_RESPONSE_MSG_LEN               9               // FunctionCode(1), RespOption (1), OptionParam(2), Measured_TOF_Time(5)
#define TAG_FINAL_MSG_LEN                   16              // FunctionCode(1), Poll_TxTime(5), Resp_RxTime(5), Final_TxTime(5)
#define RANGINGINIT_MSG_LEN					7				// FunctionCode(1), Tag Address (2), Response Time (2) * 2

#define MAX_MAC_MSG_DATA_LEN                (TAG_FINAL_MSG_LEN) //max message len of the above

#define STANDARD_FRAME_SIZE         127

#define ADDR_BYTE_SIZE_L            (8)
#define ADDR_BYTE_SIZE_S            (2)

#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2
#define FRAME_CRC					2
#define FRAME_SOURCE_ADDRESS_S        (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S          (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L        (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L          (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP					(FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID) //5
#define FRAME_CRTL_AND_ADDRESS_L    (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP) //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S    (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS	(FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //15 bytes for 1 16-bit address and 1 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 15 - 16 - 2 = 94

//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
#define MAX_USER_PAYLOAD_STRING	MAX_USER_PAYLOAD_STRING_LL

// Total frame lengths.
#if (USING_64BIT_ADDR == 1)
    #define RNG_INIT_FRAME_LEN_BYTES (RANGINGINIT_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC)
    #define POLL_FRAME_LEN_BYTES (TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC)
    #define RESP_FRAME_LEN_BYTES (ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC)
    #define FINAL_FRAME_LEN_BYTES (TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC)
#else
    #define RNG_INIT_FRAME_LEN_BYTES (RANGINGINIT_MSG_LEN + FRAME_CRTL_AND_ADDRESS_LS + FRAME_CRC)
    #define POLL_FRAME_LEN_BYTES (TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC)
    #define RESP_FRAME_LEN_BYTES (ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC)
    #define FINAL_FRAME_LEN_BYTES (TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC)
#endif

#define BLINK_FRAME_CONTROL_BYTES       (1)
#define BLINK_FRAME_SEQ_NUM_BYTES       (1)
#define BLINK_FRAME_CRC					(FRAME_CRC)
#define BLINK_FRAME_SOURCE_ADDRESS      (ADDR_BYTE_SIZE_L)
#define BLINK_FRAME_CTRLP				(BLINK_FRAME_CONTROL_BYTES + BLINK_FRAME_SEQ_NUM_BYTES) //2
#define BLINK_FRAME_CRTL_AND_ADDRESS    (BLINK_FRAME_SOURCE_ADDRESS + BLINK_FRAME_CTRLP) //10 bytes
#define BLINK_FRAME_LEN_BYTES           (BLINK_FRAME_CRTL_AND_ADDRESS + BLINK_FRAME_CRC)


#define ANCHOR_LIST_SIZE			(4) //this is limited to 4 in this application see also
#define TAG_LIST_SIZE				(1)	//anchor will range with 1st Tag it gets blink from

#define DELAYRX_WAIT4REPORT	(160)   //this is the time in us the RX turn on is delayed (after Final transmission and before Report reception starts)


#define BLINK_SLEEP_DELAY					1000 //ms
#define POLL_SLEEP_DELAY					500 //ms
//#define POLL_SLEEP_DELAY					50 //ms	//NOTE 200 gives 5 Hz range period


#define IMMEDIATE_RESPONSE (1)

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define INST_DONE_WAIT_FOR_NEXT_EVENT   1   //this signifies that the current event has been processed and instance is ready for next one
#define INST_DONE_WAIT_FOR_NEXT_EVENT_TO    2   //this signifies that the current event has been processed and that instance is waiting for next one with a timeout
                                        //which will trigger if no event coming in specified time
#define INST_NOT_DONE_YET               0   //this signifies that the instance is still processing the current event

// Function code byte offset (valid for all message types).
#define FCODE                               0               // Function code is 1st byte of messageData
// Final message byte offsets.
#define PTXT                                1
#define RRXT                                6
#define FTXT                                11
// Length of ToF value in report message. Can be used as offset to put up to
// 4 ToF values in the report message.
#define TOFR                                4
// Anchor response byte offsets.
#define RES_R1                              1               // Response option octet 0x02 (1),
#define RES_R2                              2               // Response option parameter 0x00 (1) - used to notify Tag that the report is coming
#define RES_R3                              3               // Response option parameter 0x00 (1),
// Ranging init message byte offsets. Composed of tag short address, anchor
// response delay and tag response delay.
#define RNG_INIT_TAG_SHORT_ADDR_LO 1
#define RNG_INIT_TAG_SHORT_ADDR_HI 2
#define RNG_INIT_ANC_RESP_DLY_LO 3
#define RNG_INIT_ANC_RESP_DLY_HI 4
#define RNG_INIT_TAG_RESP_DLY_LO 5
#define RNG_INIT_TAG_RESP_DLY_HI 6

// Response delay values coded in ranging init message.
// This is a bitfield composed of:
//   - bits 0 to 14: value
//   - bit 15: unit
#define RESP_DLY_VAL_SHIFT 0
#define RESP_DLY_VAL_MASK 0x7FFF
#define RESP_DLY_UNIT_SHIFT 15
#define RESP_DLY_UNIT_MASK 0x8000

// Response time possible units: microseconds or milliseconds.
#define RESP_DLY_UNIT_US 0
#define RESP_DLY_UNIT_MS 1

// Response delays types present in ranging init message.
enum
{
    RESP_DLY_ANC = 0,
    RESP_DLY_TAG,
    RESP_DLY_NB
};

// Convert microseconds to symbols, float version.
// param  x  value in microseconds
// return  value in symbols.
#define US_TO_SY(x) ((x) / 1.0256)

// Convert microseconds to symbols, integer version.
// param  x  value in microseconds
// return  value in symbols.
#define US_TO_SY_INT(x) (((x) * 10000) / 10256)

// Minimum delay between reception and following transmission.
#define RX_TO_TX_TIME_US 150
#define RXTOTXTIME          ((int)(150.0 / 1.0256)) //e.g. Poll RX to Response TX time

// Default anchor turn-around time: has to be RX_TO_TX_TIME_US when using
// immediate response, cannot be less than 170 us when not.
#define ANC_TURN_AROUND_TIME_US RX_TO_TX_TIME_US
#if (IMMEDIATE_RESPONSE == 1) && (ANC_TURN_AROUND_TIME_US != RX_TO_TX_TIME_US)
    #error "When using immediate response, anchor turn-around time has to be equal to RX to TX time!"
#endif
// Default tag turn-around time: cannot be less than 300 us. Defined as 500 us
// so that the tag is not transmitting more than one frame by millisecond (for
// power management purpose).
#define TAG_TURN_AROUND_TIME_US 500

// "Long" response delays value. Over this limit, special processes must be
// applied.
#define LONG_RESP_DLY_LIMIT_US 25000

// Delay between blink reception and ranging init message. This is the same for
// all modes.
#define RNG_INIT_REPLY_DLY_MS (150)

// Reception start-up time, in symbols.
#define RX_START_UP_SY 16

typedef enum instanceModes{LISTENER, TAG, ANCHOR, TAG_TDOA, NUM_MODES} INST_MODE;

//Listener = in this mode, the instance only receives frames, does not respond
//Tag = Exchanges DecaRanging messages (Poll-Response-Final) with Anchor and enabling Anchor to calculate the range between the two instances
//Anchor = see above

typedef enum inst_states
{
    TA_INIT, //0

    TA_TXE_WAIT,                //1
    TA_TXPOLL_WAIT_SEND,        //2
    TA_TXFINAL_WAIT_SEND,       //3
    TA_TXRESPONSE_WAIT_SEND,    //4
    TA_TX_WAIT_CONF,            //6

    TA_RXE_WAIT,                //7
    TA_RX_WAIT_DATA,            //8

    TA_SLEEP_DONE,              //9
    TA_TXBLINK_WAIT_SEND,       //10
    TA_TXRANGINGINIT_WAIT_SEND  //11
} INST_STATES;


// This file defines data and functions for access to Parameters in the Device
//message structure for Poll, Response and Final message

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  13-20 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LL] ; //  22-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlsl ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  07-08
    uint8 messageData[MAX_USER_PAYLOAD_STRING_SS] ; //  09-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dsss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  13-14
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  07-14 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dssl ;

//12 octets for Minimum IEEE ID blink
typedef struct
{
    uint8 frameCtrl;                         		//  frame control bytes 00
    uint8 seqNum;                               	//  sequence_number 01
    uint8 tagID[BLINK_FRAME_SOURCE_ADDRESS];        //  02-09 64 bit address
    uint8 fcs[2] ;                              	//  10-11  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} iso_IEEE_EUI64_blink_msg ;

//18 octets for IEEE ID blink with Temp and Vbat values
typedef struct
{
    uint8 frameCtrl;                         		//  frame control bytes 00
    uint8 seqNum;                               	//  sequence_number 01
    uint8 tagID[BLINK_FRAME_SOURCE_ADDRESS];        //  02-09 64 bit addresses
	uint8 enchead[2];								//  10-11 2 bytes (encoded header and header extension)
	uint8 messageID;								//  12 message ID (0xD1) - DecaWave message
	uint8 temp;										//  13 temperature value
	uint8 vbat;										//  14 voltage value
	uint8 gpio;										//  15 gpio status
    uint8 fcs[2] ;                              	//  16-17  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} iso_IEEE_EUI64_blinkdw_msg ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 fcs[2] ;                              	//  03-04  CRC
} ack_msg ;

typedef struct
{
    uint8 channelNumber ;       // valid range is 1 to 11
    uint8 preambleCode ;        // 00 = use NS code, 1 to 24 selects code
    uint8 pulseRepFreq ;        // NOMINAL_4M, NOMINAL_16M, or NOMINAL_64M
    uint8 dataRate ;            // DATA_RATE_1 (110K), DATA_RATE_2 (850K), DATA_RATE_3 (6M81)
    uint8 preambleLen ;         // values expected are 64, (128), (256), (512), 1024, (2048), and 4096
    uint8 pacSize ;
    uint8 nsSFD ;
    uint16 sfdTO;  //!< SFD timeout value (in symbols) e.g. preamble length (128) + SFD(8) - PAC + some margin ~ 135us... DWT_SFDTOC_DEF; //default value
} instanceConfig_t ;


typedef struct
{
	uint32 icid;

	dwt_rxdiag_t diag;

#if DECA_LOG_ENABLE==1
    int         accumLogging ;                                // log data to a file, used to indicate that we are currenty logging (file is open)
	FILE        *accumLogFile ;                               // file
#endif

} devicelogdata_t ;

/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define MAX_EVENT_NUMBER (4)
//NOTE: Accumulators don't need to be stored as part of the event structure as when reading them only one RX event can happen...
//the receiver is singly buffered and will stop after a frame is received

typedef struct
{
	uint8  type;			// event type
	uint8  type2;			// holds the event type - does not clear (not used to show if event has been processed)
	uint8  type3;
	//uint8  broadcastmsg;	// specifies if the rx message is broadcast message
	uint16 rxLength ;

	uint64 timeStamp ;		// last timestamp (Tx or Rx)

	uint32 timeStamp32l ;		   // last tx/rx timestamp - low 32 bits
	uint32 timeStamp32h ;		   // last tx/rx timestamp - high 32 bits

	union {
			//holds received frame (after a good RX frame event)
			uint8   frame[STANDARD_FRAME_SIZE];
    		srd_msg_dlsl rxmsg_ll ; //64 bit addresses
			srd_msg_dssl rxmsg_sl ;
			srd_msg_dlss rxmsg_ls ;
			srd_msg_dsss rxmsg_ss ; //16 bit addresses
			ack_msg rxackmsg ; //holds received ACK frame
			iso_IEEE_EUI64_blink_msg rxblinkmsg;
			iso_IEEE_EUI64_blinkdw_msg rxblinkmsgdw;
	}msgu;

	//uint32 eventtime ;
	//uint32 eventtimeclr ;
	//uint8 gotit;
}event_data_t ;

#define RTD_MED_SZ          8      // buffer size for mean of 8

typedef struct{
	uint32 diffRmP;
	uint32 diffFmR;
} rtd_t;

typedef struct {
                uint8 PGdelay;

                //TX POWER
                //31:24     BOOST_0.125ms_PWR
                //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
                //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
                //7:0       DEFAULT_PWR-TX_DATA_PWR
                uint32 txPwr[2]; //
}tx_struct;

typedef struct
{
    INST_MODE mode;				//instance mode (tag or anchor)

    INST_STATES testAppState ;			//state machine - current state
    INST_STATES nextState ;				//state machine - next state
    INST_STATES previousState ;			//state machine - previous state
    int done ;					//done with the current event/wait for next event to arrive

	//configuration structures
	dwt_config_t    configData ;	//DW1000 channel configuration
	dwt_txconfig_t  configTX ;		//DW1000 TX power configuration
	uint16			txantennaDelay ; //DW1000 TX antenna delay
	uint16			rxantennaDelay ; //DW1000 RX antenna delay
	uint8 antennaDelayChanged;
	// "MAC" features
    uint8 frameFilteringEnabled ;	//frame filtering is enabled

    // Is sleeping between frames enabled?
    uint8 sleep_en;

	//timeouts and delays
	int tagSleepTime_ms; //in milliseconds
	int tagBlinkSleepTime_ms;
	//this is the delay used for the delayed transmit (when sending the ranging init, response, and final messages)
	uint64 rnginitReplyDelay ;
	uint64 finalReplyDelay ;
	uint64 responseReplyDelay ;
	int finalReplyDelay_ms ;

	// xx_sy the units are 1.0256 us
	uint32 txToRxDelayAnc_sy ;    // this is the delay used after sending a response and turning on the receiver to receive final
	uint32 txToRxDelayTag_sy ;    // this is the delay used after sending a poll and turning on the receiver to receive response
	int rnginitW4Rdelay_sy ;	// this is the delay used after sending a blink and turning on the receiver to receive the ranging init message

	int fwtoTime_sy ;	//this is final message duration (longest out of ranging messages)
	int fwtoTimeB_sy ;	//this is the ranging init message duration

	uint32 delayedReplyTime;		// delayed reply time of delayed TX message - high 32 bits

    uint32 rxTimeouts ;
	// - not used in the ARM code uint32 responseTimeouts ;

    // Pre-computed frame lengths for frames involved in the ranging process,
    // in microseconds.
    uint32 fl_us[FRAME_TYPE_NB];

	//message structures used for transmitted messages
#if (USING_64BIT_ADDR == 1)
	srd_msg_dlsl rng_initmsg ;	// ranging init message (destination long, source long)
    srd_msg_dlsl msg ;			// simple 802.15.4 frame structure (used for tx message) - using long addresses
#else
	srd_msg_dlss rng_initmsg ;  // ranging init message (destination long, source short)
    srd_msg_dsss msg ;			// simple 802.15.4 frame structure (used for tx message) - using short addresses
#endif
	iso_IEEE_EUI64_blink_msg blinkmsg ; // frame structure (used for tx blink message)

//messages used in "fast" ranging ...
	srd_msg_dlss rnmsg ; // ranging init message structure
	srd_msg_dsss msg_f ; // ranging message with 16-bit addresses - used for "fast" ranging

	//Tag function address/message configuration
	uint8   eui64[8];				// devices EUI 64-bit address
	uint16  tagShortAdd ;		    // Tag's short address (16-bit) used when USING_64BIT_ADDR == 0
	uint16  psduLength ;			// used for storing the frame length
    uint8   frame_sn;				// modulo 256 frame sequence number - it is incremented for each new frame transmittion
	uint16  panid ;					// panid used in the frames

    uint8 relpyAddress[8] ;         // address of the anchor the tag is ranging with

	//64 bit timestamps
	//union of TX timestamps
	union {
		uint64 txTimeStamp ;		   // last tx timestamp
		uint64 tagPollTxTime ;		   // tag's poll tx timestamp
	    uint64 anchorRespTxTime ;	   // anchor's reponse tx timestamp
	}txu;
	uint64 anchorRespRxTime ;	    // receive time of response message
	uint64 tagPollRxTime ;          // receive time of poll message

	//32 bit timestamps (when "fast" ranging is used)
	uint32 tagPollTxTime32l ;      // poll tx time - low 32 bits
	uint32 tagPollRxTime32l ;      // poll rx time - low 32 bits
	uint32 anchorRespTxTime32l ;    // response tx time - low 32 bits

	uint32 anchResp1RxTime32l ;		// response 1 rx time - low 32 bits

	//application control parameters
    uint8	wait4ack ;				// if this is set to DWT_RESPONSE_EXPECTED, then the receiver will turn on automatically after TX completion
	uint8   instToSleep;			// if set the instance will go to sleep before sending the blink/poll message
	uint8	stoptimer;				// stop/disable an active timer
    uint8	instancetimer_en;		// enable/start a timer
    uint32	instancetimer;			// e.g. this timer is used to timeout Tag when in deep sleep so it can send the next poll message
    uint32	instancetimer_saved;
    // - not used in the ARM code
    //uint8	deviceissleeping;		// this disabled reading/writing to DW1000 while it is in sleep mode
									// (DW1000 will wake on chip select so need to disable and chip select line activity)
	uint8	gotTO;					// got timeout event

	uint8   responseRxNum;			// response number


    //diagnostic counters/data, results and logging
    int32 tof32 ;
    int64 tof ;
    double clockOffset ;

    uint32 blinkRXcount ;
	int txmsgcount;
	int	rxmsgcount;
	int lateTX;
	int lateRX;

    double adist[RTD_MED_SZ] ;
    double adist4[4] ;
    double longTermRangeSum ;
    int longTermRangeCount ;
    int tofindex ;
    int tofcount ;
    int last_update ;           // detect changes to status report

    double idistmax;
    double idistmin;
    double idistance ; // instantaneous distance
    int newrange;
    int norange;
    int newrangeancaddress; //last 4 bytes of anchor address
    int newrangetagaddress; //last 4 bytes of tag address
    // - not used in the ARM code uint32	lastReportTime;
    int respPSC;

    //if set to 1 then it means that DW1000 is in DEEP_SLEEP
    //so the ranging has finished and micro can output on USB/LCD
    //if sending data to LCD during ranging this limits the speed of ranging
    uint8 canprintinfo ;
	//devicelogdata_t devicelogdata;


	uint8 tagToRangeWith;	//it is the index of the tagList array which contains the address of the Tag we are ranging with
    uint8 tagListLen ;
    uint8 anchorListIndex ;
	uint8 tagList[TAG_LIST_SIZE][8];


	//event queue - used to store DW1000 events as they are processed by the dw_isr/callback functions
    event_data_t dwevent[MAX_EVENT_NUMBER]; //this holds any TX/RX events and associated message data
	event_data_t saved_dwevent; //holds an RX event while the ACK is being sent
    uint8 dweventIdxOut;
    uint8 dweventIdxIn;
	uint8 dweventPeek;
	uint8 monitor;
	uint32 timeofTx ;
	int dwIDLE;

} instance_data_t ;

typedef struct
{

	int (*testapprun_fn)(instance_data_t *inst, int message);

} instance_localdata_t ;
//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in logging/displaying range and status data
//
//-------------------------------------------------------------------------------------------------------------

// function to calculate and report the Time of Flight to the GUI/display
void reportTOF(instance_data_t *inst);
// clear the status/ranging data 
void instanceclearcounts(void) ;
void instcleartaglist(void);
void instsettagtorangewith(int tagID);
int instaddtagtolist(instance_data_t *inst, uint8 *tagAddr);

void instance_readaccumulatordata(void);
//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in driving/controlling the ranging application
//
//-------------------------------------------------------------------------------------------------------------
void setupmacframedata(instance_data_t *inst, int fcode);

// opent the SPI Cheetah interface - called from inittestapplication()
int instancespiopen(void) ;  // Open SPI and return handle
// close the SPI Cheetah interface  
void instance_close(void);
// Call init, then call config, then call run. call close when finished
// initialise the instance (application) structures and DW1000 device
int instance_init(void);
int instance_init_s(int mode);

// configure the instance and DW1000 device
void instance_config(instanceConfig_t *config) ;  

void instancerxon(instance_data_t *inst, int delayed, uint64 delayedReceiveTime);
void inst_processackmsg(instance_data_t *inst, uint8 seqNum);
void inst_processrxtimeout(instance_data_t *inst);

int instancesendpacket(uint16 length, uint8 txmode, uint32 dtime);

// called (periodically or from and interrupt) to process any outstanding TX/RX events and to drive the ranging application
int instance_run(void) ;       // returns indication of status report change
int testapprun(instance_data_t *inst, int message);

void instance_setapprun(int (*apprun_fn)(instance_data_t *inst, int message));

// calls the DW1000 interrupt handler
#define instance_process_irq(x) 	dwt_isr()  //call device interrupt handler
// configure TX/RX callback functions that are called from DW1000 ISR
void instance_rxcallback(const dwt_callback_data_t *rxd);
void instance_txcallback(const dwt_callback_data_t *txd);

// sets the Tag sleep delay time (the time Tag "sleeps" between each ranging attempt)
void instancesettagsleepdelay(int rangingsleep, int blinkingsleep);
void instancesetreplydelay(int datalength);

// Pre-compute frame lengths, timeouts and delays needed in ranging process.
// /!\ This function assumes that there is no user payload in the frame.
void instance_init_timings(void);

// set/get the instance roles e.g. Tag/Anchor/Listener
void instancesetrole(int mode) ;                // 
int instancegetrole(void) ;
// get the DW1000 device ID (e.g. 0xDECA0130 for MP)
uint32 instancereaddeviceid(void) ;                                 // Return Device ID reg, enables validation of physical device presence


void instancerxon(instance_data_t *inst, int delayed, uint64 delayedReceiveTime);
double instance_get_adist(void);

double instance_get_idist(void);
double instance_get_idistraw(void);
int instance_get_lcount(void);

uint64 instance_get_addr(void); //get own address (8 bytes)
uint64 instance_get_tagaddr(void); //get tag address (8 bytes)
uint64 instance_get_anchaddr(void); //get anchor address (that sent the ToF)

int instancenewrangeancadd(void);
int instancenewrangetagadd(void);
int instancenewrange(void);
int instancenorange(void);
int instancesleeping(void);
int instanceanchorwaiting(void);

int instance_get_dly(void);

int instance_get_rxf(void);

int instance_get_txf(void); //get number of Txed frames

int instance_get_respPSC(void);

int instance_get_txl(void) ;
int instance_get_rxl(void) ;

uint32 convertmicrosectodevicetimeu32 (double microsecu);
uint64 convertmicrosectodevicetimeu (double microsecu);
double convertdevicetimetosec(int32 dt);
double convertdevicetimetosec8(uint8* dt);

#define DWT_PRF_64M_RFDLY   (514.462f)
#define DWT_PRF_16M_RFDLY   (513.9067f)
const uint16 rfDelays[2];
const tx_struct txSpectrumConfig[8];

instance_data_t instance_data[NUM_INST] ;

int testapprun_af(instance_data_t *inst, int message);
int testapprun_tf(instance_data_t *inst, int message);

int instance_peekevent(void);

void instance_putevent(event_data_t newevent);

event_data_t* instance_getevent(int x);

void instance_clearevents(void);

void instance_notify_DW1000_inIDLE(int idle);

// configure the antenna delays
void instanceconfigantennadelays(uint16 tx, uint16 rx);
void instancesetantennadelays(void);
uint16 instancetxantdly(void);
uint16 instancerxantdly(void);

#ifdef __cplusplus
}
#endif

#endif
