#include "main.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

// microseconds per clock interrupt tick
#define USECPERTICK 10
//#define MYSYSCLOCK 72000000
#define MYSYSCLOCK HAL_RCC_GetSysClockFreq()

#define MYPRESCALER 0
#define MYPERIOD MYSYSCLOCK/1000000*USECPERTICK-1

#define RECIV_PIN (HAL_GPIO_ReadPin(recive_IR_GPIO_Port, recive_IR_Pin))

#define true 1
#define false 0

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define RAWBUF 256

#define STATE_IDLE      2
#define STATE_MARK      3
#define STATE_SPACE     4
#define STATE_STOP      5
#define STATE_OVERFLOW  6

// Due to sensor lag, when received, Marks  tend to be 100us too long and Spaces tend to be 100us too short
#define MARK_EXCESS 100

// Upper and Lower percentage tolerances in measurements
#define TOLERANCE 25
#define LTOL (1.0 - (TOLERANCE / 100.))
#define UTOL (1.0 + (TOLERANCE / 100.))

// Minimum gap between IR transmissions
#define _GAP            10000
#define GAP_TICKS       (_GAP/USECPERTICK)

#define TICKS_LOW(us)   ((int)(((us)*LTOL / USECPERTICK)))
#define TICKS_HIGH(us)  ((int)(((us)*UTOL / USECPERTICK + 1)))

// IR detector output is active low
#define MARK   0
#define SPACE  1

#define PRONTO_ONCE        false
#define PRONTO_REPEAT      true
#define PRONTO_FALLBACK    true
#define PRONTO_NOFALLBACK  false

#define REPEAT 0xFFFFFFFF // Decoded value for NEC when a repeat code is received

typedef struct // The fields are ordered to reduce memory over caused by struct-padding
{
	uint8_t rcvstate;        // State Machine state
	uint8_t rawlen;          // counter of entries in rawbuf
	uint16_t timer;           // State timer, counts 50uS ticks.
	uint16_t rawbuf[RAWBUF];  // raw data
	uint8_t overflow;        // Raw buffer overflow occurred
} irparams_t;

volatile irparams_t irparams;

typedef enum {
	UNUSED = -1,
	UNKNOWN = 0,
	RC5 = 1,
	RC6 = 2,
	NEC = 3,
	SONY = 4,
	PANASONIC = 5,
	JVC = 6,
	SAMSUNG = 7,
	WHYNTER = 8,
	AIWA_RC_T501 = 9,
	LG = 10,
	SANYO = 11,
	MITSUBISHI = 12,
	DISH = 13,
	SHARP = 14,
	DENON = 15,
	PRONTO = 16,
} decode_type_t;

char* getProtocolString(decode_type_t protocol);

typedef enum {
	RECV = 0, TRAN = 1, NONE = 2, DECODE = 3
} remote_mode;
extern remote_mode mode;

// Results returned from the decoder
typedef struct {
	decode_type_t decode_type; // UNKNOWN, NEC, SONY, RC5, ...
	uint16_t address; // Used by Panasonic & Sharp [16-bits]
	uint32_t value; // Decoded value [max 32-bits]
	int16_t bits; // Number of bits in decoded value
	volatile uint16_t *rawbuf; // Raw intervals in 10uS ticks
	int16_t rawlen; // Number of records in rawbuf
	int16_t overflow; // true iff IR raw code too long
} decode_results;

decode_results results;

extern int out_enabled;
extern int in_enabled;

// Mark & Space matching functions
int MATCH(int measured, int desired);
int MATCH_MARK(int measured_ticks, int desired_us);
int MATCH_SPACE(int measured_ticks, int desired_us);

void DWT_Init();

int16_t my_decode(decode_results *results);
long decodeHash(decode_results *results);
int compare(unsigned int oldval, unsigned int newval);
void my_enableIRIn();
void my_resume();
void my_disable();

////////////////////////////////////////////////////////////
#define DECODE_RC5           1
#define SEND_RC5             1

#define DECODE_RC6           1
#define SEND_RC6             1

#define DECODE_NEC           1
#define SEND_NEC             1

#define DECODE_SONY          1
#define SEND_SONY            1

#define DECODE_PANASONIC     1
#define SEND_PANASONIC       1

#define DECODE_JVC           1
#define SEND_JVC             1

#define DECODE_SAMSUNG       1
#define SEND_SAMSUNG         1

#define DECODE_WHYNTER       1
#define SEND_WHYNTER         1

#define DECODE_AIWA_RC_T501  1
#define SEND_AIWA_RC_T501    1

#define DECODE_LG            1
#define SEND_LG              1

#define DECODE_SANYO         0
#define SEND_SANYO           0

#define DECODE_MITSUBISHI    0
#define SEND_MITSUBISHI      0

#define DECODE_DISH          0
#define SEND_DISH            0

#define DECODE_SHARP         0
#define SEND_SHARP           0

#define DECODE_DENON         1
#define SEND_DENON           1

#define DECODE_PRONTO        0
#define SEND_PRONTO          0

///////////////////////////////////////////////////////////////////////////////////////////////
#if (DECODE_RC5 || DECODE_RC6)
// This helper function is shared by RC5 and RC6
int getRClevel(decode_results *results, int *offset, int *used, int t1);
#endif

#if DECODE_RC5
uint8_t decodeRC5(decode_results *results);
#endif

#if DECODE_RC6
uint8_t decodeRC6(decode_results *results);
#endif
//......................................................................
#if DECODE_NEC
uint8_t decodeNEC(decode_results *results);
#endif
//......................................................................
#if DECODE_SONY
uint8_t decodeSony(decode_results *results);
#endif
//......................................................................
#if DECODE_PANASONIC
uint8_t decodePanasonic(decode_results *results);
#endif
//......................................................................
#if DECODE_JVC
uint8_t decodeJVC(decode_results *results);
#endif
//......................................................................
#if DECODE_SAMSUNG
uint8_t decodeSAMSUNG(decode_results *results);
#endif
//......................................................................
#if DECODE_WHYNTER
uint8_t decodeWhynter(decode_results *results);
#endif
//......................................................................
#if DECODE_AIWA_RC_T501
uint8_t decodeAiwaRCT501(decode_results *results);
#endif
//......................................................................
#if DECODE_LG
uint8_t decodeLG(decode_results *results);
#endif
//......................................................................
#if DECODE_SANYO
	uint8_t decodeSanyo(decode_results *results);
#endif
//......................................................................
#if DECODE_MITSUBISHI
	uint8_t decodeMitsubishi(decode_results *results);
#endif
//......................................................................
#if DECODE_DISH
	uint8_t decodeDish(decode_results *results); // NOT WRITTEN
#endif
//......................................................................
#if DECODE_SHARP
	uint8_t decodeSharp(decode_results *results); // NOT WRITTEN
#endif
//......................................................................
#if DECODE_DENON
uint8_t decodeDenon(decode_results *results);
#endif

///////////////////////////////////////////////////////////////////////////////////////////////
void custom_delay_usec(unsigned long uSecs);
void enableIROut(uint8_t khz);
void mark(unsigned int usec);
void space(unsigned int usec);
void sendRaw(uint16_t buf[], unsigned int len, uint8_t hz);
void send(uint16_t buf[], unsigned int len, unsigned long data, int nbits,
		decode_type_t protocol);
///////////////////////////////////////////////////////////////////////////////////////////////
#if SEND_RC5
void sendRC5(unsigned long data, int nbits);
#endif

#if SEND_RC6
void sendRC6(unsigned long data, int nbits);
#endif
//......................................................................
#if SEND_NEC
void sendNEC(unsigned long data, int nbits);
#endif
//......................................................................
#if SEND_SONY
void sendSony(unsigned long data, int nbits);
#endif
//......................................................................
#if SEND_PANASONIC
void sendPanasonic(unsigned int address, unsigned long data);
#endif
//......................................................................
#if SEND_JVC
// JVC does NOT repeat by sending a separate code (like NEC does).
// The JVC protocol repeats by skipping the header.
// To send a JVC repeat signal, send the original code value
//   and set 'repeat' to true
void sendJVC(unsigned long data, int nbits, uint8_t repeat);
#endif
//......................................................................
#if SEND_SAMSUNG
void sendSAMSUNG(unsigned long data, int nbits);
#endif
//......................................................................
#if SEND_WHYNTER
void sendWhynter(unsigned long data, int nbits);
#endif
//......................................................................
#if SEND_AIWA_RC_T501
void sendAiwaRCT501(int code);
#endif
//......................................................................
#if SEND_LG
void sendLG(unsigned long data, int nbits);
#endif
//......................................................................
#if SEND_SANYO
	void sendSanyo(); // NOT WRITTEN
#endif
//......................................................................
#if SEND_MISUBISHI
	void sendMitsubishi(); // NOT WRITTEN
#endif
//......................................................................
#if SEND_DISH
	void sendDISH(unsigned long data, int nbits);
#endif
//......................................................................
#if SEND_SHARP
	void sendSharpRaw(unsigned long data, int nbits);
	void sendSharp(unsigned int address, unsigned int command);
#endif
//......................................................................
#if SEND_DENON
void sendDenon(unsigned long data, int nbits);
#endif
//......................................................................
#if SEND_PRONTO
	void sendPronto(char* code, uint8_t repeat, uint8_t fallback);
#endif

