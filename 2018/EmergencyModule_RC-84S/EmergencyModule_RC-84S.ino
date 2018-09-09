/******************************************************************************
    ファイル名：    緊急停止 Module
    概  要：
    詳  細：
    備  考：ATTINY10 用
-------------------------------------------------------------------------------
    履  歴：        2017/08/26    J.Tanaka        新規作成
******************************************************************************/
#include <avr/interrupt.h>

/******************************************************************************
    グローバル定数
******************************************************************************/
// receiver states
#define STATE_IDLE      0
#define STATE_HDR_MARK  1
#define STATE_HDR_SPACE 2
#define STATE_BIT_MARK  3
#define STATE_BIT_SPACE 4
#define STATE_STOP      5

#define MARK  0

#define IR_STAT         1
#define RECV_PIN        2
#define CUSTOMCODE      0x4BB4

// pulse parameters in usec
#define NEC_HDR_MARK    9000
#define NEC_HDR_SPACE   4500
#define NEC_BIT_MARK    560
#define NEC_ONE_SPACE   1690
#define NEC_ZERO_SPACE  560

#define USECPERTICK 50  // microseconds per clock interrupt tick

// Marks tend to be 100us too long, and spaces 100us too short
// when received due to sensor lag.
#define MARK_EXCESS 100

#define TOLERANCE 25  // percent tolerance in measurements
#define LTOL (1.0 - TOLERANCE/100.)
#define UTOL (1.0 + TOLERANCE/100.)

#define _GAP 5000 // Minimum map between transmissions
#define GAP_TICKS (_GAP/USECPERTICK)

#define TICKS_LOW(us) (int) (((us)*LTOL/USECPERTICK))
#define TICKS_HIGH(us) (int) (((us)*UTOL/USECPERTICK + 1))

#define MATCH(measured_ticks, desired_us) ((measured_ticks) >= TICKS_LOW(desired_us) && (measured_ticks) <= TICKS_HIGH(desired_us))
#define MATCH_MARK(measured_ticks, desired_us) MATCH(measured_ticks, (desired_us) + MARK_EXCESS)
#define MATCH_SPACE(measured_ticks, desired_us) MATCH((measured_ticks), (desired_us) - MARK_EXCESS)

/******************************************************************************
    グローバル変数
******************************************************************************/
volatile uint8_t  rcvstate;      // state machine

/******************************************************************************
    概  要：
    関  数：    setup()
    引  数：
    戻り値：    なし
    機  能：
    備  考：
******************************************************************************/
void setup( void )
{
    TCCR0A = 0x00;
    TCCR0B = 0x01;
    TIMSK0 = 0x01;
    TCNT0  = 65140;

    // set pin modes
    DDRB = 0x02;    // PB1:OUTPUT
    PORTB = 0x00;   // PB1:LOW

    // initialize state machine variables
    rcvstate = STATE_IDLE;

    sei();  // enable interrupts
}

/******************************************************************************
    概  要：
    関  数：    loop()
    引  数：
    戻り値：    なし
    機  能：
    備  考：
******************************************************************************/
void loop( void )
{
}

/******************************************************************************
    概  要：    初期化
    関  数：    ir_init()
    引  数：    なし
    戻り値：    なし
    機  能：    通信データの受信開始
    備  考：
******************************************************************************/
ISR(TIM0_OVF_vect)
{
    static uint32_t data_buf;
    static uint16_t irtimer;       // state timer, counts 50uS ticks.

    uint16_t custom_code;
    uint8_t  data_code;
    uint8_t  irdata;

    TCNT0 = 65140;

    irdata = (uint8_t)(PINB & _BV(RECV_PIN));
    irtimer++;                      // One more 50us tick

    switch(rcvstate)
    {
        case STATE_IDLE:            // In the middle of a gap
            if (irdata == MARK)
            {
                if (irtimer >= GAP_TICKS)
                {
                    rcvstate = STATE_HDR_MARK;
                    irtimer = 0;
                }
            }
            break;
        case STATE_HDR_MARK:        // timing Header MARK
            if (irdata != MARK)
            {
                if (!MATCH_MARK(irtimer, NEC_HDR_MARK))
                {
                    rcvstate = STATE_STOP;
                }
                else
                {
                    rcvstate = STATE_HDR_SPACE;
                }
                irtimer = 0;
            }
            break;
        case STATE_HDR_SPACE:       // timing Header SPACE
            if (irdata == MARK)
            {
                if (!MATCH_MARK(irtimer, NEC_HDR_SPACE))
                {
                    // Repeat disabled
                    rcvstate = STATE_STOP;
                }
                else
                {
                    rcvstate = STATE_BIT_MARK;
                    data_buf = 0;
                }
                irtimer = 0;
            }
            break;
        case STATE_BIT_MARK:        // timing Bit MARK
            if (irdata != MARK)
            {
                if (!MATCH_MARK(irtimer, NEC_BIT_MARK))
                {
                    rcvstate = STATE_STOP;
                }
                else
                {
                    rcvstate = STATE_BIT_SPACE;
                }
                irtimer = 0;
            }
            break;
        case STATE_BIT_SPACE:       // timing Bit SPACE
            if (irdata == MARK)
            {
                if (MATCH_SPACE(irtimer, NEC_ONE_SPACE))
                {
                    data_buf = (data_buf << 1) | 1;
                    rcvstate = STATE_BIT_MARK;
                }
                else if (MATCH_SPACE(irtimer, NEC_ZERO_SPACE))
                {
                    data_buf <<= 1;
                    rcvstate = STATE_BIT_MARK;
                }
                else
                {
                    rcvstate = STATE_STOP;
                }
                irtimer = 0;
            }
            else  // SPACE
            {
                if (irtimer > GAP_TICKS)
                {
                    // Success
                    custom_code = (word)(data_buf >> 16);
                    data_code   = (byte)(data_buf >>  8);

                    if ( custom_code == (uint16_t)CUSTOMCODE )
                    {
                        // Emergency enabled
                        if ((data_code == 0x00) || (data_code == 0xD8))
                        {
                            PORTB |= 0x02;      // PB1 ON
                        }

                        // Emergency disabled
                        if (data_code == 0x80)
                        {
                            PORTB &= ~0x02;     // PB1 OFF
                        }
                    }

                    rcvstate = STATE_STOP;
                    irtimer = 0;
                }
            }
            break;
        case STATE_STOP:            // waiting, measuring gap
            if (irdata == MARK)     // reset gap timer
            {
                rcvstate = STATE_IDLE;
                irtimer = 0;
            }
            break;
    }
}
