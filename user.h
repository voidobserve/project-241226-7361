/******************************************************************************
;  *       @�ͺ�                   : MC32F7361
;  *       @��������               : 2021.12.21
;  *       @��˾/����              : SINOMCU-FAE
;  *       @����΢����֧��         : 2048615934
;  *       @����΢����             : http://www.sinomcu.com/
;  *       @��Ȩ                   : 2021 SINOMCU��˾��Ȩ����.
;  *---------------------- ���� ---------------------------------
;  *                   ��������ʱʹ��ȫ�ֱ���
******************************************************************************/
#ifndef USER
#define USER
#include "mc32-common.h"
#include "MC32F7361.h"

/*****************************************************************
;       Function : Define variables
;*****************************************************************/

#define u8 unsigned char
#define u16 unsigned int
#define u32 unsigned long int
#define uint8_t unsigned char
#define uint16_t unsigned int
#define uint32_t unsigned long int

#define DEF_SET_BIT0 0x01
#define DEF_SET_BIT1 0x02
#define DEF_SET_BIT2 0x04
#define DEF_SET_BIT3 0x08
#define DEF_SET_BIT4 0x10
#define DEF_SET_BIT5 0x20
#define DEF_SET_BIT6 0x40
#define DEF_SET_BIT7 0x80

#define DEF_CLR_BIT0 0xFE
#define DEF_CLR_BIT1 0xFD
#define DEF_CLR_BIT2 0xFB
#define DEF_CLR_BIT3 0xF7
#define DEF_CLR_BIT4 0xEF
#define DEF_CLR_BIT5 0xDF
#define DEF_CLR_BIT6 0xBF
#define DEF_CLR_BIT7 0x7F

#define FAIL 1
#define PASS 0

#define USE_MY_DEBUG 0

// ʹ�õ�оƬ/�����:
// USE_7361_CHIP
// USE_7361_BOARD
// USE_7351_CHIP
#define USE_7351_CHIP

#define ADCDETECT_CHARING_THRESHOLD 2048 // ����Ƿ����adcֵ

#define BAT_FIX_VAL (1773 / 1000)

#ifdef BAT_FIX_VAL
#define ADCDETECT_BAT_FULL (3720) // (2098)
#define ADCDETECT_BAT_NULL_EX (280)
#define ADCDETECT_BAT_WILL_FULL (3472) // (1958)
#define ADCVAL_REF_BAT_6_0_V (2618)	   // (1477)

#else

#define ADCDETECT_BAT_FULL (2098)
// #define ADCDETECT_BAT_FULL (2083)	   // ���ʱ��������Ƿ�������adcֵ ���ʱ��ص�ѹ+ѹ��==8.40V,ʵ�ʳ䵽���ʵ�ʵĵ�ѹ��8.24V��Ϊ����
// #define ADCDETECT_BAT_FULL (2112)	   // ���ʱ��������Ƿ�������adcֵ 8.58V (8.42��δ���������õ�Դ��ģ��--����8.6V���Ҽ�⵽����)
// #define ADCDETECT_BAT_FULL (2075)	   // ���ʱ��������Ƿ�������adcֵ 8.44V��ֱ���÷�ѹϵ��������8.427845����ʵ���ǳ䵽8.28V����Ϊ����
#define ADCDETECT_BAT_WILL_FULL (1958) // ��ؽ�Ҫ�����adcֵ
// #define ADCDETECT_BAT_EMPTY 1352	   // ���Ϊ�գ���ô���ʱֻ�г��ĵ�ѹ��5.5V��ص�ѹ��

// #define ADCVAL_REF_BAT_6_7_V 1647 // ��ص�ѹ6.7V��Ӧ��adֵ
// #define ADCVAL_REF_BAT_6_4_V 1573 // ��ص�ѹ6.4V��Ӧ��adֵ
// #define ADCVAL_REF_BAT_6_5_V 1597 // ��ص�ѹ6.5V��Ӧ��adֵ
// #define ADCVAL_REF_BAT_5_7_V 1403 // ��ص�ѹ5.7V��Ӧ��adֵ ��ʵ����5.52V���ң���ر������ȶϿ��ˣ�ʵ���ϻ�û�м�⵽���ֵ�ͱ����ˣ�
#define ADCVAL_REF_BAT_6_0_V 1477	   // ��ص�ѹ6.0V��Ӧ��adֵ

#endif

#define ONE_CYCLE_TIME_MS 89 // һ����ѭ���ĺ�ʱ����λ��ms

// ����ָʾ�Ƶ����Ŷ���
#define LED_WORKING_PIN P14D	 // ����ָʾ��
#define LED_CHARGING_PIN P04D	 // ���ָʾ��
#define LED_FULL_CHARGE_PIN P03D // ����ָʾ��

#define LED_RED	  // ���
#define LED_GREEN // �̵�
#define LED_BLUE  // ����

// ===================================================
// ��е�����������                                  //
// ===================================================
// ��ⰴ��״̬�����Ŷ��壬��⵽�͵�ƽΪ��Ч
// #define KEY_HEAT_PIN P11D	// �����Ƿ���ȵ�����
// #define KEY_POWER_PIN P11D	// �����Ƿ���������
// #define KEY_CHANGE_PIN P01D // ����ģʽ������
// ���尴����id
enum
{	
	KEY_ID_NONE = 0,
	KEY_ID_MODE,
	KEY_ID_HEAT,
};
#define CONTROL_HEAT_PIN P12D // �������Ƽ��ȵ�����
// ��⿪����ģʽ����������
#define KEY_MODE_PIN P01D
// �����ȵ�����
#define KEY_HEAT_PIN P11D
#define KEY_SCAN_TIME (10) // ����ɨ��ʱ�� ����λ�� ms
#define KEY_FILTER_TIMES (3)// ������������ (����ʱ�� == �������� * ����ɨ��ʱ��)


#define LED_ON 0  // LED����ʱ����Ӧ��������ƽ
#define LED_OFF 1 // LEDϨ��ʱ����Ӧ��������ƽ

#define LED_WORKING_ON()              \
	{                                 \
		do                            \
		{                             \
			LED_WORKING_PIN = LED_ON; \
		} while (0);                  \
	}
#define LED_WORKING_OFF()              \
	{                                  \
		do                             \
		{                              \
			LED_WORKING_PIN = LED_OFF; \
		} while (0);                   \
	}
#define LED_CHARGING_ON()              \
	{                                  \
		do                             \
		{                              \
			LED_CHARGING_PIN = LED_ON; \
		} while (0);                   \
	}
#define LED_CHARGING_OFF()              \
	{                                   \
		do                              \
		{                               \
			LED_CHARGING_PIN = LED_OFF; \
		} while (0);                    \
	}
#define LED_FULL_CHARGE_ON()              \
	{                                     \
		do                                \
		{                                 \
			LED_FULL_CHARGE_PIN = LED_ON; \
		} while (0);                      \
	}
#define LED_FULL_CHARGE_OFF()              \
	{                                      \
		do                                 \
		{                                  \
			LED_FULL_CHARGE_PIN = LED_OFF; \
		} while (0);                       \
	}
/*ʵ�ʲ�ø��ߵ�ƽΪ����*/
#define HEATING_ON()              \
	{                             \
		do                        \
		{                         \
			CONTROL_HEAT_PIN = 1; \
		} while (0);              \
	}
#define HEATING_OFF()             \
	{                             \
		do                        \
		{                         \
			CONTROL_HEAT_PIN = 0; \
		} while (0);              \
	}

// ����adc�������ţ�����adc�л��������
enum
{
	ADC_PIN_P00_AN0 = 1, // ����Ƿ��г��ĵ�ѹ
	ADC_PIN_P02_AN1,	 // ����ؽ�ѹ��ĵ�ѹ������
};

// ����ģʽ�����ֲ�ͬƵ�ʵ�ģʽ
enum
{
	MODE_1 = 0, // һ�ϵ磬���µ�Դ������ʹ�õ�ģʽ
	MODE_2,
	MODE_3,
};
volatile u8 mode_flag; // ���ģʽ�ı�־λ

// ���尴����״̬
enum
{
	KEY_NONE = 0,
	KEY_HEAT_PRESS,	  // ���Ƽ��ȵİ�������
	KEY_CHANGE_PRESS, // ����ģʽ�İ�������
	KEY_POWER_PRESS,  // ��Դ��������
};
volatile u8 key_press_flag; // ��Ű���״̬�ı�־λ

volatile u8 i; // ѭ������ֵ

volatile u16 adc_val; // ���adc��⵽����ֵ

volatile u32 turn_dir_ms_cnt;  // ����������ʱÿ2min�л�һ��ת��ļ�ʱ����
volatile u32 shut_down_ms_cnt; // �������(��������15min���Զ��ػ�)

volatile u32 adc_bat_val;	   // ��ż�⵽�ĵ�ص�ѹ��adcֵ
volatile u16 adc_charging_val; // ��ż�⵽�ĳ���ѹ��adcֵ
volatile u32 max_pwm_val;	   // ��ʱ������ռ�ձȶ�Ӧ��ֵ
volatile u16 last_pwm_val;	   // ��¼֮ǰ��pwmռ�ձȵ�ֵ
volatile u16 tmp_val;		   // ��ʱ�����Ҫ���ڵ�ռ�ձȶ�Ӧ��ֵ
volatile u16 tmp_val_l[8];
volatile u8 tmp_val_cnt;
volatile u8 flag_bat_is_empty; // ��־λ�����ڼ���Ƿ�γ��˵��

volatile u16 tmp_bat_val;	   // ��ż�⵽�ĵ�ص�ѹ+�����ѹ���Ӧ��adcֵ
volatile u8 over_charging_cnt; // �ڳ��ʱ��������Ƿ�����ļ���ֵ
volatile u32 full_charge_cnt;  // ��⵽������󣬽��м����ı���
//


// �жϷ������ʹ�õ�������������
u8 abuf;
u8 statusbuf;

//============Define  Flag=================
typedef union
{
	unsigned char byte;
	struct
	{
		u8 bit0 : 1;
		u8 bit1 : 1;
		u8 bit2 : 1;
		u8 bit3 : 1;
		u8 bit4 : 1;
		u8 bit5 : 1;
		u8 bit6 : 1;
		u8 bit7 : 1;
	} bits;
} bit_flag;
volatile bit_flag flag1;
volatile bit_flag flag2;
#define FLAG_IS_DEVICE_OPEN flag1.bits.bit0		// �豸�Ƿ񿪻��ı�־λ��0--δ������1--����
#define FLAG_IS_HEATING flag1.bits.bit1			// �����Ƿ����ı�־λ
#define FLAG_IS_IN_CHARGING flag1.bits.bit2		// �Ƿ��ڳ��ı�־λ
#define FLAG_DIR flag1.bits.bit3				// ��ת����ת�ı�־λ�� 0--��ת��Ĭ����0Ϊ��ת����1--��ת
#define FLAG_BAT_IS_NEED_CHARGE flag1.bits.bit4 // ����Ƿ���Ҫ���ı�־λ, 0--����Ҫ��磬1--��Ҫ���
#define FLAG_BAT_IS_FULL flag1.bits.bit5		// ����Ƿ�����ı�־λ��0--δ���磬1--����
#define FLAG_IS_NOT_OPEN_DEVICE flag1.bits.bit6 // �Ƿ����������ı�־λ��0--����������1--���������������ǿ��Գ�磩

#define FLAG_DURING_CHARGING_BAT_IS_NULL flag1.bits.bit7 // ��־λ���ڳ��ʱ��⵽����Ƿ�Ϊ�գ�0--��Ϊ�գ�1--�ڳ��ʱ�����Ϊ��

#define flag_ctl_device_open flag2.bits.bit0 // ���Ʊ�־λ�����ƴ�/�ر��豸
#define flag_ctl_heat_open flag2.bits.bit1 // ���Ʊ�־λ������ ���ȵ� ��/��

// #define flag_key_scan_10ms flag2.bits.bit0 // ��־λ,���ڰ�����⣬�Ƿ񾭹���10ms

// ���뼶��ʱ (����1%���ڣ�1ms��10ms��100ms��ʱ������С��1%)
// ǰ��������FCPU = FHOSC / 4
void delay_ms(u16 xms)
{
	while (xms)
	{
		u16 i = 572;
		while (i--)
		{
			Nop();
		}
		xms--; // �� --��������while()�ж��������棬����ʡ�ռ�

		__asm;
		clrwdt; // ι��
		__endasm;
	}
}

// #if USE_MY_DEBUG
#define DEBUG_PIN P22D
#if 0 // ���³���Լռ��81�ֽڿռ�
// ͨ��һ�������������(����һ��Լ400ms)
#define DEBUG_PIN P22D
void send_data_msb(u32 send_data)
{
	// �ȷ��͸�ʽͷ
	// __set_input_pull_up(); // �ߵ�ƽ
	DEBUG_PIN = 1;
	delay_ms(15);
	// __set_output_open_drain(); // �͵�ƽ
	DEBUG_PIN = 0;
	delay_ms(7); //

	for (u8 i = 0; i < 32; i++)
	{
		if ((send_data >> (32 - 1 - i)) & 0x01)
		{
			// ���Ҫ�����߼�1
			// __set_input_pull_up();  	   // �ߵ�ƽ
			DEBUG_PIN = 1;
			delay_ms(5); //
			// __set_output_open_drain(); // �͵�ƽ
			DEBUG_PIN = 0;
			delay_ms(10); //
		}
		else
		{
			// ���Ҫ�����߼�0
			// __set_input_pull_up();  	   // �ߵ�ƽ
			DEBUG_PIN = 1;
			delay_ms(5); //
			// __set_output_open_drain(); // �͵�ƽ
			DEBUG_PIN = 0;
			delay_ms(5); //
		}
	}

	// �������Ϊ�͵�ƽ
	// __set_output_open_drain(); // �͵�ƽ
	DEBUG_PIN = 0;
	delay_ms(1);
	DEBUG_PIN = 1;
	delay_ms(1);
	DEBUG_PIN = 0;
}
#endif // #if USE_MY_DEBUG

#endif

/**************************** end of file *********************************************/