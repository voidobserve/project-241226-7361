/******************************************************************************
;  *       @型号                   : MC32F7361
;  *       @创建日期               : 2021.12.21
;  *       @公司/作者              : SINOMCU-FAE
;  *       @晟矽微技术支持         : 2048615934
;  *       @晟矽微官网             : http://www.sinomcu.com/
;  *       @版权                   : 2021 SINOMCU公司版权所有.
;  *---------------------- 建议 ---------------------------------
;  *                   变量定义时使用全局变量
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

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

#define USE_MY_DEBUG 0

// 使用的芯片/仿真板:
// USE_7361_CHIP
// USE_7361_BOARD
// USE_7351_CHIP
#define USE_7351_CHIP

#define ADCDETECT_CHARING_THRESHOLD 2048 // 检测是否充电的adc值

#define BAT_FIX_VAL (1773 / 1000)

#ifdef BAT_FIX_VAL
// #define ADCDETECT_BAT_FULL (3720) //  (3720--对应8.52V)
// #define ADCDETECT_BAT_FULL (3666) //  (3666--对应8.4V,但是实际测得在8.51-8.52V才停止)
// #define ADCDETECT_BAT_FULL (3613) // 计算得出是8.27V， 是在3666的基础上减去一定值（补偿）,实际测得是8.36V
// #define ADCDETECT_BAT_FULL (3626) //  实际测试是8.37
// #define ADCDETECT_BAT_FULL (3639) //  实际测试是8.39V
#define ADCDETECT_BAT_FULL (3642) //  计算是8.34V, 实际测试是8.40 V
// #define ADCDETECT_BAT_FULL (3644) //  实际测试是8.41V
// #define ADCDETECT_BAT_FULL (3647) //  实际测试是 8.41V
// #define ADCDETECT_BAT_FULL (3666) // 3666.90,计算是8.40V
#define ADCDETECT_BAT_NULL_EX (280)
#define ADCDETECT_BAT_WILL_FULL (3472) // (1958)
#define ADCVAL_REF_BAT_6_0_V (2618)	   // (1477)

#else

#define ADCDETECT_BAT_FULL (2098)
// #define ADCDETECT_BAT_FULL (2083)	   // 充电时，检测电池是否充满电的adc值 充电时电池电压+压差==8.40V,实际充到电池实际的电压在8.24V认为满电
// #define ADCDETECT_BAT_FULL (2112)	   // 充电时，检测电池是否充满电的adc值 8.58V (8.42还未检测充满，用电源来模拟--是在8.6V左右检测到充满)
// #define ADCDETECT_BAT_FULL (2075)	   // 充电时，检测电池是否充满电的adc值 8.44V（直接用分压系数来算是8.427845），实际是充到8.28V后，认为满电
#define ADCDETECT_BAT_WILL_FULL (1958) // 电池将要满电的adc值
// #define ADCDETECT_BAT_EMPTY 1352	   // 电池为空，那么充电时只有充电的电压（5.5V电池电压）

// #define ADCVAL_REF_BAT_6_7_V 1647 // 电池电压6.7V对应的ad值
// #define ADCVAL_REF_BAT_6_4_V 1573 // 电池电压6.4V对应的ad值
// #define ADCVAL_REF_BAT_6_5_V 1597 // 电池电压6.5V对应的ad值
// #define ADCVAL_REF_BAT_5_7_V 1403 // 电池电压5.7V对应的ad值 （实际是5.52V左右，电池保护板先断开了，实际上还没有检测到这个值就保护了）
#define ADCVAL_REF_BAT_6_0_V 1477	   // 电池电压6.0V对应的ad值

#endif

// #define ONE_CYCLE_TIME_MS 89 // 一次主循环的耗时，单位：ms

// ===================================================
// 低电量相关配置                                    //
// ===================================================
// 分压系数：470K / (470K + 1M)
// #define LOW_BATTERY_AD_VAL (3055) // 低电量对应的ad值 (3055,对应7V)
// #define LOW_BATTERY_AD_VAL (2837) // 低电量对应的ad值 (2837,对应6.5V)
// #define LOW_BATTERY_AD_VAL (2985) // 低电量对应的ad值 (2985,对应6.84V)
#define LOW_BATTERY_AD_VAL (2837) // 低电量对应的ad值 (2837,对应6.5V)
// #define LOW_BATTERY_AD_VAL (2794) // 低电量对应的ad值 (2794,对应6.4V)

// #define SHUT_DOWN_AD_VAL (2794) // 关机电压对应的ad值(2794,对应6.4V，实际测试是在6.45~6.46V左右关机)
// #define SHUT_DOWN_AD_VAL (2750) // 关机电压对应的ad值(2750,对应6.3V)
#define SHUT_DOWN_AD_VAL (2707) // 关机电压对应的ad值(2707,对应6.2V)

// ===================================================
// 充电相关配置                                      //
// ===================================================
// #define TMP_BAT_VAL_FIX 55 // 额外固定增益

/*
    充电时的电池电压ad值和未充电时电池电压ad值，他们之前的ad值差值

	检测电池电压 1M上拉、470K下拉 
    检测电池电压的分压系数 == 470K / (470K + 1M)
    约为 0.31972789115646258503401360544218

    如果用内部 3 V参考电压，12位精度（0-4096），
    那么 1单位ad值 相当于电池电池电压：
    0.00229076628989361702127659574468  V 
*/
// #define ADC_BAT_DIFF_VAL (55) // 测得充电时 xxA
// #define ADC_BAT_DIFF_VAL (65) // 
// #define ADC_BAT_DIFF_VAL (75) // 

#define ADC_BAT_DIFF_VAL (100) // 
// #define ADC_BAT_DIFF_VAL (105) // 
// #define ADC_BAT_DIFF_VAL (110) // 1.0-1.3AX

// #define ADC_BAT_DIFF_VAL (120) // 
// #define ADC_BAT_DIFF_VAL (130) // 
#define WAIT_CIRCUIT_STABLIZE_TIMES (5) // 等待电路稳定时间,单位:ms
// #define WAIT_CIRCUIT_STABLIZE_TIMES (10) // 等待电路稳定时间


// struct tmp_bat_val_fix
// {
//     u16 adc_bat_val;
//     u8 tmp_bat_val_fix;
// };
// struct tmp_bat_val_fix bat_val_fix_table[] = {
//     // table内每增减一项，对应会增减4个字节
//     // { 2619, TMP_BAT_VAL_FIX + 37 },  // 6.0V
//     { 2837, TMP_BAT_VAL_FIX + 37 },  // 6.5V
//     { 3056, TMP_BAT_VAL_FIX + 27 },  // 7.0V
//     { 3188, TMP_BAT_VAL_FIX + 16 },  // 7.3V
//     { 3326, TMP_BAT_VAL_FIX + 0  },  // 7.62V
// };

// ===================================================
// 机械按键相关配置                                  //
// ===================================================
// 检测按键状态的引脚定义，检测到低电平为有效

// 定义按键对应的键值(id)
enum
{
	KEY_ID_NONE = 0,
	KEY_ID_MODE,
	KEY_ID_HEAT,
};

// 检测开关与模式按键的引脚
#define KEY_MODE_PIN P01D
// 检测加热的引脚
#define KEY_HEAT_PIN P11D
#define KEY_SCAN_TIME (10) // 按键扫描时间 ，单位： ms

// 如果只消抖2次，会过滤不掉抖动
#define KEY_FILTER_TIMES (3) // 按键消抖次数 (消抖时间 == 消抖次数 * 按键扫描时间)

enum
{
	KEY_EVENT_NONE = 0,	  // 无按键事件
	KEY_EVENT_HEAT_PRESS, // 加热按键短按
	KEY_EVENT_MODE_PRESS, // 开关/模式按键短按
	KEY_EVENT_MODE_HOLD,  // 开关/模式按键长按
};
volatile u8 key_event; // 存放按键事件的变量

// ===================================================
// LED相关配置                                      //
// ===================================================
// 驱动指示灯的引脚定义
#define LED_WORKING_PIN P14D	 // 工作指示灯
#define LED_CHARGING_PIN P04D	 // 充电指示灯
#define LED_FULL_CHARGE_PIN P03D // 满电指示灯
#define LED_RED					 // 红灯
#define LED_GREEN				 // 绿灯
#define LED_BLUE				 // 蓝灯

#define CONTROL_HEAT_PIN P12D // 驱动控制加热的引脚
#define LED_ON 0			  // LED点亮时，对应的驱动电平
#define LED_OFF 1			  // LED熄灭时，对应的驱动电平

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
/*实际测得给高电平为加热*/
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

// 定义adc检测的引脚，用在adc切换检测引脚
enum
{
	ADC_PIN_P00_AN0 = 1, // 检测是否有充电的电压
	ADC_PIN_P02_AN1,	 // 检测电池分压后的电压的引脚
};

// 定义模式，三种不同频率的模式
enum
{
	MODE_1 = 0, // 一上电，按下电源按键，使用的模式
	MODE_2,
	MODE_3,
	MODE_4,
};
volatile u8 mode_flag; // 存放模式的标志位

// 定义按键的状态
enum
{
	KEY_NONE = 0,
	KEY_HEAT_PRESS,	  // 控制加热的按键按下
	KEY_CHANGE_PRESS, // 控制模式的按键按下
	KEY_POWER_PRESS,  // 电源按键按下
};
volatile u8 key_press_flag; // 存放按键状态的标志位

volatile u8 i; // 循环计数值

volatile u16 adc_val; // 存放adc检测到的数值

volatile u32 turn_dir_ms_cnt;  // 控制在运行时每2min切换一次转向的计时变量
volatile u32 shut_down_ms_cnt; // 毫秒计数(用于运行15min后自动关机)

volatile u32 adc_bat_val;	   // 存放检测到的电池电压的adc值
volatile u16 adc_charging_val; // 存放检测到的充电电压的adc值

// volatile u16 adc_initial_charging_val; // 存放初始的充电电压

volatile u32 max_pwm_val;	   // 临时存放最大占空比对应的值
volatile u16 last_pwm_val;	   // 记录之前的pwm占空比的值
volatile u16 tmp_val;		   // 临时存放需要调节的占空比对应的值
volatile u16 tmp_val_l[8];
volatile u8 tmp_val_cnt;
volatile u8 flag_bat_is_empty; // 标志位，用于检测是否拔出了电池

volatile u16 tmp_bat_val;	   // 存放检测到的电池电压+计算的压差对应的adc值
volatile u8 over_charging_cnt; // 在充电时，检测电池是否满电的计数值
volatile u8 full_charge_cnt;   // 检测到充满电后，进行计数的变量
//

// 定义变量
#define PWM_MAX_LEVEL 100 // PWM等级数（亮度级别）
#define BREATH_PERIOD 200 // 呼吸周期（ms）

static uint8_t pwm_duty;		 // 当前PWM占空比
static uint16_t pwm_counter;	 // PWM计数器
static uint8_t breath_counter;	 // 呼吸效果计数器
static uint8_t breath_direction; // 呼吸方向：0-渐亮，1-渐暗
static uint8_t led_state;		 // LED状态

// 中断服务程序使用到的两个变量：
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
volatile bit_flag flag3;

#define FLAG_IS_DEVICE_OPEN flag1.bits.bit0		// 设备是否开机的标志位，0--未开机，1--开机
#define FLAG_IS_HEATING flag1.bits.bit1			// 加热是否工作的标志位
#define FLAG_IS_IN_CHARGING flag1.bits.bit2		// 是否处于充电的标志位
#define FLAG_DIR flag1.bits.bit3				// 正转，反转的标志位， 0--正转（默认是0为正转），1--反转
#define FLAG_BAT_IS_NEED_CHARGE flag1.bits.bit4 // 电池是否需要充电的标志位, 0--不需要充电，1--需要充电
#define FLAG_BAT_IS_FULL flag1.bits.bit5		// 电池是否满电的标志位，0--未满电，1--满电
#define FLAG_IS_NOT_OPEN_DEVICE flag1.bits.bit6 // 是否允许开机的标志位，0--允许开机，1--不允许开机（但是可以充电）

#define FLAG_DURING_CHARGING_BAT_IS_NULL flag1.bits.bit7 // 标志位，在充电时检测到电池是否为空，0--不为空，1--在充电时，电池为空

#define flag_ctl_device_open flag2.bits.bit0 // 控制标志位，控制打开/关闭设备
#define flag_ctl_heat_open flag2.bits.bit1	 // 控制标志位，控制 加热的 开/关

#define flag_is_low_battery flag2.bits.bit2 // 标志位，是否检测到低电量

#define flag_ctl_dir flag3.bits.bit0   // 控制标志位，是否要切换方向
#define flag_ctl_speed flag3.bits.bit1 // 控制标志位，是否要切换电机转速

#define flag_maybe_low_battery flag3.bits.bit2 // 标志位，可能检测到了低电量

#define flag_tim_scan_maybe_shut_down flag3.bits.bit3 // 标志位，可能需要关机，由定时器扫描，定时器累计持续一段时间后，确认真的需要关机
#define flag_is_needed_shut_down flag3.bits.bit4	  // 标志位，是否检测到了低电压关机，0--否，1--是，由对应的功能来执行关机

#define flag_is_update_current flag3.bits.bit5 // 是否到了更新充电电流的时间

// 毫秒级延时 (误差：在1%以内，1ms、10ms、100ms延时的误差均小于1%)
// 前提条件：FCPU = FHOSC / 4
void delay_ms(u16 xms)
{
	while (xms)
	{
		u16 i = 572;
		while (i--)
		{
			Nop();
		}
		xms--; // 把 --操作放在while()判断条件外面，更节省空间

		__asm;
		clrwdt; // 喂狗
		__endasm;
	}
}

// #if USE_MY_DEBUG
#define DEBUG_PIN P16D
#if 0  // 以下程序约占用81字节空间
// 通过一个引脚输出数据(发送一次约400ms)
// #define DEBUG_PIN P22D
void send_data_msb(u32 send_data)
{
	// 先发送格式头
	// __set_input_pull_up(); // 高电平
	DEBUG_PIN = 1;
	delay_ms(15);
	// __set_output_open_drain(); // 低电平
	DEBUG_PIN = 0;
	delay_ms(7); //

	for (u8 i = 0; i < 32; i++)
	{
		if ((send_data >> (32 - 1 - i)) & 0x01)
		{
			// 如果要发送逻辑1
			// __set_input_pull_up();  	   // 高电平
			DEBUG_PIN = 1;
			delay_ms(5); //
			// __set_output_open_drain(); // 低电平
			DEBUG_PIN = 0;
			delay_ms(10); //
		}
		else
		{
			// 如果要发送逻辑0
			// __set_input_pull_up();  	   // 高电平
			DEBUG_PIN = 1;
			delay_ms(5); //
			// __set_output_open_drain(); // 低电平
			DEBUG_PIN = 0;
			delay_ms(5); //
		}
	}

	// 最后，设置为低电平
	// __set_output_open_drain(); // 低电平
	DEBUG_PIN = 0;
	delay_ms(1);
	DEBUG_PIN = 1;
	delay_ms(1);
	DEBUG_PIN = 0;
}
#endif // #if USE_MY_DEBUG

#endif

/**************************** end of file *********************************************/
