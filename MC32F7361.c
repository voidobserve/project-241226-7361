/******************************************************************************
;  *       @型号                 : MC32F7361
;  *       @创建日期             : 2021.12.21
;  *       @公司/作者            : SINOMCU-FAE
;  *       @晟矽微技术支持       : 2048615934
;  *       @晟矽微官网           : http://www.sinomcu.com/
;  *       @版权                 : 2021 SINOMCU公司版权所有.
;  *----------------------摘要描述---------------------------------
;  *
******************************************************************************/

#include "user.h"

/************************************************
;  *    @函数名          : CLR_RAM
;  *    @说明            : 清RAM
;  *    @输入参数        :
;  *    @返回参数        :
;  ***********************************************/
void CLR_RAM(void)
{
    for (FSR0 = 0; FSR0 < 0xff; FSR0++)
    {
        INDF0 = 0x00;
    }
    FSR0 = 0xFF;
    INDF0 = 0x00;
}
/************************************************
;  *    @函数名            : IO_Init
;  *    @说明              : IO初始化
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
// void IO_Init(void)
// {
//     // IOP0 = 0x00;   // io口数据位
//     // IOP0 = 0x18;   // 不点亮LED
//     IOP0 = (0x01 << 4) | (0x01 << 3); // 不点亮LED
//     OEP0 = 0x3F;                      // io口方向 1:out  0:in
//     PUP0 = 0x00;                      // io口上拉电阻   1:enable  0:disable
//     PDP0 = 0x00;                      // io口下拉电阻   1:enable  0:disable
//     P0ADCR = 0x00;                    // io类型选择  1:模拟输入  0:通用io

//     // IOP1 = 0x00;   // io口数据位
//     IOP1 = 0x10;   // 不点亮LED
//     OEP1 = 0xFF;   // io口方向 1:out  0:in
//     PUP1 = 0x00;   // io口上拉电阻   1:enable  0:disable
//     PDP1 = 0x00;   // io口下拉电阻   1:enable  0:disable
//     P1ADCR = 0x00; // io类型选择  1:模拟输入  0:通用io

//     IOP2 = 0x00; // io口数据位
//     OEP2 = 0xFF; // io口方向 1:out  0:in
//     PUP2 = 0x00; // io口上拉电阻   1:enable  0:disable
//     PDP2 = 0x00; // io口下拉电阻   1:enable  0:disablea

//     PMOD = 0x00;  // P00、P01、P13 io端口值从寄存器读，推挽输出
//     DRVCR = 0x80; // 普通驱动
// }

// adc配置
void adc_config(void)
{
    // 检测是否有充电的引脚：
    P00OE = 0; // 输入模式
    P00DC = 1; // 模拟模式

    // 检测电池降压后的电压的引脚：
    P02OE = 0; // 输入模式
    P02DC = 1; // 模拟模式

    ADCR0 = 0x0A; // 12位精度、不启用ADC转换，不使能ADC
    ADCR1 = 0xE1; // 125K采样（最高精度）  内部3V参考电压
    ADCR2 = 0xFF; // ADC采样时间为15个ADC时钟
    ADEN = 1;     // 使能ADC
}

// 定时器0的pwm配置--输出引脚 P16
// 控制正向的pwm
// void timer0_pwm_config(void)
// {
//     // ====================================================
//     // 接近15.645KHz版本，前提条件：FCPU = FOSC / 4
//     // T0CR |= 0x02; // 4分频

//     T0LOAD = 172 - 1;
//     // T0DATA = 80; // 占空比 == T0DATA / T0LOAD （占空比等到调节转速和方向时再调节）
//     PWM0EC = 0;  // 禁止PWM输出
//     T0CR = 0x81; // 使能定时器，时钟源为FCPU，2分频
// }

// 定时器1pwm配置，输出引脚 P17
// 控制反向的pwm
// void timer1_pwm_config(void)
// {
//     // ====================================================
//     // 接近15.645KHz版本，前提条件：FCPU = FOSC / 4
//     // T1CR |= 0x02; // 4分频
//     T1LOAD = 172 - 1;
//     // T1DATA = 25; // 占空比 == T1DATA / T1LOAD
//     PWM1EC = 0;  // 禁止PWM1输出
//     T1CR = 0x81; // 使能定时器，时钟源为FCPU，2分频
// }

// 定时器2的PWM配置，输出引脚 P15
// 控制充电的pwm
// void timer2_pwm_config(void)
// {
//     // 时钟源选择：FTMR，由TMR配置时钟源
//     T2CKS0 = 1;
//     T2CKS1 = 0;
//     // 定时器高频时钟 FTMR 频率选择 FHOSC/2：
//     TMRCKS0 = 0;
//     TMRCKS1 = 1;

//     T2LOAD = 209; //
//     T2DATA = 0;
//     PWM2EC = 0; // 禁止PWM输出
//     // PWM2EC = 1;	 // 使能PWM输出
//     T2EN = 1;
// }

// 定时器3
// void timer3_config(void)
// {
//     // T3LOAD = 250 - 1; // FCPU 32分频后，这里是1ms触发一次中断
//     // T3CR = 0x85;      // 使能定时器，时钟源选择FCPU，32分频
//     // T3IE = 1;

//     T3LOAD = 25 - 1; //
//     T3CR = 0x85;     // 使能定时器，时钟源选择FCPU，32分频
//     T3IE = 1;
// }

// 按键检测引脚的配置：
// void key_config(void)
// {
//     // 检测加热按键的引脚配置：
//     P11PU = 1; // 上拉
//     P11OE = 0; // 输入模式

//     // 检测 开关机/模式切换 按键的引脚 配置 ：
//     P01PU = 1; // 上拉
//     P01OE = 0; // 输入模式
// }

// 切换adc检测的引脚
void adc_sel_pin(u8 adc_pin)
{
    // 根据传参，切换成对应的通道
    switch (adc_pin)
    {
    case ADC_PIN_P00_AN0:      // 检测是否有充电的引脚
        ADCR0 &= ~(0x0F << 4); // 清空寄存器的通道选择位
        // 清空后的通道就是 AIN0--P00

        // 下面这种写法更占用程序空间：
        // ADCHS3 = 0;
        // ADCHS2 = 0;
        // ADCHS1 = 0;
        // ADCHS0 = 0;
        break;

    case ADC_PIN_P02_AN1:      // 检测电池分压后的引脚
        ADCR0 &= ~(0x0F << 4); // 清空寄存器的通道选择位
        ADCR0 |= 0x01 << 4;    // AIN1--P02;

        // 下面这种写法更占用程序空间：
        // ADCHS3 = 0;
        // ADCHS2 = 0;
        // ADCHS1 = 0;
        // ADCHS0 = 1;
        break;

    default:
        break;
    }

    delay_ms(1); // 切换adc检测的引脚后，要延时一段时间，等待adc稳定，防止意料之外的检测结果
}

// 获取adc单次转换后的值
u16 adc_get_val(void)
{
    u8 i = 0; // adc采集次数的计数
    u16 g_temp_value = 0;
    u32 g_tmpbuff = 0;
    u16 g_adcmax = 0;
    u16 g_adcmin = 0xFFFF;

    // 采集20次，去掉前两次采样，再去掉一个最大值和一个最小值，再取平均值
    for (i = 0; i < 20; i++)
    {
        ADEOC = 0; // 清除ADC转换完成标志位，启动AD转换
        while (!ADEOC)
            ;                // 等待转换完成
        g_temp_value = ADRH; // 取出转换后的值
        g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);
        if (i < 2)
            continue; // 丢弃前两次采样的
        if (g_temp_value > g_adcmax)
            g_adcmax = g_temp_value; // 最大
        if (g_temp_value < g_adcmin)
            g_adcmin = g_temp_value; // 最小
        g_tmpbuff += g_temp_value;
    }
    g_tmpbuff -= g_adcmax;           // 去掉一个最大
    g_tmpbuff -= g_adcmin;           // 去掉一个最小
    g_temp_value = (g_tmpbuff >> 4); // 除以16，取平均值

// 用7361仿真板进行调试时，这里采集到的ad值要加上193，才接近7351芯片采集到的ad值
#ifdef USE_7361_BOARD
    for (i = 0; i < 193; i++)
    {
        if (g_temp_value < 4095)
        {
            g_temp_value++;
        }
    }
#endif // #ifdef USE_7361_BOARD

#ifdef USE_7361_CHIP
    // 用7361芯片进行调试时，采集到的ad值只需要加上 14，便接近7351芯片采集到的ad值
    for (i = 0; i < 14; i++)
    {
        if (g_temp_value < 4095)
        {
            g_temp_value++;
        }
    }
#endif // #ifdef USE_7361_CHIP

    return g_temp_value;
}

// 获取adc单次转换后的值
u16 adc_get_val_once(void)
{
    u16 g_temp_value = 0;
    ADEOC = 0; // 清除ADC转换完成标志位，启动AD转换
    while (!ADEOC)
        ;                // 等待转换完成
    g_temp_value = ADRH; // 取出转换后的值
    g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);

// 用7361仿真板进行调试时，这里采集到的ad值要加上193，才接近7351芯片采集到的ad值
#ifdef USE_7361_BOARD
    for (i = 0; i < 193; i++)
    {
        if (g_temp_value < 4095)
        {
            g_temp_value++;
        }
    }
#endif // #ifdef USE_7361_BOARD

#ifdef USE_7361_CHIP
    // 用7361芯片进行调试时，采集到的ad值只需要加上 14，便接近7351芯片采集到的ad值
    for (i = 0; i < 14; i++)
    {
        if (g_temp_value < 4095)
        {
            g_temp_value++;
        }
    }
#endif // #ifdef USE_7361_CHIP

    return g_temp_value;
}

/************************************************
;  *    @Function Name       : Sys_Init
;  *    @Description         : 系统初始化
;  *    @IN_Parameter      	 :
;  *    @Return parameter    :
;  ***********************************************/
void Sys_Init(void)
{
    GIE = 0;
    CLR_RAM();
    // IO_Init();
    {
        IOP0 = (0x01 << 4) | (0x01 << 3); // 不点亮LED
        OEP0 = 0x3F;                      // io口方向 1:out  0:in
        PUP0 = 0x00;                      // io口上拉电阻   1:enable  0:disable
        PDP0 = 0x00;                      // io口下拉电阻   1:enable  0:disable
        P0ADCR = 0x00;                    // io类型选择  1:模拟输入  0:通用io

        // IOP1 = 0x00;   // io口数据位
        IOP1 = 0x10;   // 不点亮LED
        OEP1 = 0xFF;   // io口方向 1:out  0:in
        PUP1 = 0x00;   // io口上拉电阻   1:enable  0:disable
        PDP1 = 0x00;   // io口下拉电阻   1:enable  0:disable
        P1ADCR = 0x00; // io类型选择  1:模拟输入  0:通用io

        IOP2 = 0x00; // io口数据位
        OEP2 = 0xFF; // io口方向 1:out  0:in
        PUP2 = 0x00; // io口上拉电阻   1:enable  0:disable
        PDP2 = 0x00; // io口下拉电阻   1:enable  0:disablea

        PMOD = 0x00;  // P00、P01、P13 io端口值从寄存器读，推挽输出
        DRVCR = 0x80; // 普通驱动
    }
    // 关闭所有指示灯
    LED_WORKING_OFF();
    LED_FULL_CHARGE_OFF();
    LED_CHARGING_OFF();

    // timer0_pwm_config();
    {
        T0LOAD = 172 - 1;
        // T0DATA = 80; // 占空比 == T0DATA / T0LOAD （占空比等到调节转速和方向时再调节）
        PWM0EC = 0;  // 禁止PWM输出
        T0CR = 0x81; // 使能定时器，时钟源为FCPU，2分频
    }
    // timer1_pwm_config();
    {
        // ====================================================
        // 接近15.645KHz版本，前提条件：FCPU = FOSC / 4
        // T1CR |= 0x02; // 4分频
        T1LOAD = 172 - 1;
        // T1DATA = 25; // 占空比 == T1DATA / T1LOAD
        PWM1EC = 0;  // 禁止PWM1输出
        T1CR = 0x81; // 使能定时器，时钟源为FCPU，2分频
    }
    // timer2_pwm_config();
    {
        /* 改成使用125KHz的PWM */
        // 时钟源选择：FTMR，由TMR配置时钟源
        T2CKS1 = 0;
        T2CKS0 = 1;
        // 定时器高频时钟 FTMR 频率选择 FHOSC
        TMRCKS1 = 1;
        TMRCKS0 = 0;

        // 定时器时钟2分频：
        T2PRS2 = 0;
        T2PRS1 = 0;
        T2PRS0 = 1;

        // T2LOAD = 209; //
        // T2LOAD = 136; // 125.654Khz
        // T2LOAD = 137; // 125KHz【实际测试只有116.5KHz】
        // T2LOAD = 130;// 【实际测试是122-123KHz】
        // T2LOAD = 129;// 【实际测试是123.7KHz】
        // T2LOAD = 125;// 【126-127.6KHz】
        // T2LOAD = 126;// 【126KHz】
        T2LOAD = 127; // 【125或125.654KHz】

        T2DATA = 0;
        PWM2EC = 0; // 禁止PWM输出
        T2EN = 1;
    }

    // timer3_config();
    {
        // T3LOAD = 250 - 1; // FCPU 32分频后，这里是1ms触发一次中断
        // T3CR = 0x85;      // 使能定时器，时钟源选择FCPU，32分频
        // T3IE = 1;

        T3LOAD = 25 - 1; //
        T3CR = 0x85;     // 使能定时器，时钟源选择FCPU，32分频
        T3IE = 1;
    }

    // key_config();
    {
        // 检测加热按键的引脚配置：
        P11PU = 1; // 上拉
        P11OE = 0; // 输入模式

        // 检测 开关机/模式切换 按键的引脚 配置 ：
        P01PU = 1; // 上拉
        P01OE = 0; // 输入模式
    }
    adc_config();

    GIE = 1;

    delay_ms(1); // 等待系统稳定
}

// 获取按键对应的键值(id)
// u8 get_key_id(void)
// {
//     u8 ret = KEY_ID_NONE;
//     if (0 == KEY_MODE_PIN)
//     {
//         ret = KEY_ID_MODE;
//     }
//     else if (0 == KEY_HEAT_PIN)
//     {
//         ret = KEY_ID_HEAT;
//     }

//     return ret;
// }

// 按键扫描函数，最后会得到对应的按键事件
// 需要放在10ms的定时器中
void key_scan(void)
{
    static volatile u8 last_key_id = KEY_ID_NONE;
    static volatile u8 press_cnt = 0;               // 按键按下的时间计数
    static volatile u8 filter_cnt = 0;              // 按键消抖，使用的变量
    static volatile u8 filter_key_id = KEY_ID_NONE; // 按键消抖时使用的变量
    // volatile u8 cur_key_id = get_key_id();          // 获取当前按键的键值(id)
    volatile u8 cur_key_id = KEY_ID_NONE;

    static volatile u8 flag_is_key_mode_hold = 0;

    if (0 == KEY_MODE_PIN)
    {
        cur_key_id = KEY_ID_MODE;
    }
    else if (0 == KEY_HEAT_PIN)
    {
        cur_key_id = KEY_ID_HEAT;
    }

    if (cur_key_id != filter_key_id)
    {
        // 如果有按键按下/松开
        filter_cnt = 0;
        filter_key_id = cur_key_id;
        return;
    }

    if (filter_cnt < KEY_FILTER_TIMES)
    {
        // 如果检测到相同的按键按下/松开
        // 防止计数溢出
        filter_cnt++;
        return;
    }

    // 滤波/消抖完成后，执行到这里

    if (last_key_id != cur_key_id)
    {
        if (last_key_id == KEY_ID_NONE)
        {
            // 如果有按键按下，清除按键按下的时间计数
            press_cnt = 0;
        }
        else if (cur_key_id == KEY_ID_NONE)
        {
            // 如果按键松开
            if (press_cnt < 75)
            {
                // 按下时间小于 750ms ，是短按
                if (KEY_ID_MODE == last_key_id)
                {
                    // 开关/模式按键短按
                    key_event = KEY_EVENT_MODE_PRESS;

                    // PWM0EC = 0;
                    // P16OE = 1;
                    // DEBUG_PIN = ~DEBUG_PIN;
                }
                else if (KEY_ID_HEAT == last_key_id)
                {
                    // 加热按键短按
                    key_event = KEY_EVENT_HEAT_PRESS;
                }
            }
            else
            {
                // 长按、长按持续之后松手
                // key_event = KEY_EVENT_NONE;

                if (KEY_ID_MODE == last_key_id)
                {
                    flag_is_key_mode_hold = 0;
                }
            }
        }
    }
    else if (cur_key_id != KEY_ID_NONE)
    {
        // 如果按键按住不放
        if (press_cnt < 255)
            press_cnt++;

        if (KEY_ID_MODE == cur_key_id)
        {
            if (FLAG_IS_DEVICE_OPEN)
            {
                if (press_cnt >= 200) // 2000ms
                {
                    if (flag_is_key_mode_hold)
                    {
                    }
                    else
                    {
                        key_event = KEY_EVENT_MODE_HOLD;
                        flag_is_key_mode_hold = 1;
                    }
                }
            }
            else
            {
                // 如果当前设备是关闭的
                if (press_cnt >= 100) // 1000ms加上看门狗唤醒的1024ms
                {
                    if (flag_is_key_mode_hold)
                    {
                    }
                    else
                    {
                        key_event = KEY_EVENT_MODE_HOLD;
                        flag_is_key_mode_hold = 1;
                    }
                }
            }
        }
    }

    last_key_id = cur_key_id;
}

// 对扫描到的按键事件进行处理
void key_event_handle(void)
{
    if (FLAG_IS_DEVICE_OPEN)
    {
        // 如果设备正在运行
        if (KEY_EVENT_MODE_HOLD == key_event)
        {
            // 关机：
            LED_WORKING_OFF(); // 关闭电源指示灯
            LED_CHARGING_OFF();
            HEATING_OFF(); // 关闭加热
            FLAG_IS_DEVICE_OPEN = 0;
            FLAG_IS_HEATING = 0;

            if (flag_is_low_battery)
            {
                // 如果关机前处于 低电量报警 的状态
                flag_is_low_battery = 0;
                LED_CHARGING_OFF(); // 关闭用于报警的LED
            }

            mode_flag = MODE_1; // 下一次切换模式时，会变成 MODE_2

            // 关闭 正转和反转的PWM
            PWM0EC = 0;
            PWM1EC = 0;

            inflation_ctl_status = INFLATION_CTL_STATUS_DEFLATION; // 放气
        }
        else if (KEY_EVENT_MODE_PRESS == key_event)
        {
            if (MODE_1 == mode_flag)
            {
                // 设置PWM的占空比
                T0DATA = 160;
                T1DATA = 160;

                mode_flag = MODE_2;
                flag_ctl_dir = 1;
            }
            else if (MODE_2 == mode_flag)
            {
                // 设置PWM的占空比
                T0DATA = 255;
                T1DATA = 255; // 100%占空比（确保大于TxLOAD的值就可以）

                mode_flag = MODE_3;
                flag_ctl_dir = 1;
            }
            else if (MODE_3 == mode_flag)
            {
                // 设置PWM的占空比
                T0DATA = 0;
                T1DATA = 0;
                PWM0EC = 0;
                PWM1EC = 0;
                mode_flag = MODE_4;
            }
            else if (MODE_4 == mode_flag)
            {
                // PWM0EC = 1;
                // PWM1EC = 1;
                T0DATA = 150;
                T1DATA = 150;
                mode_flag = MODE_1;
                flag_ctl_dir = 1;
            }

            turn_dir_ms_cnt = 0; // 重置切换方向的计数
        }
        else if (KEY_EVENT_HEAT_PRESS == key_event)
        {
            if (0 == FLAG_IS_HEATING)
            {
                HEATING_ON(); // 打开加热
                LED_WORKING_OFF();
                LED_CHARGING_ON();
                FLAG_IS_HEATING = 1;
            }
            else
            {
                HEATING_OFF(); // 关闭加热
                LED_CHARGING_OFF();
                LED_WORKING_ON();
                FLAG_IS_HEATING = 0;
            }
        }
    }
    else
    {
        // 如果设备没有在运行
        if (KEY_EVENT_MODE_HOLD == key_event && 0 == FLAG_IS_IN_CHARGING)
        {
            LED_FULL_CHARGE_OFF(); // 关闭满电指示灯
            LED_CHARGING_OFF();    // 关闭充电指示灯
            LED_WORKING_ON();      // 打开电源指示灯
            FLAG_IS_DEVICE_OPEN = 1;

            // 设定正转、反转的PWM的初始占空比
            T0DATA = 150;
            T1DATA = 150;
            mode_flag = MODE_1; // 下一次切换模式时，会变成 MODE_2

            // // 打开控制正转的PWM
            // PWM0EC = 1;

            PWM1EC = 1;

            inflation_ctl_status = INFLATION_CTL_STATUS_INFLATION; // 充气
        }
    }

    // 处理完成后，清除按键事件
    key_event = KEY_EVENT_NONE;
}

void adc_scan_handle(void)
{
    volatile u8 cnt = 0;             // 计数值，用于检测电池是否满电，也是用于检测是否插入/拔出充电器的计数值
    volatile u8 need_charge_cnt = 0; // 计数值，用于检测是否要快速充电

    adc_bat_val = 0;
    adc_charging_val = 0;

    adc_sel_pin(ADC_PIN_P02_AN1); // 切换到检测电池降压后的电压的检测引脚
    for (i = 0; i < 10; i++)      //  // 每55ms进入一次，循环内每次间隔约4.8ms
    {
        adc_val = adc_get_val();

        if (i == 0)
        {
            adc_bat_val = adc_val;
        }

        if (FLAG_IS_IN_CHARGING)
        {
            // 如果检测到充满电（可能触发了电池保护板的过充保护），直接输出较低的PWM
            if (adc_val >= ADCDETECT_BAT_FULL + ADCDETECT_BAT_NULL_EX)
            {
                // T2DATA = 10; // 触发过充保护后会一直进入这里，充电电流维持在0.03不变，此时电池两端电压在12-13V
                T2DATA = 0;
                over_charging_cnt++;
            }

            if (adc_val >= ADCDETECT_BAT_FULL) // 检测电池是否满电
            {
                cnt++;
            }
            else if (adc_val < ADCDETECT_BAT_WILL_FULL) // 检测是否要快速充电
            {
                need_charge_cnt++;
            }

            if (flag_bat_is_empty)
            {
// 如果检测到拔出了电池
#if 1                               // 新版的功能--要求，在充电时，没有电池的情况下，让绿灯快闪
                LED_CHARGING_OFF(); // 关闭充电指示灯
                PWM2EC = 0;         // 关闭控制升压电路的pwm（建议是，插上充钱器时，不关闭控制升压电路的PWM，但是在测试时，充电一侧会严重发热）
                T2DATA = 0;
                FLAG_BAT_IS_NEED_CHARGE = 0;
                FLAG_DURING_CHARGING_BAT_IS_NULL = 1; // 标志位置一，在主循环让绿灯快闪
#endif
                break;
            }
            else if (cnt >= 8 || over_charging_cnt >= 8)
            {
                // 如果在充电时，电池之前需要充电，现在检测到充满电
                full_charge_cnt++;
                if (full_charge_cnt >= 100 || over_charging_cnt >= 8) // 5.5s或者是连续触发了若干次过充保护
                {
                    full_charge_cnt = 0; //

                    PWM2EC = 0; // 关闭控制升压电路的pwm
                    T2DATA = 0;
                    // FLAG_IS_IN_CHARGING = 0; // 不能给这个标志位清零（交给充电扫描来清零）
                    FLAG_BAT_IS_NEED_CHARGE = 0;
                    FLAG_BAT_IS_FULL = 1;

                    LED_CHARGING_OFF(); // 关闭充电指示灯
                    // LED_WORKING_ON();   // 开启工作指示灯（蓝灯常亮）
                    // LED_FULL_CHARGE_ON(); // 开启电池充满电的指示灯
                    LED_WORKING_ON(); // 打开电源指示灯，表示充满电
                    break;
                }
            }
            else if (need_charge_cnt >= 8)
            {
                // 如果在充电，且电池需要充电（电池电量小于 ADCDETECT_BAT_WILL_FULL，电池将要满电的电压）
                // 根据当前的电池电压来决定要输出多大的PWM，电池电压越小，PWM占空比也要小，否则充电电流会过大
                LED_FULL_CHARGE_OFF(); // 关闭充满电的指示灯
                FLAG_BAT_IS_NEED_CHARGE = 1;
                FLAG_BAT_IS_FULL = 0;
                break;
            }
        } // if (FLAG_IS_IN_CHARGING)
        else // 如果未在充电，检测电池是否需要充电
        {
            // if (adc_val <= ADCDETECT_BAT_FULL - 130)
            if (flag_bat_is_empty == 0 && adc_val <= ADCDETECT_BAT_FULL - ((ADCDETECT_BAT_FULL - ADCDETECT_BAT_WILL_FULL) / 2))
            {
                cnt++;
            }

            if (cnt >= 8)
            {
                // 如果未在充电，且电池需要充电（电池电量小于 满电-死区电量 ）
                FLAG_BAT_IS_NEED_CHARGE = 1;
                FLAG_BAT_IS_FULL = 0;
                break;
            }

            if (                           /* 检测电池是否处于关机对应的电压 */
                flag_bat_is_empty == 0 &&  /* 电池不为空 */
                FLAG_IS_DEVICE_OPEN &&     /* 设备正在运行 */
                adc_val < SHUT_DOWN_AD_VAL /* 采集到的ad值小于关机对应的ad值 */
            )
            {
                flag_tim_scan_maybe_shut_down = 1;
            }
            else if (flag_bat_is_empty == 0 &&
                     FLAG_IS_DEVICE_OPEN &&
                     adc_val < LOW_BATTERY_AD_VAL)
            {
                // 检测电池是否处于低电量：
                flag_maybe_low_battery = 1;
                flag_tim_scan_maybe_shut_down = 0;
            }
            else
            {
                flag_maybe_low_battery = 0;
                flag_tim_scan_maybe_shut_down = 0;
            }

            // 如果真的到了关机对应的电压
            if (flag_is_needed_shut_down && FLAG_IS_DEVICE_OPEN)
            {
                flag_is_needed_shut_down = 0;
                // 关机：
                LED_WORKING_OFF(); // 关闭电源指示灯
                LED_CHARGING_OFF();
                HEATING_OFF(); // 关闭加热
                FLAG_IS_DEVICE_OPEN = 0;
                FLAG_IS_HEATING = 0;

                if (flag_is_low_battery)
                {
                    // 如果关机前处于低电量报警的状态
                    flag_is_low_battery = 0;
                    LED_CHARGING_OFF(); // 关闭用于报警的LED
                }

                mode_flag = MODE_1; // 下一次切换模式时，会变成 MODE_2

                // 关闭 正转和反转的PWM
                PWM0EC = 0;
                PWM1EC = 0;

                inflation_ctl_status = INFLATION_CTL_STATUS_DEFLATION; // 放气
            }
        }

        // key_scan_handle(); // 按键扫描和处理函数
        key_event_handle();
    } // for (i = 0; i < 10; i++)

    cnt = 0;
    adc_sel_pin(ADC_PIN_P00_AN0); // 切换到检测充电的电压检测引脚(检测到的充电电压 == USB-C口电压 / 2)
    for (i = 0; i < 10; i++)
    {
        adc_val = adc_get_val();
        if (flag_bat_is_empty) // 如果电池为空
        {
            adc_val = 4095; // 防止进入下面的if (adc_val < ADCDETECT_CHARING_THRESHOLD)
        }

        if (adc_charging_val != 0)
        {
            adc_charging_val += adc_val;
            adc_charging_val /= 2;
        }
        else
        {
            adc_charging_val += adc_val;
        }

        if (FLAG_IS_IN_CHARGING)
        {
            // 如果正在充电，检测是否拔出了充电线
            if (adc_val < ADCDETECT_CHARING_THRESHOLD)
            {
                cnt++;
            }

            if (cnt >= 8)
            {
                // 如果在充电时，检测到拔出了充电线
                FLAG_IS_IN_CHARGING = 0;
                LED_CHARGING_OFF(); // 关闭充电指示灯
                PWM2EC = 0;         // 关闭控制升压电路的pwm
                T2DATA = 0;
                LED_FULL_CHARGE_OFF(); // 关闭电池充满电的指示灯
                LED_WORKING_OFF();     // 关闭电池充满电的指示灯（白灯）

                FLAG_DURING_CHARGING_BAT_IS_NULL = 0; // 清空该标志位，因为已经不在充电的情况下
                break;
            } // if (cnt >= 8)

            if (FLAG_IS_DEVICE_OPEN)
            {
            }
            else // 如果设备未开启
            {
                if (FLAG_BAT_IS_FULL)
                {
                }
                else
                {
                    // 如果未满电
                    LED_FULL_CHARGE_OFF();
                    LED_WORKING_OFF();
                    // LED_CHARGING_ON();
                }
            }
        } // if (FLAG_IS_IN_CHARGING)
        else
        {
            // 如果不在充电，检测是否插入了充电线
            if (flag_bat_is_empty == 0 && adc_val >= ADCDETECT_CHARING_THRESHOLD)
            {
                cnt++;
            }

            if (cnt >= 8)
            {
                // 确认是插入充电线后，无论处于什么状态，都变为关机状态
                // adc_initial_charging_val = adc_get_val();
                // adc_initial_charging_val = adc_charging_val; // 只有刚插入充电器时，才更新初始的充电电压对应的ad值

                full_charge_cnt = 0;
                over_charging_cnt = 0;

                FLAG_IS_IN_CHARGING = 1;
                // LED_CHARGING_ON(); // 开启充电指示灯
                // T2DATA = 0;
                PWM2EC = 1; // 开启控制升压电路的pwm
                LED_FULL_CHARGE_OFF();
                LED_WORKING_OFF(); // 关闭电源指示灯
                HEATING_OFF();     // 关闭加热
                FLAG_IS_DEVICE_OPEN = 0;
                FLAG_IS_HEATING = 0;
                mode_flag = MODE_1; // 下一次切换模式时，会变成 MODE_2
                // 关闭 正转和反转的PWM
                PWM0EC = 0;
                PWM1EC = 0;
                inflation_ctl_status = INFLATION_CTL_STATUS_DEFLATION; // 放气

                flag_is_low_battery = 0; // 清除该标志位
                break;
            } // if (cnt >= 8)
            // if (cnt >= 8)
            // {
            //     // 确认是插入充电线后
            //     full_charge_cnt = 0;
            //     over_charging_cnt = 0;

            //     if (FLAG_IS_DEVICE_OPEN)
            //     {
            //         LED_WORKING_ON(); // 打开工作指示灯
            //     }
            //     else
            //     {
            //         // 设备不工作时，才打开该指示灯
            //         LED_CHARGING_ON(); // 开启充电指示灯
            //     }

            //     PWM2EC = 1; // 开启控制升压电路的pwm
            //     FLAG_IS_IN_CHARGING = 1;
            //     break;
            // } // if (cnt >= 8)
        }

        // key_scan_handle(); // 按键扫描和处理函数
        key_event_handle();
    } // for (i = 0; i < 10; i++)
}

void turn_dir_scan_handle(void)
{
    if (FLAG_IS_DEVICE_OPEN)
    {
        // 设备运行时，才开始计时并判断是否要转向
        // turn_dir_ms_cnt += ONE_CYCLE_TIME_MS;
        if (turn_dir_ms_cnt >= (120000) ||
            flag_ctl_dir)
        {
            if (0 == FLAG_DIR)
            {
                PWM1EC = 0;
                delay_ms(1000); // 电机状态切换需要延时等待放电，否则会拉住电机，无法驱动
                PWM0EC = 1;
                FLAG_DIR = 1; //
            }
            else
            {
                PWM0EC = 0;     //
                delay_ms(1000); // 电机状态切换需要延时等待放电，否则会拉住电机，无法驱动
                PWM1EC = 1;     //
                FLAG_DIR = 0;   //
            }

            flag_ctl_dir = 0;
            turn_dir_ms_cnt = 0;
        }
    }
    else
    {
        turn_dir_ms_cnt = 0; // 设备未运行时，清空计数值
    }
}

// 关机检测和处理
void shutdown_scan_handle(void)
{
    if (FLAG_IS_DEVICE_OPEN)
    {
        // 如果设备开启，开始计时，15min后自动关机
        // shut_down_ms_cnt += ONE_CYCLE_TIME_MS;
        if (shut_down_ms_cnt >= 900000)
        {
            // 如果超过了15min，关机：
            LED_WORKING_OFF(); // 关闭电源指示灯
            LED_CHARGING_OFF();
            HEATING_OFF(); // 关闭加热
            FLAG_IS_HEATING = 0;
            mode_flag = MODE_4; // 下一次切换模式时，会变成 MODE_1
            // 关闭 正转和反转的PWM
            PWM0EC = 0;
            PWM1EC = 0;

            FLAG_IS_DEVICE_OPEN = 0;

            inflation_ctl_status = INFLATION_CTL_STATUS_DEFLATION; // 放气
        }
    }
    else
    {
        // 如果设备未开启，清空计时
        shut_down_ms_cnt = 0;
    }
}

void low_power_scan_handle(void)
{
    // if (FLAG_DURING_CHARGING_BAT_IS_NULL)
    if (FLAG_DURING_CHARGING_BAT_IS_NULL ||               // 只插着充电器且没有电池时，不进入低功耗
        FLAG_IS_DEVICE_OPEN ||                            // 如果设备已经启动，不进入低功耗
        FLAG_IS_IN_CHARGING ||                            // 如果正在充电，不进入低功耗，因为还需要输出PWM来控制充电
        (0 == P01D) ||                                    // 如果检测到 开关/模式 按键按下，不进入低功耗(交给按键事件处理函数来判断是否要开机)
        INFLATION_CTL_STATUS_NONE != inflation_ctl_status // 气泵、气阀还在工作，不进入低功耗
    )
    {
        return;
    }

    // if (FLAG_IS_DEVICE_OPEN)
    // {
    //     // 如果设备已经启动，不进入低功耗
    //     return;
    // }

    // // 如果运行到这里，说明设备没有启动
    // // 可能需要考虑正在充电的情况
    // if (FLAG_IS_IN_CHARGING)
    // {
    //     // 如果正在充电，不进入低功耗，因为还需要输出PWM来控制充电
    //     return;
    // }

label:
    GIE = 0;  // 禁用所有中断
    T0EN = 0; // 关闭定时器和PWM输出
    T1EN = 0;
    T2EN = 0;
    T3EN = 0;

    PWM0EC = 0;
    PWM1EC = 0;
    PWM2EC = 0;
    T2DATA = 0;
    ADEN = 0; // 不使能ad

    // LED脚配置为输入模式（从外部来看，相当于高阻态）：
    // P14OE = 0;
    // P04OE = 0;
    // P03OE = 0;

    LED_WORKING_OFF();
    LED_CHARGING_OFF();
    LED_FULL_CHARGE_OFF();

    // 开关/模式按键的配置，配置为输入上拉
    P01PU = 1;
    P01OE = 0;

    // 配置充电检测引脚
    P00PD = 1;
    P00OE = 0;

    HFEN = 0; // 关闭高速时钟
    LFEN = 1;

    // 休眠前关闭外设
    Nop();
    Nop();
    Stop();
    Nop();
    Nop();

    HFEN = 1; // 开启高速时钟
    // 唤醒后，重新初始化
    CLR_RAM();

    adc_config();
    adc_sel_pin(ADC_PIN_P00_AN0);
    adc_val = adc_get_val_once();
    // 如果按下开机按键/插入充电器，不会满足条件：
    if (adc_val < ADCDETECT_CHARING_THRESHOLD && P01D)
    {
        // 如果没有按下开机按键、没有插入充电器
        // 关闭ADC，继续进入休眠：
        // ADEN = 0; //
        goto label;
    }

    // P02PU = 0; // 关闭上下拉
    // P02PD = 0;
    // P02OE = 0; // 输入模式
    // P02DC = 1; // 模拟输入
    // adc_sel_pin(ADC_PIN_P02_AN1);
    // adc_val = adc_get_val();
    // if (adc_val < ADCVAL_REF_BAT_6_0_V)
    // {
    //     // 如果电池电量过低，不开机，但是可以充电
    //     FLAG_IS_NOT_OPEN_DEVICE = 1; // 不允许开机，但是可以充电
    // }
    // else
    // {
    //     FLAG_IS_NOT_OPEN_DEVICE = 0;
    // }

    // FLAG_IS_NOT_OPEN_DEVICE = 0;

    Sys_Init();
    // GIE = 1;

    // delay_ms(1); // 等待系统稳定
}

void main(void)
{
    Sys_Init();

    // delay_ms(1); // 等待系统稳定

    // flag_bat_is_empty = 0; // （可以优化掉，上电默认就是0）

    // 上电时检测电池是否正确安装:
    PWM2EC = 1; // 打开控制升压电路的pwm
    T2DATA = 100;
    adc_sel_pin(ADC_PIN_P02_AN1); // 切换到检测电池降压后的电压的检测引脚
    for (i = 0; i < 10; i++)      // 每55ms进入一次，循环内每次间隔约4.8ms
    {
        adc_val = adc_get_val();
        if (adc_val >= ADCDETECT_BAT_FULL + ADCDETECT_BAT_NULL_EX)
        {
            flag_bat_is_empty = 1;
            FLAG_DURING_CHARGING_BAT_IS_NULL = 1; // 标志位置一，在主循环让绿灯快闪
        }
    }
    PWM2EC = 0; // 关闭控制升压电路的pwm
    T2DATA = 0;

    while (1)
    {
#if 1
        // DEBUG_PIN = ~DEBUG_PIN; // 测试主循环是否正常

        // P10D = 1; // 测试一次循环所需的时间

        if (flag_bat_is_empty)
        {
            // 如果电池为空，让对应的LED闪烁
            LED_CHARGING_PIN = LED_ON;
            delay_ms(500);
            LED_CHARGING_PIN = LED_OFF;
            delay_ms(500);
            continue;
        }

        // key_scan_handle(); // 按键扫描和处理函数
        key_event_handle(); // 按键事件处理函数，扫描在中断服务函数内执行
        adc_scan_handle();  // 充电扫描和处理函数(约占用89ms)

        turn_dir_scan_handle();
        shutdown_scan_handle(); // 自动关机检测和处理函数

        // 根据电池电压和充电口的电压来调节控制充电的PWM占空比
        // |-- 在充电时检测电池是否为空，如果为空，让绿灯快闪
        if (FLAG_IS_IN_CHARGING)
        {
            last_pwm_val = T2DATA;      // 读出上一次PWM占空比对应的值
            max_pwm_val = (T2LOAD + 1); // 读出PWM占空比设定的、最大的值

            /*
                修改电压差值，电压差值 = 203 - (adc_bat_val * 122 / 1000)

                推导过程：
                在充电时测得，充电电流1.1A左右，压差为-30(ad值)时，电池一侧电压为7.8V(ad值：1917)
                             充电电流1.1A左右，压差为0(ad值)时，电池一侧电压为6.8V(ad值：1671)
                假设x轴为电压对应的ad值，y轴为压差对应的ad值，建立参考坐标系
                根据这两个测试点，发现x轴正向增长，y轴负向增长，画出的斜线向下，斜率为负，求出斜率
                    k = Δy/Δx = (0 - 30) / (1917 - 1671)，约为 -0.122
                建立公式：y = kx + b，代入，解得 b 约为 203 （四舍五入是204）
                y = kx + b ==> 压差 = -0.122 * 充电时的电池电压 + 203
                转换成单片机可以计算的形式：压差 = 203 - (充电时的电池电压 * 122 / 1000)
            */

            /*
                检测电池电压 1M上拉、470K下拉
                检测电池电压的分压系数 == 470K / (470K + 1M)
                约为 0.31972789115646258503401360544218
            */
#if 1
            tmp_bat_val = adc_bat_val;
            if (adc_bat_val <= 2837) // 如果检测电池电压小于 6.5V
            {
                // tmp_bat_val += 30;
                tmp_bat_val += 70; /* 6.25--1.02，6.35--1.08 */
            }
            else if (adc_bat_val <= 3056) // 如果检测电池电压小于 7.0V
            {
                // tmp_bat_val += 30; //
                tmp_bat_val += 50; // 6.64--1.01，6.70--1.03，6.80--1.028，6.90--1.10
                // tmp_bat_val += 70; //
            }
            else if (adc_bat_val <= 3188) // 如果检测电池电压小于 7.3V
            {
                // tmp_bat_val += 20; //
                tmp_bat_val += 35; // 7.20--1.08，7.25--1.04
                // tmp_bat_val += 40; // 7.02--1.06，7.13--1.07，7.17--1.115
                // tmp_bat_val += 60; //
            }
            else if (adc_bat_val <= 3326) // 如果检测电池电压小于 7.62V
            {
                // tmp_bat_val += 10; //
                tmp_bat_val += 20; // 7.33--0.991，7.37--1.03，7.40--1.021，7.50--1.05
                // tmp_bat_val += 30; // 7.33--1.087
                // tmp_bat_val += 40; //
            }
            else // 如果在充电时检测到电池电压大于
            {
                /*
                    tmp_bat_val += 15;  这个时候常态下可能只有0.97，但是动一下线路板或者线缆，会跳到1.07A
                    7.76--0.975，7.78--1.04，8.00--1.084
                */
                tmp_bat_val += 15;
                // tmp_bat_val += 25; //
                // tmp_bat_val += 30; // 7.70--1.06，7.73--1.100，
                // tmp_bat_val += 35; //
                // tmp_bat_val += 40; // 7.70--1.16，8.07V -- 1.06A，8.2V之后好像会升到1.1A，8.23V--1.08A
                // tmp_bat_val += 45; //
                // tmp_bat_val += 50; // 在8.08V会到1.10
                // tmp_bat_val += 52; //
                // tmp_bat_val += 55; //
                // tmp_bat_val += 60; //   超过8V会到1.10
                // tmp_bat_val += 70; // 超过8V时会超过1.1A，导致电感发热
                tmp_bat_val -= ((u32)adc_bat_val * 157 / 1000 - 522);
            }

            tmp_bat_val += 50;
            // tmp_bat_val += 70;
            // tmp_bat_val += 80; // ==========
            // tmp_bat_val += 90;

            // if (adc_bat_val >= 3579) // 8.2V及以上 , 降低电流
            if (adc_bat_val >= 3623) // 8.30V及以上 , 降低电流
            {
                u16 i;
                // for (i = 0; i < 40; i++) //
                // for (i = 0; i < 50; i++) //
                // for (i = 0; i < 70; i++) // 8.33--0.93A
                // for (i = 0; i < 75; i++) // 8.33--0.86A
                // for (i = 0; i < 80; i++) // 8.34--0.88
                // for (i = 0; i < 85; i++) // 8.33V--0.62A
                // for (i = 0; i < 90; i++) // 8.33--0.866，
                // for (i = 0; i < 100; i++) // 8.32--0.90A，8.34--0.89
                for (i = 0; i < 120; i++) // 8.32--0.737，8.34--0.725
                {
                    if (tmp_bat_val > 2)
                    {
                        tmp_bat_val--;
                    }
                }
            }
#endif

            /*
                升压公式：Vo = Vi / (1 - D)

                通过PWM来控制升压，这里假设当前PWM占空比寄存器的值 为 D，PWM占空比寄存器可以到的最大的值 为 1
                Vo = Vi / (PWM占空比寄存器可以到的最大的值 - 当前PWM占空比寄存器的值)
                当前PWM占空比越大，Vo也越大，充电的电流也会越大

                (PWM占空比寄存器可以到的最大的值 - 当前PWM占空比寄存器的值) = Vi / Vo
                当前PWM占空比寄存器的值 = PWM占空比寄存器可以到的最大的值 - Vi / Vo

                这里检测到的充电电压的ad值 == USB-C口电压 / 2[上下拉电阻分压] / 参考电压[3V，那么这里就是除以3] * 4096[ad转换精度，12位-->0~4096]
                即，这里检测到的充电电压的ad值 == USB-C口电压 / 2 / 3 * 4096
                检测到的电池电压的ad值 == 电池电压 * 0.18 / 3V参考电压 * 4096 == 电池电压 * 220 / 1220 / 3V参考电压 * 4096
                (电池的分压电阻： 上拉220K，下拉1M，分压系数： 220 / 1220)

                检测充电电压和检测电池电压使用的不是同一个分压系数，要一起运算时，这里将充电电压的ad再 * 2 * 220 / 1220
                即 (adc_charging_val * 22 / 61)

                再代回公式：当前PWM占空比寄存器的值 = PWM占空比寄存器可以到的最大的值 - Vi / Vo
                当前PWM占空比寄存器的值 = PWM占空比寄存器可以到的最大的值 - 充电电压 / 充电时电池两侧的电压
                tmp_val = max_pwm_val - 充电电压 / 充电时电池两侧的电压
                转换成单片机可以计算的形式：
                tmp_val = max_pwm_val - (adc_charging_val * 22 / 61) / tmp_bat_val，但是 max_pwm_val 的值不是1，不符合 Vo = Vi / (1 - D)
                这里要改成 tmp_val = max_pwm_val - max_pwm_val * (adc_charging_val * 22 / 61) / tmp_bat_val
                tmp_val = max_pwm_val - (adc_charging_val * max_pwm_val * 22 / 61) / tmp_bat_val
            */
            // D = 1 - (Vi / Vo)
            // tmp_val = max_pwm_val - (adc_charging_val * max_pwm_val * 22 / 61) / tmp_bat_val;
            tmp_val = max_pwm_val - (adc_charging_val * max_pwm_val * 94 / 147) / tmp_bat_val;

            // adc_charging_val 改成了 adc_initial_charging_val，只使用刚插入充电头时对应的ad值来代入公式：
            // tmp_val = max_pwm_val - ((u32)adc_initial_charging_val * max_pwm_val * 94 / 147) / tmp_bat_val;

            if (tmp_val >= max_pwm_val)
            {
                // 如果PWM占空比对应的值 大于 最大占空比对应的值，说明计算溢出（可能是电池电压过小），按0处理
                tmp_val = 0;
            }

            // 滤波操作，一开始tmp_val会很小，采集多次后趋于一个平均值：
            tmp_val_cnt++;
            tmp_val_cnt &= 0x07;
            tmp_val_l[tmp_val_cnt] = (tmp_val_l[tmp_val_cnt] + tmp_val) >> 1;
            tmp_val = 0;
            for (i = 0; i < 8; i++)
            {
                tmp_val += tmp_val_l[i];
            }
            tmp_val >>= 3;

            // {
            //     /*
            //         如果差值过大，则快速调节，如果差值过小，则慢速调节，
            //         防止电流突变，导致不同的板子最终充电电流不一致
            //     */
            //     static u8 cnt = 0;
            //     cnt++;

            //     if (tmp_val > last_pwm_val)
            //     {
            //         if ((tmp_val - last_pwm_val) > 2 || cnt >= 10)
            //         // if ((tmp_val - last_pwm_val) > 2 || cnt >= 100)
            //         {
            //             last_pwm_val++;
            //             cnt = 0;
            //         }
            //     }
            //     else if (tmp_val < last_pwm_val)
            //     {
            //         if ((last_pwm_val - tmp_val) > 2 || cnt >= 10)
            //         // if ((last_pwm_val - tmp_val) > 2 || cnt >= 100)
            //         {
            //             last_pwm_val--;
            //             cnt = 0;
            //         }
            //     }
            // }

            {
                /*
                    如果差值过大，则快速调节，如果差值过小，则慢速调节，
                    防止电流突变，导致不同的板子最终充电电流不一致
                */
                // static u8 cnt = 0;
                // cnt++;

                if (flag_is_adjust_pwm_time_comes)
                {
                    flag_is_adjust_pwm_time_comes = 0;

                    if (tmp_val > last_pwm_val)
                    {
                        // if ((tmp_val - last_pwm_val) > 2 || cnt >= 10)
                        // if ((tmp_val - last_pwm_val) > 2 || cnt >= 100)
                        {
                            last_pwm_val++;
                            // cnt = 0;
                        }
                    }
                    else if (tmp_val < last_pwm_val)
                    {
                        // if ((last_pwm_val - tmp_val) > 2 || cnt >= 10)
                        // if ((last_pwm_val - tmp_val) > 2 || cnt >= 100)
                        {
                            last_pwm_val--;
                            // cnt = 0;
                        }
                    }
                }
            }

#if 0 // 使用检测充电前后电池电压变化的差值来控制充电电流

            u16 adc_bat_val_when_charging;     // 充电时的电池电压
            u16 adc_bat_val_when_not_charging; // 未充电时的电池电压
            u8 adjust_pwm_val_dir;             // 调整方向

            if (flag_is_update_current)
            {                
                flag_is_update_current = 0;
                PWM2EC = 1; // 使能升压的PWM
                delay_ms(WAIT_CIRCUIT_STABLIZE_TIMES);
                adc_sel_pin(ADC_PIN_P02_AN1);
                adc_bat_val_when_charging = adc_get_val();

                PWM2EC = 0; // 不使能升压的PWM
                delay_ms(WAIT_CIRCUIT_STABLIZE_TIMES);
                adc_bat_val_when_not_charging = adc_get_val();

                if (adc_bat_val_when_charging > adc_bat_val_when_not_charging) /* 如果充电时，测得的ad值比没有充电时的ad值大 */
                {
                    if ((adc_bat_val_when_charging - adc_bat_val_when_not_charging) > ADC_BAT_DIFF_VAL) /* 如果充电时和没有充电时的差值大于设定的差值 */
                    {
                        adjust_pwm_val_dir = 0;
                    }
                    else
                    {
                        adjust_pwm_val_dir = 1;
                    }
                }
                else
                {
                    adjust_pwm_val_dir = 1;
                }

                if (adjust_pwm_val_dir)
                {
                    if (last_pwm_val < max_pwm_val)
                    {
                        last_pwm_val++;
                    }
                }
                else
                {
                    if (last_pwm_val >= 1)
                    {
                        last_pwm_val--;
                    }
                }
            }

            PWM2EC = 1; // 使能升压的PWM

#endif // 使用检测充电前后电池电压变化的差值来控制充电电流

            T2DATA = last_pwm_val;

        } // if (FLAG_IS_IN_CHARGING)
        else // 如果未在充电
        {
            if (FLAG_DURING_CHARGING_BAT_IS_NULL)
            {
                // 如果电池未安装
                LED_FULL_CHARGE_ON();
                delay_ms(200);
                LED_FULL_CHARGE_OFF();
                delay_ms(200);
            }
        } // else // 如果未在充电

        if (flag_is_low_battery)
        {
            LED_WORKING_OFF();

            // 工作时检测到低电量
            LED_CHARGING_ON();
            // delay_ms(200);
            for (i = 0; i < 20; i++)
            {
                // key_scan_handle();
                key_event_handle();
                delay_ms(10);
            }

            LED_CHARGING_OFF();
            // delay_ms(200);
            for (i = 0; i < 20; i++)
            {
                // key_scan_handle();
                key_event_handle();
                delay_ms(10);
            }
        }

        low_power_scan_handle();

        // P10D = 0; // 测试一次循环所需的时间

#endif
        __asm;
        CLRWDT; // 喂狗指令
        __endasm;

    } // while (1)
}

/************************************************
;  *    @函数名            : interrupt
;  *    @说明              : 中断函数
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void int_isr(void) __interrupt
{
    __asm;
    movra _abuf;
    swapar _PFLAG;
    movra _statusbuf;
    __endasm;
    //=======外部中断0(由开关按键触发，该中断只用于唤醒单片机)=================
    // if (INT0IF & INT0IE)
    // {
    //     INT0IF = 0;
    // }

    if (T3IF & T3IE)
    {
        {
            static u8 __1ms_cnt = 0;
            __1ms_cnt++;
            if (__1ms_cnt >= 10)
            {
                __1ms_cnt = 0;
                // 目前每1ms进入一次中断
                { // 按键扫描
                    static u8 key_scan_cnt = 0;
                    key_scan_cnt++;
                    if (key_scan_cnt >= 10)
                    {
                        key_scan_cnt = 0;
                        key_scan();
                    }
                } // 按键扫描

                if (FLAG_IS_DEVICE_OPEN)
                {
                    turn_dir_ms_cnt++;  // 会在 turn_dir_scan_handle() 内处理并清零
                    shut_down_ms_cnt++; // 会在 shutdown_scan_handle() 内处理并清零
                }

                { // 低电量检测
                    static u16 low_bat_cnt = 0;
                    // static u16 cancel_low_bat_alarm_cnt = 0; // 取消低电量报警的时间计数

                    if (flag_maybe_low_battery)
                    {
                        low_bat_cnt++;
                        // if (low_bat_cnt >= 2000) // xx ms
                        if (low_bat_cnt >= 5000) // xx ms
                        {
                            low_bat_cnt = 0;
                            flag_is_low_battery = 1;
                        }
                    }
                    else
                    {
                        low_bat_cnt = 0;

                        // if (flag_is_low_battery) // 如果之前处于低电量报警
                        // {
                        // cancel_low_bat_alarm_cnt++;
                        //     if (cancel_low_bat_alarm_cnt >= 2000) // 持续 xx ms检测到电池电量正常，取消低电量报警
                        //     {
                        //         cancel_low_bat_alarm_cnt = 0;
                        //     }
                        // }
                        // else
                        // {
                        //     cancel_low_bat_alarm_cnt = 0;
                        // }
                    }
                } // 低电量检测

                { // 关机电量检测
                    static u16 shut_down_bat_cnt = 0;
                    if (flag_tim_scan_maybe_shut_down)
                    {
                        shut_down_bat_cnt++;
                        if (shut_down_bat_cnt >= 1000) // xx ms
                        {
                            shut_down_bat_cnt = 0;
                            flag_is_needed_shut_down = 1;
                        }
                    }
                    else
                    {
                        shut_down_bat_cnt = 0;
                        flag_is_needed_shut_down = 0;
                    }
                } // 关机电量检测

                { // 充电时，调节电流时间间隔控制
                    static u16 update_current_time_cnt;
                    if (FLAG_IS_IN_CHARGING)
                    {
                        update_current_time_cnt++;
                        if (update_current_time_cnt >= 500)
                        {
                            update_current_time_cnt = 0;
                            flag_is_update_current = 1;
                        }
                    }
                    else
                    {
                        FLAG_IS_IN_CHARGING = 0;
                    }

                } // 充电时，调节电流时间间隔控制

                {
                    static u16 cnt = 0;
                    cnt++;
                    if (cnt > 500)
                    {
                        cnt = 0;
                        flag_is_adjust_pwm_time_comes = 1;
                    }
                }

                {
                    static u16 cnt = 0;
                    cnt++;
                    if (INFLATION_CTL_STATUS_NONE == inflation_ctl_status)
                    {
                        cnt = 0;
                        INFLATION_CTL_OFF();
                        DEFLATION_CTL_OFF();
                    }
                    else if (INFLATION_CTL_STATUS_INFLATION == inflation_ctl_status)
                    {
                        DEFLATION_CTL_OFF();
                        INFLATION_CTL_ON();
                    }
                    else if (INFLATION_CTL_STATUS_DEFLATION == inflation_ctl_status)
                    {
                        INFLATION_CTL_OFF();
                        DEFLATION_CTL_ON();
                    }

                    if (cnt >= (u16)10000)
                    {
                        inflation_ctl_status = INFLATION_CTL_STATUS_NONE;
                    }
                }
            }
        }

        // 呼吸灯控制：
        if (FLAG_IS_IN_CHARGING && 0 == FLAG_BAT_IS_FULL)
        {
            // PWM控制
            if (pwm_counter < pwm_duty)
            {
                led_state = 1; // LED亮
            }
            else
            {
                led_state = 0; // LED灭
            }

            // PWM周期计数（20级PWM，所以计数到19后归零）
            pwm_counter++;
            if (pwm_counter >= PWM_MAX_LEVEL)
            {
                pwm_counter = 0;
            }

            // 呼吸效果控制（每20ms改变一次亮度）
            breath_counter++;
            if (breath_counter >= BREATH_PERIOD)
            {
                breath_counter = 0;

                // 根据方向调整PWM占空比
                if (breath_direction == 0)
                {
                    // 渐亮
                    pwm_duty++;
                    if (pwm_duty >= PWM_MAX_LEVEL)
                    {
                        pwm_duty = PWM_MAX_LEVEL;
                        breath_direction = 1; // 切换到渐暗
                    }
                }
                else
                {
                    // 渐暗
                    if (pwm_duty > 0)
                    {
                        pwm_duty--;
                    }
                    else
                    {
                        pwm_duty = 0;
                        breath_direction = 0; // 切换到渐亮
                    }
                }
            }

            // 输出LED状态
            // LED_SET(led_state);  // 假设这是控制LED的函数
            if (led_state)
            {
                LED_CHARGING_PIN = LED_ON;
            }
            else
            {
                LED_CHARGING_PIN = LED_OFF;
            }
        } // 呼吸灯控制 if (FLAG_IS_IN_CHARGING && 0 == FLAG_BAT_IS_FULL)

        T3IF = 0;
    }

    __asm;
    swapar _statusbuf;
    movra _PFLAG;
    swapr _abuf;
    swapar _abuf;
    __endasm;
}

/**************************** end of file *********************************************/
