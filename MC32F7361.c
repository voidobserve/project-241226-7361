/******************************************************************************
;  *       @�ͺ�                 : MC32F7361
;  *       @��������             : 2021.12.21
;  *       @��˾/����            : SINOMCU-FAE
;  *       @����΢����֧��       : 2048615934
;  *       @����΢����           : http://www.sinomcu.com/
;  *       @��Ȩ                 : 2021 SINOMCU��˾��Ȩ����.
;  *----------------------ժҪ����---------------------------------
;  *
******************************************************************************/

#include "user.h"

/************************************************
;  *    @������          : CLR_RAM
;  *    @˵��            : ��RAM
;  *    @�������        :
;  *    @���ز���        :
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
;  *    @������            : IO_Init
;  *    @˵��              : IO��ʼ��
;  *    @�������          :
;  *    @���ز���          :
;  ***********************************************/
void IO_Init(void)
{
    // IOP0 = 0x00;   // io������λ
    // IOP0 = 0x18;   // ������LED
    IOP0 = (0x01 << 4) | (0x01 << 3); // ������LED
    OEP0 = 0x3F;                      // io�ڷ��� 1:out  0:in
    PUP0 = 0x00;                      // io����������   1:enable  0:disable
    PDP0 = 0x00;                      // io����������   1:enable  0:disable
    P0ADCR = 0x00;                    // io����ѡ��  1:ģ������  0:ͨ��io

    // IOP1 = 0x00;   // io������λ
    IOP1 = 0x10;   // ������LED
    OEP1 = 0xFF;   // io�ڷ��� 1:out  0:in
    PUP1 = 0x00;   // io����������   1:enable  0:disable
    PDP1 = 0x00;   // io����������   1:enable  0:disable
    P1ADCR = 0x00; // io����ѡ��  1:ģ������  0:ͨ��io

    IOP2 = 0x00; // io������λ
    OEP2 = 0xFF; // io�ڷ��� 1:out  0:in
    PUP2 = 0x00; // io����������   1:enable  0:disable
    PDP2 = 0x00; // io����������   1:enable  0:disablea

    PMOD = 0x00;  // P00��P01��P13 io�˿�ֵ�ӼĴ��������������
    DRVCR = 0x80; // ��ͨ����
}

// adc����
void adc_config(void)
{
    // ����Ƿ��г������ţ�
    P00OE = 0; // ����ģʽ
    P00DC = 1; // ģ��ģʽ

    // ����ؽ�ѹ��ĵ�ѹ�����ţ�
    P02OE = 0; // ����ģʽ
    P02DC = 1; // ģ��ģʽ

    ADCR0 = 0x0A; // 12λ���ȡ�������ADCת������ʹ��ADC
    ADCR1 = 0xE1; // 125K��������߾��ȣ�  �ڲ�3V�ο���ѹ
    ADCR2 = 0xFF; // ADC����ʱ��Ϊ15��ADCʱ��
    ADEN = 1;     // ʹ��ADC
}

// ��ʱ��0��pwm����--������� P16
// ���������pwm
void timer0_pwm_config(void)
{
    // ====================================================
    // �ӽ�15.645KHz�汾��ǰ��������FCPU = FOSC / 4
    // T0CR |= 0x02; // 4��Ƶ

    T0LOAD = 172 - 1;
    // T0DATA = 80; // ռ�ձ� == T0DATA / T0LOAD ��ռ�ձȵȵ�����ת�ٺͷ���ʱ�ٵ��ڣ�
    PWM0EC = 0;  // ��ֹPWM���
    T0CR = 0x81; // ʹ�ܶ�ʱ����ʱ��ԴΪFCPU��2��Ƶ
}

// ��ʱ��1pwm���ã�������� P17
// ���Ʒ����pwm
void timer1_pwm_config(void)
{
    // ====================================================
    // �ӽ�15.645KHz�汾��ǰ��������FCPU = FOSC / 4
    // T1CR |= 0x02; // 4��Ƶ
    T1LOAD = 172 - 1;
    // T1DATA = 25; // ռ�ձ� == T1DATA / T1LOAD
    PWM1EC = 0;  // ��ֹPWM1���
    T1CR = 0x81; // ʹ�ܶ�ʱ����ʱ��ԴΪFCPU��2��Ƶ
}

// ��ʱ��2��PWM���ã�������� P15
// ���Ƴ���pwm
void timer2_pwm_config(void)
{
    // ʱ��Դѡ��FTMR����TMR����ʱ��Դ
    T2CKS0 = 1;
    T2CKS1 = 0;
    // ��ʱ����Ƶʱ�� FTMR Ƶ��ѡ�� FHOSC/2��
    TMRCKS0 = 0;
    TMRCKS1 = 1;

    T2LOAD = 209; //
    T2DATA = 0;
    PWM2EC = 0; // ��ֹPWM���
    // PWM2EC = 1;	 // ʹ��PWM���
    T2EN = 1;
}

// ��ʱ��3
void timer3_config(void)
{
    // T3LOAD = 250 - 1; // FCPU 32��Ƶ��������1ms����һ���ж�
    // T3CR = 0x85;      // ʹ�ܶ�ʱ����ʱ��Դѡ��FCPU��32��Ƶ
    // T3IE = 1;

    T3LOAD = 25 - 1; //
    T3CR = 0x85;     // ʹ�ܶ�ʱ����ʱ��Դѡ��FCPU��32��Ƶ
    T3IE = 1;
}

// ����������ŵ����ã�
void key_config(void)
{
    // �����Ȱ������������ã�
    P11PU = 1; // ����
    P11OE = 0; // ����ģʽ

    // ��� ���ػ�/ģʽ�л� ���������� ���� ��
    P01PU = 1; // ����
    P01OE = 0; // ����ģʽ
}

// �л�adc��������
void adc_sel_pin(u8 adc_pin)
{
    // ���ݴ��Σ��л��ɶ�Ӧ��ͨ��
    switch (adc_pin)
    {
    case ADC_PIN_P00_AN0:
        ADCR0 &= ~(0x0F << 4); // ��ռĴ�����ͨ��ѡ��λ
        // ��պ��ͨ������ AIN0--P00

        // ��������д����ռ�ó���ռ䣺
        // ADCHS3 = 0;
        // ADCHS2 = 0;
        // ADCHS1 = 0;
        // ADCHS0 = 0;
        break;

    case ADC_PIN_P02_AN1:
        ADCR0 &= ~(0x0F << 4); // ��ռĴ�����ͨ��ѡ��λ
        ADCR0 |= 0x01 << 4;    // AIN1--P02;

        // ��������д����ռ�ó���ռ䣺
        // ADCHS3 = 0;
        // ADCHS2 = 0;
        // ADCHS1 = 0;
        // ADCHS0 = 1;
        break;

    default:
        break;
    }

    delay_ms(1); // �л�adc�������ź�Ҫ��ʱһ��ʱ�䣬�ȴ�adc�ȶ�����ֹ����֮��ļ����
}

// ��ȡadc����ת�����ֵ
u16 adc_get_val(void)
{
    u8 i = 0; // adc�ɼ������ļ���
    u16 g_temp_value = 0;
    u32 g_tmpbuff = 0;
    u16 g_adcmax = 0;
    u16 g_adcmin = 0xFFFF;

    // �ɼ�20�Σ�ȥ��ǰ���β�������ȥ��һ�����ֵ��һ����Сֵ����ȡƽ��ֵ
    for (i = 0; i < 20; i++)
    {
        ADEOC = 0; // ���ADCת����ɱ�־λ������ADת��
        while (!ADEOC)
            ;                // �ȴ�ת�����
        g_temp_value = ADRH; // ȡ��ת�����ֵ
        g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);
        if (i < 2)
            continue; // ����ǰ���β�����
        if (g_temp_value > g_adcmax)
            g_adcmax = g_temp_value; // ���
        if (g_temp_value < g_adcmin)
            g_adcmin = g_temp_value; // ��С
        g_tmpbuff += g_temp_value;
    }
    g_tmpbuff -= g_adcmax;           // ȥ��һ�����
    g_tmpbuff -= g_adcmin;           // ȥ��һ����С
    g_temp_value = (g_tmpbuff >> 4); // ����16��ȡƽ��ֵ

// ��7361�������е���ʱ������ɼ�����adֵҪ����193���Žӽ�7351оƬ�ɼ�����adֵ
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
    // ��7361оƬ���е���ʱ���ɼ�����adֵֻ��Ҫ���� 14����ӽ�7351оƬ�ɼ�����adֵ
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

// ��ȡadc����ת�����ֵ
u16 adc_get_val_once(void)
{
    u16 g_temp_value = 0;
    ADEOC = 0; // ���ADCת����ɱ�־λ������ADת��
    while (!ADEOC)
        ;                // �ȴ�ת�����
    g_temp_value = ADRH; // ȡ��ת�����ֵ
    g_temp_value = g_temp_value << 4 | (ADRL & 0x0F);

// ��7361�������е���ʱ������ɼ�����adֵҪ����193���Žӽ�7351оƬ�ɼ�����adֵ
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
    // ��7361оƬ���е���ʱ���ɼ�����adֵֻ��Ҫ���� 14����ӽ�7351оƬ�ɼ�����adֵ
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
;  *    @Description         : ϵͳ��ʼ��
;  *    @IN_Parameter      	 :
;  *    @Return parameter    :
;  ***********************************************/
void Sys_Init(void)
{
    GIE = 0;
    CLR_RAM();
    IO_Init();
    // �ر�����ָʾ��
    LED_WORKING_OFF();
    LED_FULL_CHARGE_OFF();
    LED_CHARGING_OFF();

    timer0_pwm_config();
    timer1_pwm_config();
    timer2_pwm_config();

    timer3_config();

    key_config();
    adc_config();

    GIE = 1;
}

// ��ȡ������Ӧ�ļ�ֵ(id)
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

// ����ɨ�躯��������õ���Ӧ�İ����¼�
// ��Ҫ����10ms�Ķ�ʱ����
void key_scan(void)
{
    static volatile u8 last_key_id = KEY_ID_NONE;
    static volatile u8 press_cnt = 0;               // �������µ�ʱ�����
    static volatile u8 filter_cnt = 0;              // ����������ʹ�õı���
    static volatile u8 filter_key_id = KEY_ID_NONE; // ��������ʱʹ�õı���
    // volatile u8 cur_key_id = get_key_id();          // ��ȡ��ǰ�����ļ�ֵ(id)
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
        // ����а�������/�ɿ�
        filter_cnt = 0;
        filter_key_id = cur_key_id;
        return;
    }

    if (filter_cnt < KEY_FILTER_TIMES)
    {
        // �����⵽��ͬ�İ�������/�ɿ�
        // ��ֹ�������
        filter_cnt++;
        return;
    }

    // �˲�/������ɺ�ִ�е�����

    if (last_key_id != cur_key_id)
    {
        if (last_key_id == KEY_ID_NONE)
        {
            // ����а������£�����������µ�ʱ�����
            press_cnt = 0;
        }
        else if (cur_key_id == KEY_ID_NONE)
        {
            // ��������ɿ�
            if (press_cnt < 75)
            {
                // ����ʱ��С�� 750ms ���Ƕ̰�
                if (KEY_ID_MODE == last_key_id)
                {
                    // ����/ģʽ�����̰�
                    key_event = KEY_EVENT_MODE_PRESS;

                    // PWM0EC = 0;
                    // P16OE = 1;
                    // DEBUG_PIN = ~DEBUG_PIN;
                }
                else if (KEY_ID_HEAT == last_key_id)
                {
                    // ���Ȱ����̰�
                    key_event = KEY_EVENT_HEAT_PRESS;
                }
            }
            else
            {
                // ��������������֮������
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
        // ���������ס����
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
                // �����ǰ�豸�ǹرյ�
                if (press_cnt >= 100) // 1000ms���Ͽ��Ź����ѵ�1024ms
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

// ��ɨ�赽�İ����¼����д���
void key_event_handle(void)
{
    if (FLAG_IS_DEVICE_OPEN)
    {
        // ����豸��������
        if (KEY_EVENT_MODE_HOLD == key_event)
        {
            // �ػ���
            LED_WORKING_OFF(); // �رյ�Դָʾ��
            LED_CHARGING_OFF();
            HEATING_OFF(); // �رռ���
            FLAG_IS_DEVICE_OPEN = 0;
            FLAG_IS_HEATING = 0;

            if (flag_is_low_battery)
            {
                // ����ػ�ǰ���� �͵������� ��״̬
                flag_is_low_battery = 0;
                LED_CHARGING_OFF(); // �ر����ڱ�����LED
            }

            mode_flag = MODE_1; // ��һ���л�ģʽʱ������ MODE_2

            // �ر� ��ת�ͷ�ת��PWM
            PWM0EC = 0;
            PWM1EC = 0;
        }
        else if (KEY_EVENT_MODE_PRESS == key_event)
        {
            if (MODE_1 == mode_flag)
            {
                // ����PWM��ռ�ձ�
                T0DATA = 160;
                T1DATA = 160;

                mode_flag = MODE_2;
                flag_ctl_dir = 1;
            }
            else if (MODE_2 == mode_flag)
            {
                // ����PWM��ռ�ձ�
                T0DATA = 255;
                T1DATA = 255; // 100%ռ�ձȣ�ȷ������TxLOAD��ֵ�Ϳ��ԣ�

                mode_flag = MODE_3;
                flag_ctl_dir = 1;
            }
            else if (MODE_3 == mode_flag)
            {
                // ����PWM��ռ�ձ�
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

            turn_dir_ms_cnt = 0; // �����л�����ļ���
        }
        else if (KEY_EVENT_HEAT_PRESS == key_event)
        {
            if (0 == FLAG_IS_HEATING)
            {
                HEATING_ON(); // �򿪼���
                LED_WORKING_OFF();
                LED_CHARGING_ON();
                FLAG_IS_HEATING = 1;
            }
            else
            {
                HEATING_OFF(); // �رռ���
                LED_CHARGING_OFF();
                LED_WORKING_ON();
                FLAG_IS_HEATING = 0;
            }
        }
    }
    else
    {
        // ����豸û��������
        if (KEY_EVENT_MODE_HOLD == key_event && 0 == FLAG_IS_IN_CHARGING)
        {
            LED_FULL_CHARGE_OFF(); // �ر�����ָʾ��
            LED_CHARGING_OFF();    // �رճ��ָʾ��
            LED_WORKING_ON();      // �򿪵�Դָʾ��
            FLAG_IS_DEVICE_OPEN = 1;

            // �趨��ת����ת��PWM�ĳ�ʼռ�ձ�
            T0DATA = 150;
            T1DATA = 150;
            mode_flag = MODE_1; // ��һ���л�ģʽʱ������ MODE_2

            // // �򿪿�����ת��PWM
            // PWM0EC = 1;

            PWM1EC = 1;
        }
    }

    // ������ɺ���������¼�
    key_event = KEY_EVENT_NONE;
}

void adc_scan_handle(void)
{
    volatile u8 cnt = 0;             // ����ֵ�����ڼ�����Ƿ����磬Ҳ�����ڼ���Ƿ����/�γ�������ļ���ֵ
    volatile u8 need_charge_cnt = 0; // ����ֵ�����ڼ���Ƿ�Ҫ���ٳ��

    adc_bat_val = 0;
    adc_charging_val = 0;

    adc_sel_pin(ADC_PIN_P02_AN1); // �л�������ؽ�ѹ��ĵ�ѹ�ļ������
    for (i = 0; i < 10; i++)      //  // ÿ55ms����һ�Σ�ѭ����ÿ�μ��Լ4.8ms
    {
        adc_val = adc_get_val();

        if (i == 0)
        {
            adc_bat_val = adc_val;
        }

        if (FLAG_IS_IN_CHARGING)
        {
            // �����⵽�����磨���ܴ����˵�ر�����Ĺ��䱣������ֱ������ϵ͵�PWM
            if (adc_val >= ADCDETECT_BAT_FULL + ADCDETECT_BAT_NULL_EX)
            {
                // T2DATA = 10; // �������䱣�����һֱ�������������ά����0.03���䣬��ʱ������˵�ѹ��12-13V
                T2DATA = 0;
                over_charging_cnt++;
            }

            if (adc_val >= ADCDETECT_BAT_FULL) // ������Ƿ�����
            {
                cnt++;
            }
            else if (adc_val < ADCDETECT_BAT_WILL_FULL) // ����Ƿ�Ҫ���ٳ��
            {
                need_charge_cnt++;
            }

            if (flag_bat_is_empty)
            {
// �����⵽�γ��˵��
#if 1                               // �°�Ĺ���--Ҫ���ڳ��ʱ��û�е�ص�����£����̵ƿ���
                LED_CHARGING_OFF(); // �رճ��ָʾ��
                PWM2EC = 0;         // �رտ�����ѹ��·��pwm�������ǣ����ϳ�Ǯ��ʱ�����رտ�����ѹ��·��PWM�������ڲ���ʱ�����һ������ط��ȣ�
                T2DATA = 0;
                FLAG_BAT_IS_NEED_CHARGE = 0;
                FLAG_DURING_CHARGING_BAT_IS_NULL = 1; // ��־λ��һ������ѭ�����̵ƿ���
#endif
                break;
            }
            else if (cnt >= 8 || over_charging_cnt >= 8)
            {
                // ����ڳ��ʱ�����֮ǰ��Ҫ��磬���ڼ�⵽������
                full_charge_cnt++;
                if (full_charge_cnt >= 100 || over_charging_cnt >= 8) // 5.5s�������������������ɴι��䱣��
                {
                    full_charge_cnt = 0; //

                    PWM2EC = 0; // �رտ�����ѹ��·��pwm
                    T2DATA = 0;
                    // FLAG_IS_IN_CHARGING = 0; // ���ܸ������־λ���㣨�������ɨ�������㣩
                    FLAG_BAT_IS_NEED_CHARGE = 0;
                    FLAG_BAT_IS_FULL = 1;

                    LED_CHARGING_OFF(); // �رճ��ָʾ��
                    // LED_WORKING_ON();   // ��������ָʾ�ƣ����Ƴ�����
                    // LED_FULL_CHARGE_ON(); // ������س������ָʾ��
                    LED_WORKING_ON(); // �򿪵�Դָʾ�ƣ���ʾ������
                    break;
                }
            }
            else if (need_charge_cnt >= 8)
            {
                // ����ڳ�磬�ҵ����Ҫ��磨��ص���С�� ADCDETECT_BAT_WILL_FULL����ؽ�Ҫ����ĵ�ѹ��
                // ���ݵ�ǰ�ĵ�ص�ѹ������Ҫ�������PWM����ص�ѹԽС��PWMռ�ձ�ҲҪС����������������
                LED_FULL_CHARGE_OFF(); // �رճ������ָʾ��
                FLAG_BAT_IS_NEED_CHARGE = 1;
                FLAG_BAT_IS_FULL = 0;
                break;
            }
        } // if (FLAG_IS_IN_CHARGING)
        else // ���δ�ڳ�磬������Ƿ���Ҫ���
        {
            // if (adc_val <= ADCDETECT_BAT_FULL - 130)
            if (flag_bat_is_empty == 0 && adc_val <= ADCDETECT_BAT_FULL - ((ADCDETECT_BAT_FULL - ADCDETECT_BAT_WILL_FULL) / 2))
            {
                cnt++;
            }

            if (cnt >= 8)
            {
                // ���δ�ڳ�磬�ҵ����Ҫ��磨��ص���С�� ����-�������� ��
                FLAG_BAT_IS_NEED_CHARGE = 1;
                FLAG_BAT_IS_FULL = 0;
                break;
            }

            if (                           /* ������Ƿ��ڹػ���Ӧ�ĵ�ѹ */
                flag_bat_is_empty == 0 &&  /* ��ز�Ϊ�� */
                FLAG_IS_DEVICE_OPEN &&     /* �豸�������� */
                adc_val < SHUT_DOWN_AD_VAL /* �ɼ�����adֵС�ڹػ���Ӧ��adֵ */
            )
            {
                flag_tim_scan_maybe_shut_down = 1;
            }
            else if (flag_bat_is_empty == 0 &&
                     FLAG_IS_DEVICE_OPEN &&
                     adc_val < LOW_BATTERY_AD_VAL)
            {
                // ������Ƿ��ڵ͵�����
                flag_maybe_low_battery = 1;
                flag_tim_scan_maybe_shut_down = 0;
            }
            else
            {
                flag_maybe_low_battery = 0;
                flag_tim_scan_maybe_shut_down = 0;
            }

            // �����ĵ��˹ػ���Ӧ�ĵ�ѹ
            if (flag_is_needed_shut_down && FLAG_IS_DEVICE_OPEN)
            {
                flag_is_needed_shut_down = 0;
                // �ػ���
                LED_WORKING_OFF(); // �رյ�Դָʾ��
                LED_CHARGING_OFF();
                HEATING_OFF(); // �رռ���
                FLAG_IS_DEVICE_OPEN = 0;
                FLAG_IS_HEATING = 0;

                if (flag_is_low_battery)
                {
                    // ����ػ�ǰ���ڵ͵���������״̬
                    flag_is_low_battery = 0;
                    LED_CHARGING_OFF(); // �ر����ڱ�����LED
                }

                mode_flag = MODE_1; // ��һ���л�ģʽʱ������ MODE_2

                // �ر� ��ת�ͷ�ת��PWM
                PWM0EC = 0;
                PWM1EC = 0;
            }
        }

        // key_scan_handle(); // ����ɨ��ʹ�����
        key_event_handle();
    } // for (i = 0; i < 10; i++)

    cnt = 0;
    adc_sel_pin(ADC_PIN_P00_AN0); // �л��������ĵ�ѹ�������(��⵽�ĳ���ѹ == USB-C�ڵ�ѹ / 2)
    for (i = 0; i < 10; i++)
    {
        adc_val = adc_get_val();
        if (flag_bat_is_empty) // ������Ϊ��
        {
            adc_val = 4095; // ��ֹ���������if (adc_val < ADCDETECT_CHARING_THRESHOLD)
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
            // ������ڳ�磬����Ƿ�γ��˳����
            if (adc_val < ADCDETECT_CHARING_THRESHOLD)
            {
                cnt++;
            }

            if (cnt >= 8)
            {
                // ����ڳ��ʱ����⵽�γ��˳����
                FLAG_IS_IN_CHARGING = 0;
                LED_CHARGING_OFF(); // �رճ��ָʾ��
                PWM2EC = 0;         // �رտ�����ѹ��·��pwm
                T2DATA = 0;
                LED_FULL_CHARGE_OFF(); // �رյ�س������ָʾ��
                LED_WORKING_OFF();     // �رյ�س������ָʾ�ƣ��׵ƣ�

                FLAG_DURING_CHARGING_BAT_IS_NULL = 0; // ��ոñ�־λ����Ϊ�Ѿ����ڳ��������
                break;
            } // if (cnt >= 8)

            if (FLAG_IS_DEVICE_OPEN)
            {
            }
            else // ����豸δ����
            {
                if (FLAG_BAT_IS_FULL)
                {
                }
                else
                {
                    // ���δ����
                    LED_FULL_CHARGE_OFF();
                    LED_WORKING_OFF();
                    // LED_CHARGING_ON();
                }
            }
        } // if (FLAG_IS_IN_CHARGING)
        else
        {
            // ������ڳ�磬����Ƿ�����˳����
            if (flag_bat_is_empty == 0 && adc_val >= ADCDETECT_CHARING_THRESHOLD)
            {
                cnt++;
            }

            if (cnt >= 8)
            {
                // ȷ���ǲ������ߺ����۴���ʲô״̬������Ϊ�ػ�״̬
                full_charge_cnt = 0;
                over_charging_cnt = 0;

                FLAG_IS_IN_CHARGING = 1;
                // LED_CHARGING_ON(); // �������ָʾ��
                // T2DATA = 0;
                PWM2EC = 1; // ����������ѹ��·��pwm
                LED_FULL_CHARGE_OFF();
                LED_WORKING_OFF(); // �رյ�Դָʾ��
                HEATING_OFF();     // �رռ���
                FLAG_IS_DEVICE_OPEN = 0;
                FLAG_IS_HEATING = 0;
                mode_flag = MODE_1; // ��һ���л�ģʽʱ������ MODE_2
                // �ر� ��ת�ͷ�ת��PWM
                PWM0EC = 0;
                PWM1EC = 0;

                flag_is_low_battery = 0; // ����ñ�־λ
                break;
            } // if (cnt >= 8)
            // if (cnt >= 8)
            // {
            //     // ȷ���ǲ������ߺ�
            //     full_charge_cnt = 0;
            //     over_charging_cnt = 0;

            //     if (FLAG_IS_DEVICE_OPEN)
            //     {
            //         LED_WORKING_ON(); // �򿪹���ָʾ��
            //     }
            //     else
            //     {
            //         // �豸������ʱ���Ŵ򿪸�ָʾ��
            //         LED_CHARGING_ON(); // �������ָʾ��
            //     }

            //     PWM2EC = 1; // ����������ѹ��·��pwm
            //     FLAG_IS_IN_CHARGING = 1;
            //     break;
            // } // if (cnt >= 8)
        }

        // key_scan_handle(); // ����ɨ��ʹ�����
        key_event_handle();
    } // for (i = 0; i < 10; i++)
}

void turn_dir_scan_handle(void)
{
    if (FLAG_IS_DEVICE_OPEN)
    {
        // �豸����ʱ���ſ�ʼ��ʱ���ж��Ƿ�Ҫת��
        // turn_dir_ms_cnt += ONE_CYCLE_TIME_MS;
        if (turn_dir_ms_cnt >= (120000) ||
            flag_ctl_dir)
        {
            if (0 == FLAG_DIR)
            {
                PWM1EC = 0;
                delay_ms(1000);
                PWM0EC = 1;
                FLAG_DIR = 1; //
            }
            else
            {
                PWM0EC = 0; //
                delay_ms(1000);
                PWM1EC = 1;   //
                FLAG_DIR = 0; //
            }

            flag_ctl_dir = 0;
            turn_dir_ms_cnt = 0;
        }
    }
    else
    {
        turn_dir_ms_cnt = 0; // �豸δ����ʱ����ռ���ֵ
    }
}

// �ػ����ʹ���
void shutdown_scan_handle(void)
{
    if (FLAG_IS_DEVICE_OPEN)
    {
        // ����豸��������ʼ��ʱ��15min���Զ��ػ�
        // shut_down_ms_cnt += ONE_CYCLE_TIME_MS;
        if (shut_down_ms_cnt >= 900000)
        {
            // ���������15min���ػ���
            LED_WORKING_OFF(); // �رյ�Դָʾ��
            LED_CHARGING_OFF();
            HEATING_OFF(); // �رռ���
            FLAG_IS_HEATING = 0;
            mode_flag = MODE_4; // ��һ���л�ģʽʱ������ MODE_1
            // �ر� ��ת�ͷ�ת��PWM
            PWM0EC = 0;
            PWM1EC = 0;

            FLAG_IS_DEVICE_OPEN = 0;
        }
    }
    else
    {
        // ����豸δ��������ռ�ʱ
        shut_down_ms_cnt = 0;
    }
}

void low_power_scan_handle(void)
{
    // if (FLAG_DURING_CHARGING_BAT_IS_NULL)
    if (FLAG_DURING_CHARGING_BAT_IS_NULL || // ֻ���ų������û�е��ʱ��������͹���
        FLAG_IS_DEVICE_OPEN ||              // ����豸�Ѿ�������������͹���
        FLAG_IS_IN_CHARGING ||              // ������ڳ�磬������͹��ģ���Ϊ����Ҫ���PWM�����Ƴ��
        (0 == P01D))                        // �����⵽ ����/ģʽ �������£�������͹���(���������¼����������ж��Ƿ�Ҫ����)
    {
        return;
    }

    // if (FLAG_IS_DEVICE_OPEN)
    // {
    //     // ����豸�Ѿ�������������͹���
    //     return;
    // }

    // // ������е����˵���豸û������
    // // ������Ҫ�������ڳ������
    // if (FLAG_IS_IN_CHARGING)
    // {
    //     // ������ڳ�磬������͹��ģ���Ϊ����Ҫ���PWM�����Ƴ��
    //     return;
    // }

label:
    GIE = 0;  // ���������ж�
    T0EN = 0; // �رն�ʱ����PWM���
    T1EN = 0;
    T2EN = 0;
    T3EN = 0;

    PWM0EC = 0;
    PWM1EC = 0;
    PWM2EC = 0;
    T2DATA = 0;
    ADEN = 0; // ��ʹ��ad

    // LED������Ϊ����ģʽ�����ⲿ�������൱�ڸ���̬����
    // P14OE = 0;
    // P04OE = 0;
    // P03OE = 0;

    LED_WORKING_OFF();
    LED_CHARGING_OFF();
    LED_FULL_CHARGE_OFF();

    // ����/ģʽ���������ã�����Ϊ��������
    P01PU = 1;
    P01OE = 0;

    // ���ó��������
    P00PD = 1;
    P00OE = 0;

    HFEN = 0; // �رո���ʱ��
    LFEN = 1;

    // ����ǰ�ر�����
    Nop();
    Nop();
    Stop();
    Nop();
    Nop();

    HFEN = 1; // ��������ʱ��
    // ���Ѻ����³�ʼ��
    CLR_RAM();

    adc_config();
    adc_sel_pin(ADC_PIN_P00_AN0);
    adc_val = adc_get_val_once();
    // ������¿�������/������������������������
    if (adc_val < ADCDETECT_CHARING_THRESHOLD && P01D)
    {
        // ���û�а��¿���������û�в�������
        // �ر�ADC�������������ߣ�
        // ADEN = 0; //
        goto label;
    }

    // P02PU = 0; // �ر�������
    // P02PD = 0;
    // P02OE = 0; // ����ģʽ
    // P02DC = 1; // ģ������
    // adc_sel_pin(ADC_PIN_P02_AN1);
    // adc_val = adc_get_val();
    // if (adc_val < ADCVAL_REF_BAT_6_0_V)
    // {
    //     // �����ص������ͣ������������ǿ��Գ��
    //     FLAG_IS_NOT_OPEN_DEVICE = 1; // �������������ǿ��Գ��
    // }
    // else
    // {
    //     FLAG_IS_NOT_OPEN_DEVICE = 0;
    // }

    // FLAG_IS_NOT_OPEN_DEVICE = 0;

    Sys_Init();
    GIE = 1;

    delay_ms(1); // �ȴ�ϵͳ�ȶ�
}

void main(void)
{
    Sys_Init();

    delay_ms(1); // �ȴ�ϵͳ�ȶ�

    // flag_bat_is_empty = 0; // �������Ż������ϵ�Ĭ�Ͼ���0��

    // �ϵ�ʱ������Ƿ���ȷ��װ:
    PWM2EC = 1; // �򿪿�����ѹ��·��pwm
    T2DATA = 100;
    adc_sel_pin(ADC_PIN_P02_AN1); // �л�������ؽ�ѹ��ĵ�ѹ�ļ������
    for (i = 0; i < 10; i++)      // ÿ55ms����һ�Σ�ѭ����ÿ�μ��Լ4.8ms
    {
        adc_val = adc_get_val();
        if (adc_val >= ADCDETECT_BAT_FULL + ADCDETECT_BAT_NULL_EX)
        {
            flag_bat_is_empty = 1;
            FLAG_DURING_CHARGING_BAT_IS_NULL = 1; // ��־λ��һ������ѭ�����̵ƿ���
        }
    }
    PWM2EC = 0; // �رտ�����ѹ��·��pwm
    T2DATA = 0;

    while (1)
    {
#if 1
        // DEBUG_PIN = ~DEBUG_PIN; // ������ѭ���Ƿ�����

        // P10D = 1; // ����һ��ѭ�������ʱ��
        // key_scan_handle(); // ����ɨ��ʹ�����
        key_event_handle(); // �����¼���������ɨ�����жϷ�������ִ��
        adc_scan_handle();  // ���ɨ��ʹ�����(Լռ��89ms)

        turn_dir_scan_handle();
        shutdown_scan_handle(); // �Զ��ػ����ʹ�����

        // ���ݵ�ص�ѹ�ͳ��ڵĵ�ѹ�����ڿ��Ƴ���PWMռ�ձ�
        // |-- �ڳ��ʱ������Ƿ�Ϊ�գ����Ϊ�գ����̵ƿ���
        if (FLAG_IS_IN_CHARGING)
        {
            last_pwm_val = T2DATA;      // ������һ��PWMռ�ձȶ�Ӧ��ֵ
            max_pwm_val = (T2LOAD + 1); // ����PWMռ�ձ��趨�ġ�����ֵ

            /*
                �޸ĵ�ѹ��ֵ����ѹ��ֵ = 203 - (adc_bat_val * 122 / 1000)

                �Ƶ����̣�
                �ڳ��ʱ��ã�������1.1A���ң�ѹ��Ϊ-30(adֵ)ʱ�����һ���ѹΪ7.8V(adֵ��1917)
                             ������1.1A���ң�ѹ��Ϊ0(adֵ)ʱ�����һ���ѹΪ6.8V(adֵ��1671)
                ����x��Ϊ��ѹ��Ӧ��adֵ��y��Ϊѹ���Ӧ��adֵ�������ο�����ϵ
                �������������Ե㣬����x������������y�Ḻ��������������б�����£�б��Ϊ�������б��
                    k = ��y/��x = (0 - 30) / (1917 - 1671)��ԼΪ -0.122
                ������ʽ��y = kx + b�����룬��� b ԼΪ 203 ������������204��
                y = kx + b ==> ѹ�� = -0.122 * ���ʱ�ĵ�ص�ѹ + 203
                ת���ɵ�Ƭ�����Լ������ʽ��ѹ�� = 203 - (���ʱ�ĵ�ص�ѹ * 122 / 1000)
            */

#if 1
            // if (adc_bat_val < 2619) // ����ڳ��ʱ��⵽��ص�ѹС��6.0V
            // {
            //     // tmp_bat_val = (u32)adc_bat_val + (294 - (u32)adc_bat_val * 157 / 1000);
            //     // tmp_bat_val = adc_bat_val  + (522 - adc_bat_val * 157 / 1000);
            //     tmp_bat_val = (adc_bat_val + 37);
            // }
            // else if (adc_bat_val <= 2837) // �������ص�ѹС�� 6.5V
            if (adc_bat_val <= 2837) // �������ص�ѹС�� 6.5V
            {
                tmp_bat_val = (adc_bat_val + 37);
            }
            else if (adc_bat_val <= 3056) // �������ص�ѹС�� 7.0V
            {
                // tmp_bat_val = (adc_bat_val + 18);
                tmp_bat_val = (adc_bat_val + 27);
            }
            else if (adc_bat_val <= 3188) // �������ص�ѹС�� 7.3V
            {
                // tmp_bat_val = (adc_bat_val + 9);
                tmp_bat_val = (adc_bat_val + 16);
            }
            else if (adc_bat_val <= 3326) // �������ص�ѹС�� 7.62V
            {
                // tmp_bat_val = (adc_bat_val + 0);
                tmp_bat_val = (adc_bat_val + 0);
                // tmp_bat_val = (u32)adc_bat_val + (294 - (u32)adc_bat_val * 157 / 1000);
            }
            else // ����ڳ��ʱ��⵽��ص�ѹ����
            {
                // tmp_bat_val = (u32)adc_bat_val - ((u32)adc_bat_val * 157 / 1000 - 260); // ʵ�ʵĳ�������С�� 0.75-0.85
                // tmp_bat_val = (u32)adc_bat_val - ((u32)adc_bat_val * 157 / 1000 - 304); // 1.3-1.5
                // tmp_bat_val = (u32)adc_bat_val - ((u32)adc_bat_val * 157 / 1000 - 284); // 1.1A-1.2A (���û��ʱ��0.9-1.2���������8Vʱ��0.99��1.08����)
                // tmp_bat_val = (u32)adc_bat_val - ((u32)adc_bat_val * 157 / 1000 - 274); // 0.3(�տ�ʼ�������Ӻ������0.9)-0.9

                // �������صķ�ѹ������ 22K / 100K��1.2-1.3A,�������1.22A��1.26A
                // �������صķ�ѹ������ 220K / 1M����������0.9A-1A
                // tmp_bat_val = (u32)adc_bat_val - ((u32)adc_bat_val * 157 / 1000 - 294);
                tmp_bat_val = (u32)adc_bat_val - ((u32)adc_bat_val * 157 / 1000 - 522);
            }

            // tmp_bat_val += 30;
            // tmp_bat_val += 32;
            // tmp_bat_val += 52; //
            // tmp_bat_val += 33; // 1.3A(7361оƬ)

            // ��7351оƬ,������ 0.89-0.90A
            // 7361,0.83-0.84A
            // tmp_bat_val += 27; //

            // 7361,0.84-0.85A
            // tmp_bat_val += 30;

            // 7361,0.9A
            // tmp_bat_val += 33;

            // 7361,0.95A
            // tmp_bat_val += 40;

            // 7361,0.98A-1.0A
            // tmp_bat_val += 55;

            // tmp_bat_val += 60;

            // tmp_bat_val += 75; // 1.16-1.17
            // tmp_bat_val += 80; //  1.21

            tmp_bat_val += 95; // 1.18A(���8V��ʹ���ⲿ�ĵ��)
                               // tmp_bat_val += 105; // 1.32-1.39A

#endif

            // for (i = 0; i < ARRAY_SIZE(bat_val_fix_table); i++)
            // {
            //     if (adc_bat_val <= bat_val_fix_table[i].adc_bat_val)
            //     {
            //         tmp_bat_val = (adc_bat_val + bat_val_fix_table[i].tmp_bat_val_fix);
            //         break;
            //     }

            //     if (i == (ARRAY_SIZE(bat_val_fix_table) - 1))
            //     {
            //         tmp_bat_val = (u32)adc_bat_val - ((u32)adc_bat_val * 157 / 1000 - 522) + TMP_BAT_VAL_FIX;
            //     }
            // }

            /*
                ��ѹ��ʽ��Vo = Vi / (1 - D)

                ͨ��PWM��������ѹ��������赱ǰPWMռ�ձȼĴ�����ֵ Ϊ D��PWMռ�ձȼĴ������Ե�������ֵ Ϊ 1
                Vo = Vi / (PWMռ�ձȼĴ������Ե�������ֵ - ��ǰPWMռ�ձȼĴ�����ֵ)
                ��ǰPWMռ�ձ�Խ��VoҲԽ�󣬳��ĵ���Ҳ��Խ��

                (PWMռ�ձȼĴ������Ե�������ֵ - ��ǰPWMռ�ձȼĴ�����ֵ) = Vi / Vo
                ��ǰPWMռ�ձȼĴ�����ֵ = PWMռ�ձȼĴ������Ե�������ֵ - Vi / Vo

                �����⵽�ĳ���ѹ��adֵ == USB-C�ڵ�ѹ / 2[�����������ѹ] / �ο���ѹ[3V����ô������ǳ���3] * 4096[adת�����ȣ�12λ-->0~4096]
                ���������⵽�ĳ���ѹ��adֵ == USB-C�ڵ�ѹ / 2 / 3 * 4096
                ��⵽�ĵ�ص�ѹ��adֵ == ��ص�ѹ * 0.18 / 3V�ο���ѹ * 4096 == ��ص�ѹ * 220 / 1220 / 3V�ο���ѹ * 4096
                (��صķ�ѹ���裺 ����220K������1M����ѹϵ���� 220 / 1220)

                ������ѹ�ͼ���ص�ѹʹ�õĲ���ͬһ����ѹϵ����Ҫһ������ʱ�����ｫ����ѹ��ad�� * 2 * 220 / 1220
                �� (adc_charging_val * 22 / 61)

                �ٴ��ع�ʽ����ǰPWMռ�ձȼĴ�����ֵ = PWMռ�ձȼĴ������Ե�������ֵ - Vi / Vo
                ��ǰPWMռ�ձȼĴ�����ֵ = PWMռ�ձȼĴ������Ե�������ֵ - ����ѹ / ���ʱ�������ĵ�ѹ
                tmp_val = max_pwm_val - ����ѹ / ���ʱ�������ĵ�ѹ
                ת���ɵ�Ƭ�����Լ������ʽ��
                tmp_val = max_pwm_val - (adc_charging_val * 22 / 61) / tmp_bat_val������ max_pwm_val ��ֵ����1�������� Vo = Vi / (1 - D)
                ����Ҫ�ĳ� tmp_val = max_pwm_val - max_pwm_val * (adc_charging_val * 22 / 61) / tmp_bat_val
                tmp_val = max_pwm_val - (adc_charging_val * max_pwm_val * 22 / 61) / tmp_bat_val
            */
            // D = 1 - (Vi / Vo)
            // tmp_val = max_pwm_val - (adc_charging_val * max_pwm_val * 22 / 61) / tmp_bat_val;
            tmp_val = max_pwm_val - (adc_charging_val * max_pwm_val * 94 / 147) / tmp_bat_val;

            if (tmp_val >= max_pwm_val)
            {
                // ���PWMռ�ձȶ�Ӧ��ֵ ���� ���ռ�ձȶ�Ӧ��ֵ��˵����������������ǵ�ص�ѹ��С������0����
                tmp_val = 0;
            }

            // �˲�������һ��ʼtmp_val���С���ɼ���κ�����һ��ƽ��ֵ��
            tmp_val_cnt++;
            tmp_val_cnt &= 0x07;
            tmp_val_l[tmp_val_cnt] = (tmp_val_l[tmp_val_cnt] + tmp_val) >> 1;
            tmp_val = 0;
            for (i = 0; i < 8; i++)
            {
                tmp_val += tmp_val_l[i];
            }
            tmp_val >>= 3;

            if (tmp_val > last_pwm_val)
            {
                last_pwm_val = last_pwm_val + 1;
            }
            else if (tmp_val < last_pwm_val)
            {
                last_pwm_val = last_pwm_val - 1;
            }

            T2DATA = last_pwm_val;
        } // if (FLAG_IS_IN_CHARGING)
        else // ���δ�ڳ��
        {
            if (FLAG_DURING_CHARGING_BAT_IS_NULL)
            {
                // ������δ��װ
                LED_FULL_CHARGE_ON();
                delay_ms(200);
                LED_FULL_CHARGE_OFF();
                delay_ms(200);
            }
        } // else // ���δ�ڳ��

        if (flag_is_low_battery)
        {
            LED_WORKING_OFF();

            // ����ʱ��⵽�͵���
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

        // P10D = 0; // ����һ��ѭ�������ʱ��

#endif
        __asm;
        CLRWDT; // ι��ָ��
        __endasm;

    } // while (1)
}

/************************************************
;  *    @������            : interrupt
;  *    @˵��              : �жϺ���
;  *    @�������          :
;  *    @���ز���          :
;  ***********************************************/
void int_isr(void) __interrupt
{
    __asm;
    movra _abuf;
    swapar _PFLAG;
    movra _statusbuf;
    __endasm;
    //=======�ⲿ�ж�0(�ɿ��ذ������������ж�ֻ���ڻ��ѵ�Ƭ��)=================
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
                // Ŀǰÿ1ms����һ���ж�
                { // ����ɨ��
                    static u8 key_scan_cnt = 0;
                    key_scan_cnt++;
                    if (key_scan_cnt >= 10)
                    {
                        key_scan_cnt = 0;
                        key_scan();
                    }
                } // ����ɨ��

                if (FLAG_IS_DEVICE_OPEN)
                {
                    turn_dir_ms_cnt++;  // ���� turn_dir_scan_handle() �ڴ�������
                    shut_down_ms_cnt++; // ���� shutdown_scan_handle() �ڴ�������
                }

                { // �͵������
                    static u16 low_bat_cnt = 0;
                    // static u16 cancel_low_bat_alarm_cnt = 0; // ȡ���͵���������ʱ�����

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

                        // if (flag_is_low_battery) // ���֮ǰ���ڵ͵�������
                        // {
                            // cancel_low_bat_alarm_cnt++;
                        //     if (cancel_low_bat_alarm_cnt >= 2000) // ���� xx ms��⵽��ص���������ȡ���͵�������
                        //     {
                        //         cancel_low_bat_alarm_cnt = 0;
                        //     }
                        // }
                        // else
                        // {
                        //     cancel_low_bat_alarm_cnt = 0;
                        // }
                    }
                } // �͵������

                { // �ػ��������
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
                } // �ػ��������
            }
        }

        // �����ƿ��ƣ�
        if (FLAG_IS_IN_CHARGING && 0 == FLAG_BAT_IS_FULL)
        {
            // PWM����
            if (pwm_counter < pwm_duty)
            {
                led_state = 1; // LED��
            }
            else
            {
                led_state = 0; // LED��
            }

            // PWM���ڼ�����20��PWM�����Լ�����19����㣩
            pwm_counter++;
            if (pwm_counter >= PWM_MAX_LEVEL)
            {
                pwm_counter = 0;
            }

            // ����Ч�����ƣ�ÿ20ms�ı�һ�����ȣ�
            breath_counter++;
            if (breath_counter >= BREATH_PERIOD)
            {
                breath_counter = 0;

                // ���ݷ������PWMռ�ձ�
                if (breath_direction == 0)
                {
                    // ����
                    pwm_duty++;
                    if (pwm_duty >= PWM_MAX_LEVEL)
                    {
                        pwm_duty = PWM_MAX_LEVEL;
                        breath_direction = 1; // �л�������
                    }
                }
                else
                {
                    // ����
                    if (pwm_duty > 0)
                    {
                        pwm_duty--;
                    }
                    else
                    {
                        pwm_duty = 0;
                        breath_direction = 0; // �л�������
                    }
                }
            }

            // ���LED״̬
            // LED_SET(led_state);  // �������ǿ���LED�ĺ���
            if (led_state)
            {
                LED_CHARGING_PIN = LED_ON;
            }
            else
            {
                LED_CHARGING_PIN = LED_OFF;
            }
        } // �����ƿ��� if (FLAG_IS_IN_CHARGING && 0 == FLAG_BAT_IS_FULL)

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
