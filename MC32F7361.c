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
    IOP0 = 0x00;   // io������λ
    OEP0 = 0x3F;   // io�ڷ��� 1:out  0:in
    PUP0 = 0x00;   // io����������   1:enable  0:disable
    PDP0 = 0x00;   // io����������   1:enable  0:disable
    P0ADCR = 0x00; // io����ѡ��  1:ģ������  0:ͨ��io

    IOP1 = 0x00;   // io������λ
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
    T0CR |= 0x02; // 4��Ƶ
    T0LOAD = 128 - 1;
    T0DATA = 25; // ռ�ձ� == T0DATA / T0LOAD
    PWM0EC = 0;  // ��ֹPWM���
    T0EN = 1;    // ������ʱ��
}

// ��ʱ��1pwm���ã�������� P17
// ���Ʒ����pwm
void timer1_pwm_config(void)
{
    // ====================================================
    // �ӽ�15.645KHz�汾��ǰ��������FCPU = FOSC / 4
    T1CR |= 0x02; // 4��Ƶ
    T1LOAD = 128 - 1;
    T1DATA = 25; // ռ�ձ� == T1DATA / T1LOAD
    PWM1EC = 0;  // ��ֹPWM1���
    T1EN = 1;    // ������ʱ��
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
// void timer3_config(void)
// {
//     T3LOAD = 250 - 1; // FCPU 32��Ƶ��������1ms����һ���ж�
//     // T3EN = 1;
//     T3CR = 0x85; // ʹ�ܶ�ʱ����ʱ��Դѡ��FCPU��32��Ƶ
//     T3IE = 1;
// }

// ����������ŵ����ã�
void key_config(void)
{
    // �����Ƿ���ȵİ���������
#if USE_MY_DEBUG
    // ����P13�޷�������棬����ʹ��P05
    P05PU = 1;
    P05OE = 0;
#else
    P13PU = 1; // ����
    P13OE = 0; // ����ģʽ
#endif
    // ����/�ػ����������ã�һ��Ҫ�жϴ�����
    INT0M0 = 1; // INT0M0��INT0M1���������INT0Ϊ�½����ж�
    INT0M1 = 0;
    P11PU = 1;
    P11OE = 0;
    INT0IF = 0; // ����жϱ�־
    INT0IE = 1; // �ⲿ�ж�ʹ��

    // �л�ģʽ����������
    P01PU = 1;
    P01OE = 0;
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
        break;

    case ADC_PIN_P02_AN1:
        ADCR0 &= ~(0x0F << 4); // ��ռĴ�����ͨ��ѡ��λ
        ADCR0 |= 0x01 << 4;    // AIN1--P02;
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

    timer0_pwm_config();
    timer1_pwm_config();
    timer2_pwm_config();

    // timer3_config();

    key_config();
    adc_config();
    delay_ms(100); // �ȴ�ϵͳ�ȶ�

    GIE = 1;
}

// ����ɨ�躯��
void key_scan_handle(void)
{
    if (0 == KEY_MODE_PIN)
    {
        // ��־λ������ ����/ģʽ �����ĳ���������
        // 0--����δ����,1--�����Ѿ�����
        static volatile u8 flag_is_key_mode_hold = 1;
        u16 press_cnt = 0;

        delay_ms(20); // ��ʱ����

        if (0 == KEY_MODE_PIN)
        {
            // ���ȷ����������
            while (0 == KEY_MODE_PIN)
            {
                // �ȴ���������
                if (press_cnt < 65535)
                    press_cnt++;

                if (press_cnt >= 2000)
                {
                    // ���������2s
                    // ���֮ǰ��⵽û�а��¸ð������Ž��룺
                    if (0 == flag_is_key_mode_hold)
                    {
                        flag_is_key_mode_hold = 1; // ��ʾ�������º�δ����
  
                        if (0 == FLAG_IS_DEVICE_OPEN &&
                            // 0 == FLAG_IS_IN_CHARGING &&
                            0 == FLAG_IS_NOT_OPEN_DEVICE)
                        {
                            // ���δ������
                            // ��ص�ѹ���� xxV ���������ػ�->����
                            LED_WORKING_ON(); // �򿪵�Դָʾ��
                            // HEATING_ON();     // �򿪼���
                            FLAG_IS_DEVICE_OPEN = 1;
                            // FLAG_IS_HEATING = 1;

                            // �趨��ת����ת��PWM�ĳ�ʼռ�ձ�
                            T0DATA = 115;
                            T1DATA = 115; // ԼΪ 89.9%
                            // T0DATA = 103;
                            // T1DATA = 103;       // ԼΪ 80.5%
                            mode_flag = MODE_1; // ��һ���л�ģʽʱ������ MODE_2

                            // �򿪿�����ת��PWM
                            PWM0EC = 1;
                        }
                        else
                        {
                            // ����->�ػ�
                            LED_WORKING_OFF(); // �رյ�Դָʾ��
                            HEATING_OFF();     // �رռ���
                            FLAG_IS_DEVICE_OPEN = 0;
                            FLAG_IS_HEATING = 0;

                            mode_flag = MODE_1; // ��һ���л�ģʽʱ������ MODE_2

                            // �ر� ��ת�ͷ�ת��PWM
                            PWM0EC = 0;
                            PWM1EC = 0;
                        }
                    }
                }

                delay_ms(1);
            }

            flag_is_key_mode_hold = 0; // ��ʾ�ð����Ѿ�����

            if (press_cnt < 750)
            {
                // ����Ƕ̰�
                if (FLAG_IS_DEVICE_OPEN)
                {
                    // ��ʼΪ 89.9%ռ�ձȣ�ÿ��һ�� �� 89.9%->100%->80.5%->89.9%->...�����仯
                    if (FLAG_IS_DEVICE_OPEN)
                    {
                        // ������������л�ģʽ
                        if (MODE_1 == mode_flag)
                        {
                            // ����PWM��ռ�ձ�
                            T0DATA = 255;
                            T1DATA = 255; // 100%ռ�ձȣ�ȷ������TxLOAD��ֵ�Ϳ��ԣ�

                            mode_flag = MODE_2;
                        }
                        else if (MODE_2 == mode_flag)
                        {
                            // ����PWM��ռ�ձ�
                            T0DATA = 103;
                            T1DATA = 103; // 80.5%ռ�ձ�

                            mode_flag = MODE_3;
                        }
                        else if (MODE_3 == mode_flag)
                        {
                            // ����PWM��ռ�ձ�
                            T0DATA = 115;
                            T1DATA = 115; // 89.9%ռ�ձ�
                            mode_flag = MODE_1;
                        }
                    }
                }
            }
        }
    }
    else if (0 == KEY_HEAT_PIN)
    {
        u16 press_cnt = 0;

        delay_ms(20); // ��ʱ����
        if (0 == KEY_HEAT_PIN)
        {
            //  ���ȷ���ǰ�������
            while (0 == KEY_HEAT_PIN)
            {
                // �ȴ���������
                if (press_cnt < 65535)
                    press_cnt++;
                delay_ms(1);
            }

            if (press_cnt < 750)
            {
                // ����Ƕ̰�
                if (FLAG_IS_DEVICE_OPEN)
                {  
                    // ����豸�Ѿ����ڹ���״̬���ſ��Դ򿪼���
                    if (0 == FLAG_IS_HEATING)
                    {
                        HEATING_ON(); // �򿪼���
                        FLAG_IS_HEATING = 1;
                    }
                    else
                    {
                        HEATING_OFF(); // �رռ���
                        FLAG_IS_HEATING = 0;
                    }
                }
            }
        }
    }
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
            if (adc_bat_val != 0)
            {
                adc_bat_val += adc_val;
                adc_bat_val /= 2;
            }
            else
            {
                adc_bat_val += adc_val;
            }
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
                    // LED_FULL_CHARGE_ON(); // ������س������ָʾ��
                    LED_CHARGING_OFF(); // �رճ��ָʾ��
                    LED_WORKING_ON();   // ��������ָʾ�ƣ����Ƴ�����
                    PWM2EC = 0;         // �رտ�����ѹ��·��pwm
                    T2DATA = 0;
                    // FLAG_IS_IN_CHARGING = 0; // ���ܸ������־λ���㣨�������ɨ�������㣩
                    FLAG_BAT_IS_NEED_CHARGE = 0;
                    FLAG_BAT_IS_FULL = 1;
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
        }
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
                LED_CHARGING_OFF(); // �رճ��ָʾ��
                PWM2EC = 0;         // �رտ�����ѹ��·��pwm
                T2DATA = 0;
                LED_FULL_CHARGE_OFF(); // �رյ�س������ָʾ��
                LED_WORKING_OFF();     // �رչ���״ָ̬ʾ�ƣ��ر����ƣ� =====================================
                FLAG_IS_IN_CHARGING = 0;
                FLAG_DURING_CHARGING_BAT_IS_NULL = 0; // ��ոñ�־λ����Ϊ�Ѿ����ڳ��������
                break;
            } // if (cnt >= 8)
        } // if (FLAG_IS_IN_CHARGING)
        else
        {
            // ������ڳ�磬����Ƿ�����˳����
            if (flag_bat_is_empty == 0 && adc_val >= ADCDETECT_CHARING_THRESHOLD)
            {
                cnt++;
            }

            // if (cnt >= 8)
            // {
            //     // ȷ���ǲ������ߺ����۴���ʲô״̬������Ϊ�ػ�״̬
            //     full_charge_cnt = 0;
            //     over_charging_cnt = 0;

            //     LED_CHARGING_ON(); // �������ָʾ��
            //     // T2DATA = 0;
            //     PWM2EC = 1;        // ����������ѹ��·��pwm
            //     LED_WORKING_OFF(); // �رյ�Դָʾ��
            //     HEATING_OFF();     // �رռ���
            //     FLAG_IS_DEVICE_OPEN = 0;
            //     FLAG_IS_HEATING = 0;
            //     mode_flag = MODE_1; // ��һ���л�ģʽʱ������ MODE_2
            //     // �ر� ��ת�ͷ�ת��PWM
            //     PWM0EC = 0;
            //     PWM1EC = 0;

            //     FLAG_IS_IN_CHARGING = 1;
            //     break;
            // } // if (cnt >= 8)
            if (cnt >= 8)
            {
                // ȷ���ǲ������ߺ�
                full_charge_cnt = 0;
                over_charging_cnt = 0;

                LED_CHARGING_ON(); // �������ָʾ��
                PWM2EC = 1;        // ����������ѹ��·��pwm
                FLAG_IS_IN_CHARGING = 1;
                break;
            } // if (cnt >= 8)
        }
    } // for (i = 0; i < 10; i++)
}

void turn_dir_scan_handle(void)
{
    if (FLAG_IS_DEVICE_OPEN)
    {
        // �豸����ʱ���ſ�ʼ��ʱ���ж��Ƿ�Ҫת��
        turn_dir_ms_cnt += ONE_CYCLE_TIME_MS;
        if (turn_dir_ms_cnt >= (120000)) // ԭ���趨��2min
        {
            // �������2min
            if (0 == FLAG_DIR)
            {
                // �����ǰ����ת
                PWM0EC = 0; // �ر�������ת��pwm
                delay_ms(500);
                PWM1EC = 1;   // ��������ת��pwm
                FLAG_DIR = 1; // ��ʾ��ǰΪ��ת
            }
            else
            {
                // �����ǰ�Ƿ�ת
                PWM1EC = 0; // �ر�������ת��pwm
                delay_ms(500);
                PWM0EC = 1;   // ��������ת��pwm
                FLAG_DIR = 0; // ��ʾ��ǰΪ��ת
            }

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
        shut_down_ms_cnt += ONE_CYCLE_TIME_MS;
        if (shut_down_ms_cnt >= 900000)
        {
            // ���������15min���ػ���

            LED_WORKING_OFF(); // �رյ�Դָʾ��
            HEATING_OFF();     // �رռ���
            FLAG_IS_HEATING = 0;
            mode_flag = MODE_3; // ��һ���л�ģʽʱ������ MODE_1
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
    if (FLAG_IS_DEVICE_OPEN)
    {
        // ����豸�Ѿ�������������͹���
        return;
    }

    // ������е����˵���豸û������
    // ������Ҫ�������ڳ������
    if (FLAG_IS_IN_CHARGING)
    {
        // ������ڳ�磬������͹��ģ���Ϊ���ܻ���Ҫ���PWM�����Ƴ��
        return;
    }

label:
    GIE = 0; // ���������ж�
    // KBIF = 0;
    // KBIE = 1; // ʹ�ܼ����ж�
    T0EN = 0; // �رն�ʱ����PWM���
    T1EN = 0;
    T2EN = 0;
    // T3EN = 0;

    PWM0EC = 0;
    PWM1EC = 0;
    PWM2EC = 0;
    T2DATA = 0;
    ADEN = 0; // ��ʹ��ad

    // IOȫ����Ϊ���ģʽ������͵�ƽ��LED��ȫ������ߵ�ƽ��������LED ��
    IOP0 = 0x18;   // io������λ
    OEP0 = 0xFF;   // io�ڷ��� 1:out  0:in
    PUP0 = 0x00;   // io����������   1:enable  0:disable
    PDP0 = 0x00;   // io����������   1:enable  0:disable
    P0ADCR = 0x00; // �˿ڵ����ֹ���    1��disable
    IOP1 = 0x10;   // io������λ
    OEP1 = 0xFF;   // io�ڷ��� 1:out  0:in
    PUP1 = 0x00;   // io����������   1:enable  0:disable
    PDP1 = 0x00;   // io����������   1:enable  0:disable
    P1ADCR = 0x00; // �˿ڵ����ֹ���    1��disable

    // ����/�ػ����������ã�һ��Ҫ�жϴ�����
    // INT0M0 = 1; // INT0M0��INT0M1���������INT0Ϊ�½����ж�
    // INT0M1 = 0;
    // P11PU = 1;
    // P11OE = 0;

    // ����/ģʽ���������ã�����Ϊ����������ͨ�������жϴ���
    P01PU = 1;
    P01OE = 0;
    P01KE = 1;
    KBIF = 0;
    KBIE = 1;

    // ���ó�������ţ�ͨ�������жϴ���
    P00PD = 1;
    P00OE = 0;
    P00KE = 1;

    HFEN = 0; // �رո���ʱ��
    LFEN = 1;

    // ����ǰ�ر����� AD�� ʹ�ܻ������������ж�
    Nop();
    Nop();
    Stop();
    Nop();
    Nop();

    P01KE = 0; // �ر� ����/ģʽ�����ļ����ж�
    P00KE =0 ; // �رճ�������ŵļ����ж�
    KBIE = 0; // ��ʹ�ܼ����ж�
    KBIF = 0;
    // ���Ѻ����³�ʼ��
    CLR_RAM();
    // adc_config();

    P00PU = 0; // �ر�������
    P00PD = 0;
    P00OE = 0;    // ����ģʽ
    P00DC = 1;    // ģ������
    ADCR1 = 0xE1; // 125K��������߾��ȣ�  �ڲ�3V�ο���ѹ
    ADCR2 = 0xFF; // ADC����ʱ��Ϊ15��ADCʱ��
    ADEN = 1;     // ʹ��ADC

    adc_sel_pin(ADC_PIN_P00_AN0);
    adc_val = adc_get_val_once();
    if (adc_val < ADCDETECT_CHARING_THRESHOLD && P11D) // ������¿�������/����������������������
    {
        // ���û�а��¿���������û�в�������
        // �ر�ADC�������������ߣ�
        // ADEN = 0; //
        goto label;
    }

    P02PU = 0; // �ر�������
    P02PD = 0;
    P02OE = 0; // ����ģʽ
    P02DC = 1; // ģ������
    adc_sel_pin(ADC_PIN_P02_AN1);
    adc_val = adc_get_val();
    // if (adc_val < ADCVAL_REF_BAT_6_0_V)
    // {
    //     // �����ص������ͣ������������ǿ��Գ��
    //     FLAG_IS_NOT_OPEN_DEVICE = 1; // �������������ǿ��Գ��
    // }
    // else
    // {
    //     FLAG_IS_NOT_OPEN_DEVICE = 0;
    // }

    FLAG_IS_NOT_OPEN_DEVICE = 0;

    HFEN = 1; // ��������ʱ��

#if 0
    IOP0 = 0x18;   // io������λ
    OEP0 = 0xFF;   // io�ڷ��� 1:out  0:in
    PUP0 = 0x00;   // io����������   1:enable  0:disable
    PDP0 = 0x00;   // io����������   1:enable  0:disable
    P0ADCR = 0x00; // �˿ڵ����ֹ���    1��disable
    IOP1 = 0x10;   // io������λ
    OEP1 = 0xFF;   // io�ڷ��� 1:out  0:in
    PUP1 = 0x00;   // io����������   1:enable  0:disable
    PDP1 = 0x00;   // io����������   1:enable  0:disable
    P1ADCR = 0x00; // �˿ڵ����ֹ���    1��disable
    PMOD = 0x07;   // bit7-bit5 P17��P13��P01 io����ѡ�� 1:ģ������  0:ͨ��io
                   // bit2-bit0 P13��P01��P00 io�˿����ģʽ  1:������� 0:��©���
    DRVCR = 0x80;  // ��ͨ����
    timer0_pwm_config(); 
    timer1_pwm_config();
    timer2_pwm_config();
    // timer3_config();
    key_config();
    adc_config();
#endif
    Sys_Init();

    LED_WORKING_OFF();
    LED_FULL_CHARGE_OFF();
    LED_CHARGING_OFF();
    GIE = 1;
    // ���Ѻ�ʹ������رռ����жϣ�ʵ��Ӧ���û���������
}

void main(void)
{
    Sys_Init();

    // �ر�����ָʾ��
    LED_WORKING_OFF();
    LED_FULL_CHARGE_OFF();
    LED_CHARGING_OFF();

    // flag_bat_is_empty = 0; // �������Ż������ϵ�Ĭ�Ͼ���0��

    // �ϵ�ʱ������Ƿ���ȷ��װ:
    PWM2EC = 1; // �򿪿�����ѹ��·��pwm
    T2DATA = 20;
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
        key_scan_handle(); // ����ɨ��ʹ�����
        adc_scan_handle(); // ���ɨ��ʹ�����(Լռ��89ms)

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

            if (adc_bat_val < 2619) // ����ڳ��ʱ��⵽��ص�ѹС��6.0V
            {
                // tmp_bat_val = (u32)adc_bat_val + (294 - (u32)adc_bat_val * 157 / 1000);
                // tmp_bat_val = adc_bat_val  + (522 - adc_bat_val * 157 / 1000);
                tmp_bat_val = (adc_bat_val + 37);
            }
            else if (adc_bat_val <= 2837) // �������ص�ѹС�� 6.5V
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
            tmp_bat_val += 27; //  
            // tmp_bat_val += 39; // �� tmp_bat_val+= 27 �� tmp_bat_val += 51 ȡ�м�ֵ
            // tmp_bat_val += 51; // �� tmp_bat_val += 27 �Ļ����ϼ�һ��pwm_val

            // tmp_bat_val = (adc_bat_val + 0);

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
            if (tmp_val_cnt >= 8)
                tmp_val_cnt = 0;
            tmp_val_l[tmp_val_cnt] += tmp_val;
            tmp_val_l[tmp_val_cnt] >>= 1;
            tmp_val = 0;
            tmp_val += tmp_val_l[0];
            tmp_val += tmp_val_l[1];
            tmp_val += tmp_val_l[2];
            tmp_val += tmp_val_l[3];
            tmp_val += tmp_val_l[4];
            tmp_val += tmp_val_l[5];
            tmp_val += tmp_val_l[6];
            tmp_val += tmp_val_l[7];
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

        if (FLAG_DURING_CHARGING_BAT_IS_NULL == 0) // ֻ���ų������û�е��ʱ��������͹���
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
    if (INT0IF & INT0IE)
    {
        INT0IF = 0;
    }

    // if (T3IF & T3IE)
    // {
    //     // Ŀǰÿ1ms����һ���ж�
    //     // static u8 key_scan_cnt = 0;
    //     // key_scan_cnt++;
    //     // if (key_scan_cnt >= KEY_SCAN_TIME)
    //     // {
    //     //     key_scan_cnt = 0;
    //     //     flag_key_scan_10ms = 1;
    //     // }

    //     T3IF = 0;
    // }

    __asm;
    swapar _statusbuf;
    movra _PFLAG;
    swapr _abuf;
    swapar _abuf;
    __endasm;
}

/**************************** end of file *********************************************/
