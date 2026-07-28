#include "Uart_cmp.h"
#include "stm32f4xx_hal_uart.h"

#define TX_BUF_SIZE 256

static uint8_t  uart_tx_busy = 0;          // 0=���� 1=æ
static uint8_t  uart_tx_buf[TX_BUF_SIZE];  // ���ͻ���
static uint16_t uart_tx_len = 0;           // ���η��ͳ���

#define UART_RX_DMA_BUFFER_SIZE 512
#define UART_DMA_BUFFER_SIZE 512

uint8_t uart_rx_dma_buffer[UART_RX_DMA_BUFFER_SIZE];
uint8_t uart_dma_buffer[UART_DMA_BUFFER_SIZE];
volatile uint8_t uart_flag = 0;


void Uart_init(void)
{  	
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_dma_buffer, sizeof(uart_rx_dma_buffer));
}

int my_printf(UART_HandleTypeDef *huart, const char *format, ...)
{
    // æ��ֱ�Ӷ��������ȵ���С������
    // ����㲻�붪�����Һ����ٽ���Ӷ���
    if (uart_tx_busy) return 0;

    va_list arg;
    va_start(arg, format);
    int len = vsnprintf((char *)uart_tx_buf, TX_BUF_SIZE, format, arg);
    va_end(arg);

    if (len <= 0) return 0;
    if (len > TX_BUF_SIZE) len = TX_BUF_SIZE;

    uart_tx_len = (uint16_t)len;
    uart_tx_busy = 1;

    if (HAL_UART_Transmit_IT(huart, uart_tx_buf, uart_tx_len) != HAL_OK)
    {
        uart_tx_busy = 0; // ����ʧ�ܣ��ͷ�
        return 0;
    }
    return len;
}

// name=value command table, single-pass parse (see uart_proc below)
typedef struct
{
    const char *name;
    void (*handler)(float val);
} uart_cmd_t;

static void cmd_vp(float v)
{
    InverterStatus status;

    (void)Inverter_SetParameter(INVERTER_PARAMETER_VOLTAGE_KP, v);
    Inverter_GetStatus(&status);
    my_printf(&huart1, "DQ_Voltage: kp=%.4f ki=%.4f\r\n",
              status.config.voltage_kp, status.config.voltage_ki);
}

static void cmd_vi(float v)
{
    InverterStatus status;

    (void)Inverter_SetParameter(INVERTER_PARAMETER_VOLTAGE_KI, v);
    Inverter_GetStatus(&status);
    my_printf(&huart1, "DQ_Voltage: kp=%.4f ki=%.4f\r\n",
              status.config.voltage_kp, status.config.voltage_ki);
}

static void cmd_vref(float v)
{
    InverterStatus status;

    /* The user-facing reference is line-to-line RMS voltage. */
    (void)Inverter_SetParameter(INVERTER_PARAMETER_VLL_REF_RMS, v);
    Inverter_GetStatus(&status);
    my_printf(&huart1, "vref=%.3f Vrms line-to-line\r\n",
              status.config.vll_ref_rms);
}

static void cmd_vstat(float v)
{
    InverterStatus status;
    float vll_feedback_rms;

    (void)v;
    Inverter_GetStatus(&status);
    /* Keep this diagnostic sqrtf outside the 20 kHz control ISR. */
    vll_feedback_rms = sqrtf(status.vd * status.vd + status.vq * status.vq)
                       * 1.224744871f;

    my_printf(&huart1,
              "vll_ref=%.3f vd_ref=%.3f vd=%.3f vq=%.3f "
              "vll_fb=%.3f ud=%.3f uq=%.3f kp=%.4f ki=%.4f\r\n",
              status.config.vll_ref_rms, status.vd_ref, status.vd, status.vq,
              vll_feedback_rms, status.ud, status.uq,
              status.config.voltage_kp, status.config.voltage_ki);
}

static void print_cascade_gains(void)
{
    InverterStatus status;

    Inverter_GetStatus(&status);
    my_printf(&huart1,
              "DQ_Cascade: ovp=%.4f ovi=%.4f icp=%.4f ici=%.4f\r\n",
              status.config.outer_kp, status.config.outer_ki,
              status.config.current_kp, status.config.current_ki);
}

static void report_cascade_set_result(bool accepted)
{
    if (accepted)
    {
        print_cascade_gains();
    }
    else if (wave_enable_tim8 != 0U)
    {
        my_printf(&huart1, "busy\r\n");
    }
    else
    {
        my_printf(&huart1, "invalid\r\n");
    }
}

static void cmd_ovp(float v)
{
    report_cascade_set_result(
        Inverter_SetParameter(INVERTER_PARAMETER_OUTER_KP, v));
}

static void cmd_ovi(float v)
{
    report_cascade_set_result(
        Inverter_SetParameter(INVERTER_PARAMETER_OUTER_KI, v));
}

static void cmd_icp(float v)
{
    report_cascade_set_result(
        Inverter_SetParameter(INVERTER_PARAMETER_CURRENT_KP, v));
}

static void cmd_ici(float v)
{
    report_cascade_set_result(
        Inverter_SetParameter(INVERTER_PARAMETER_CURRENT_KI, v));
}

static void cmd_dqstat(float v)
{
    InverterStatus status;

    (void)v;
    Inverter_GetStatus(&status);
    my_printf(&huart1,
              "mode=%u vdc=%.2f vd_ref=%.2f vd=%.2f vq=%.2f "
              "id_ref=%.2f id=%.2f iq_ref=%.2f iq=%.2f "
              "ud=%.2f uq=%.2f ovp=%.4f ovi=%.4f "
              "icp=%.4f ici=%.4f current_ref_limited=%u "
              "voltage_limited=%u\r\n",
              status.mode, status.config.vdc,
              status.vd_ref, status.vd, status.vq,
              status.id_ref, status.id, status.iq_ref, status.iq,
              status.ud, status.uq,
              status.config.outer_kp, status.config.outer_ki,
              status.config.current_kp, status.config.current_ki,
              status.current_ref_limited, status.voltage_limited);
}

static void cmd_wave8(float v)
{
    if (v != 0.0f)
        Inverter_Start();
    else
        Inverter_Stop();
    my_printf(&huart1, "wave8=%d\r\n", wave_enable_tim8);
}

static void cmd_clrfault(float v)
{
    if (v != 0.0f)
        Inverter_ClearFault();
    my_printf(&huart1, "clrfault done, wave8=%d\r\n", wave_enable_tim8);
}

static const uart_cmd_t uart_cmds[] =
{
    {"vp", cmd_vp},
    {"vi", cmd_vi},
    {"vref", cmd_vref},
    {"vstat", cmd_vstat},
    {"ovp", cmd_ovp},
    {"ovi", cmd_ovi},
    {"icp", cmd_icp},
    {"ici", cmd_ici},
    {"dqstat", cmd_dqstat},
    {"wave8", cmd_wave8},
    {"clrfault", cmd_clrfault},
};
#define UART_CMD_COUNT (sizeof(uart_cmds) / sizeof(uart_cmds[0]))

// @brief non-blocking uart command processing, called from main loop
void uart_proc(void)
{
    // 1. no new frame received yet, return immediately
    if(uart_flag == 0)
        return;

    // 2. clear receive flag to avoid re-processing the same frame
    uart_flag = 0;

    // 3. single-pass parse: find '=' once, match name against command table
    char *eq = strchr((char *)uart_dma_buffer, '=');
    uint8_t any_matched = 0;

    if (eq != NULL)
    {
        size_t name_len = (size_t)(eq - (char *)uart_dma_buffer);
        float val = 0.0f;

        if (sscanf(eq + 1, "%f", &val) == 1)
        {
            for (size_t idx = 0; idx < UART_CMD_COUNT; idx++)
            {
                size_t cmd_len = strlen(uart_cmds[idx].name);
                if (name_len == cmd_len &&
                    strncmp((char *)uart_dma_buffer, uart_cmds[idx].name, name_len) == 0)
                {
                    uart_cmds[idx].handler(val);
                    any_matched = 1;
                    break;
                }
            }
        }
    }

    // no known command matched: echo back the raw received data
    if (!any_matched)
    {
        my_printf(&huart1,"%s\n",uart_dma_buffer);
    }
    // note: DMA reception fills the buffer via interrupt, main loop only polls uart_flag
    // 4. clear receive buffer, ready for next frame
    memset(uart_dma_buffer, 0, sizeof(uart_dma_buffer));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart_tx_busy = 0; // ������ɣ��ͷ�
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 1. ȷ����Ŀ�괮�� (USART1)
    if (huart->Instance == USART1)
    {
        // 2. ����ֹͣ��ǰ�� DMA ���� (������ڽ�����)
        //    ��Ϊ�����ж���ζ�ŷ��ͷ��Ѿ�ֹͣ����ֹ DMA �����ȴ������
        HAL_UART_DMAStop(huart);

        // 3. �� DMA ����������Ч������ (Size ���ֽ�) ���Ƶ�������������
        memcpy(uart_dma_buffer, uart_rx_dma_buffer, Size); 
        // ע�⣺����ʹ���� Size��ֻ����ʵ�ʽ��յ�������

        // 4. ����"����֪ͨ��"��������ѭ�������ݴ�����
        uart_flag = 1;

        // 5. ��� DMA ���ջ�������Ϊ�´ν�����׼��
        //    ��Ȼ memcpy ֻ������ Size �������������������������
        memset(uart_rx_dma_buffer, 0, sizeof(uart_rx_dma_buffer));

        // 6. **�ؼ�������������һ�� DMA ���н���**
        //    �����ٴε��ã�����ֻ�������һ��
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_dma_buffer, sizeof(uart_rx_dma_buffer));
        
        // 7. ���֮ǰ�ر��˰����жϣ�������Ҫ�������ٴιر� (������Ҫ)
         __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}
