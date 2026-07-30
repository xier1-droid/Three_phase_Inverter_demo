#include "Uart_cmp.h"
#include "Inverter_sampling.h"
#include "stm32f4xx_hal_uart.h"

#define TX_BUF_SIZE 256
#define JUSTFLOAT_CHANNEL_COUNT 1U
#define JUSTFLOAT_DATA_SIZE     (JUSTFLOAT_CHANNEL_COUNT * sizeof(float))
#define JUSTFLOAT_FRAME_SIZE    (JUSTFLOAT_DATA_SIZE + 4U)

static uint8_t  uart_tx_busy = 0;          // 0=���� 1=æ
static uint8_t  uart_tx_buf[TX_BUF_SIZE];  // ���ͻ���
static uint16_t uart_tx_len = 0;           // ���η��ͳ���

static volatile uint8_t justfloat_enabled = 0U;

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
    if (justfloat_enabled != 0U) return 0;

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

bool JustFloat_IsEnabled(void)
{
    return (justfloat_enabled != 0U);
}

void JustFloat_Task(void)
{
    static uint32_t last_send_ms = 0U;
    static const uint8_t frame_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    float vdc;
    uint32_t now_ms = HAL_GetTick();

    if ((justfloat_enabled == 0U) ||
        ((uint32_t)(now_ms - last_send_ms) < 2U))
    {
        return;
    }
    last_send_ms = now_ms;

    if (uart_tx_busy != 0U)
    {
        return;
    }

    vdc = InverterSampling_GetVdc();
    memcpy(&uart_tx_buf[0U * sizeof(float)], &vdc, sizeof(float));
    memcpy(&uart_tx_buf[JUSTFLOAT_DATA_SIZE], frame_tail, sizeof(frame_tail));

    uart_tx_len = (uint16_t)JUSTFLOAT_FRAME_SIZE;
    uart_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart1, uart_tx_buf, uart_tx_len) != HAL_OK)
    {
        uart_tx_busy = 0U;
    }
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

static void cmd_jf(float v)
{
    justfloat_enabled = (v != 0.0f) ? 1U : 0U;
}

static const uart_cmd_t uart_cmds[] =
{
    {"vp", cmd_vp},
    {"vi", cmd_vi},
    {"vref", cmd_vref},
    {"vstat", cmd_vstat},
    {"wave8", cmd_wave8},
    {"clrfault", cmd_clrfault},
    {"jf", cmd_jf},
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
