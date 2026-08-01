#include "Oled_app.h"

#define OLED_LINE_LENGTH 16U
#define OLED_SQRT_THREE  1.732050808f

static const char *OLED_GetStateText(InverterRunState state)
{
    switch (state)
    {
        case INVERTER_RUN_STATE_RAMP_UP:
            return "START";
        case INVERTER_RUN_STATE_RUN:
            return "RUN";
        case INVERTER_RUN_STATE_RAMP_FREQ:
            return "RAMP";
        case INVERTER_RUN_STATE_RAMP_DOWN:
            return "STOPPING";
        case INVERTER_RUN_STATE_STOP:
        default:
            return "STOP";
    }
}

static void OLED_PadLine(char *line)
{
    size_t length = strlen(line);

    while (length < OLED_LINE_LENGTH)
    {
        line[length++] = ' ';
    }
    line[OLED_LINE_LENGTH] = '\0';
}

static float OLED_GetModulationIndex(const InverterStatus *status)
{
    if (status->vdc <= 0.0f)
    {
        return 0.0f;
    }

    return OLED_SQRT_THREE
           * sqrtf(status->ud * status->ud + status->uq * status->uq)
           / status->vdc;
}

void OLED_DisplayStatus(void)
{
    static char previous_lines[4][OLED_LINE_LENGTH + 1U];
    InverterStatus status;
    char lines[4][OLED_LINE_LENGTH + 1U];
    float measured_vll_rms;
    float displayed_vll_ref_rms;
    float modulation_index;
    uint8_t line;

    Inverter_GetStatus(&status);
    displayed_vll_ref_rms = status.effective_vll_ref_rms;
    modulation_index = OLED_GetModulationIndex(&status);
    (void)snprintf(lines[0], sizeof(lines[0]),
                   "OUTPUT: %s", OLED_GetStateText(status.run_state));
    (void)snprintf(lines[1], sizeof(lines[1]),
                   "F:%4.1f>%4.1fHz",
                   status.actual_frequency_hz,
                   status.target_frequency_hz);
    (void)snprintf(lines[2], sizeof(lines[2]),
                   "M:%.6f", modulation_index);
    if (status.cycle_diagnostic_valid != 0U)
    {
        measured_vll_rms = sqrtf(
            (status.u_uv_cycle_mean_square
             + status.u_vw_cycle_mean_square
             + status.u_wu_cycle_mean_square) / 3.0f);
        (void)snprintf(lines[3], sizeof(lines[3]),
                       "V:%5.2f/%5.2fV",
                       measured_vll_rms,
                       displayed_vll_ref_rms);
    }
    else
    {
        (void)snprintf(lines[3], sizeof(lines[3]),
                       "V:--.--/%5.2fV",
                       displayed_vll_ref_rms);
    }

    for (line = 0U; line < 4U; line++)
    {
        OLED_PadLine(lines[line]);
        if (strcmp(lines[line], previous_lines[line]) != 0)
        {
            OLED_ShowString(0U, line, lines[line]);
            strcpy(previous_lines[line], lines[line]);
        }
    }
}
