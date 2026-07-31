#include "Oled_app.h"

#define OLED_LINE_LENGTH 16U

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

void OLED_DisplayStatus(void)
{
    static char previous_lines[4][OLED_LINE_LENGTH + 1U];
    InverterStatus status;
    char lines[4][OLED_LINE_LENGTH + 1U];
    uint8_t line;

    Inverter_GetStatus(&status);
    (void)snprintf(lines[0], sizeof(lines[0]),
                   "OUTPUT: %s", OLED_GetStateText(status.run_state));
    (void)snprintf(lines[1], sizeof(lines[1]),
                   "FREQ: %4.1fHz", status.actual_frequency_hz);
    (void)snprintf(lines[2], sizeof(lines[2]),
                   "TARGET: %2.0fHz", status.target_frequency_hz);
    lines[3][0] = '\0';

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
