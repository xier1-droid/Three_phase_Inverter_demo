#include "Key_cmp.h"

#define KEY_COUNT 6U
#define KEY_DEBOUNCE_SAMPLES 3U
#define KEY6_INDEX 5U
#define KEY6_LONG_PRESS_MS 1000U

static const uint16_t key_pins[KEY_COUNT] =
{
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6
};

static uint8_t stable_pressed[KEY_COUNT];
static uint8_t debounce_count[KEY_COUNT];
static Key_State pending_key = KEY_NONE;
static uint32_t key6_press_started_ms;
static uint8_t key6_long_reported;
static uint8_t key6_ignore_until_release;

void KEY_Init(void)
{
    uint8_t index;

    pending_key = KEY_NONE;
    key6_press_started_ms = 0U;
    key6_long_reported = 0U;
    for (index = 0U; index < KEY_COUNT; index++)
    {
        stable_pressed[index] =
            (HAL_GPIO_ReadPin(GPIOE, key_pins[index]) == GPIO_PIN_RESET)
            ? 1U : 0U;
        debounce_count[index] = 0U;
    }
    key6_ignore_until_release = stable_pressed[KEY6_INDEX];
}

void KEY_Scan(void)
{
    uint8_t index;
    uint32_t now_ms = HAL_GetTick();

    for (index = 0U; index < KEY_COUNT; index++)
    {
        uint8_t sampled_pressed =
            (HAL_GPIO_ReadPin(GPIOE, key_pins[index]) == GPIO_PIN_RESET)
            ? 1U : 0U;

        if (sampled_pressed == stable_pressed[index])
        {
            debounce_count[index] = 0U;

            if ((index == KEY6_INDEX)
                && (sampled_pressed != 0U)
                && (key6_ignore_until_release == 0U)
                && (key6_long_reported == 0U)
                && ((uint32_t)(now_ms - key6_press_started_ms)
                    >= KEY6_LONG_PRESS_MS)
                && (pending_key == KEY_NONE))
            {
                pending_key = KEY6_LONG_PRESS;
                key6_long_reported = 1U;
            }
            continue;
        }

        debounce_count[index]++;
        if (debounce_count[index] < KEY_DEBOUNCE_SAMPLES)
        {
            continue;
        }

        debounce_count[index] = 0U;
        stable_pressed[index] = sampled_pressed;
        if (index == KEY6_INDEX)
        {
            if (sampled_pressed != 0U)
            {
                if (key6_ignore_until_release == 0U)
                {
                    key6_press_started_ms = now_ms;
                    key6_long_reported = 0U;
                }
            }
            else if (key6_ignore_until_release != 0U)
            {
                key6_ignore_until_release = 0U;
                key6_long_reported = 0U;
            }
            else if ((key6_long_reported == 0U)
                     && (pending_key == KEY_NONE))
            {
                pending_key = KEY6_PRESS;
            }
        }
        else if ((sampled_pressed != 0U) && (pending_key == KEY_NONE))
        {
            pending_key = (Key_State)(KEY1_PRESS + index);
        }
    }
}

Key_State KEY_GetState(void)
{
    Key_State key = pending_key;

    pending_key = KEY_NONE;
    return key;
}
