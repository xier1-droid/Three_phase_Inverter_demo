#include "Key_cmp.h"

#define KEY_COUNT 6U
#define KEY_DEBOUNCE_SAMPLES 3U

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

void KEY_Init(void)
{
    uint8_t index;

    pending_key = KEY_NONE;
    for (index = 0U; index < KEY_COUNT; index++)
    {
        stable_pressed[index] =
            (HAL_GPIO_ReadPin(GPIOE, key_pins[index]) == GPIO_PIN_RESET)
            ? 1U : 0U;
        debounce_count[index] = 0U;
    }
}

void KEY_Scan(void)
{
    uint8_t index;

    for (index = 0U; index < KEY_COUNT; index++)
    {
        uint8_t sampled_pressed =
            (HAL_GPIO_ReadPin(GPIOE, key_pins[index]) == GPIO_PIN_RESET)
            ? 1U : 0U;

        if (sampled_pressed == stable_pressed[index])
        {
            debounce_count[index] = 0U;
            continue;
        }

        debounce_count[index]++;
        if (debounce_count[index] < KEY_DEBOUNCE_SAMPLES)
        {
            continue;
        }

        debounce_count[index] = 0U;
        stable_pressed[index] = sampled_pressed;
        if ((sampled_pressed != 0U) && (pending_key == KEY_NONE))
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
