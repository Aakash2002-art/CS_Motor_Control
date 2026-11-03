/**
 * @file    console.c
 * @brief   Minimal serial console output helpers.
 */
#include "console.h"
#include "bsp_hw.h"

#define CONSOLE_NUMBER_BUFFER   (12U)
#define CONSOLE_MAX_STRING      (200U)
#define CONSOLE_MAX_DECIMALS    (6U)
#define CONSOLE_BASE_10         (10U)
#define CONSOLE_BASE_16         (16U)

/**
 * @brief Send a zero-terminated string.
 */
void Console_PrintString(const char *text)
{
    uint16_t length = 0U;

    while ((text[length] != '\0') && (length < CONSOLE_MAX_STRING))
    {
        ++length;
    }
    Bsp_UartWrite(text, length);
}

/**
 * @brief Send a string followed by CR/LF.
 */
void Console_PrintLine(const char *text)
{
    Console_PrintString(text);
    Console_PrintNewLine();
}

/**
 * @brief Send CR/LF.
 */
void Console_PrintNewLine(void)
{
    Bsp_UartWrite("\r\n", 2U);
}

/**
 * @brief Print an unsigned decimal number (no stdio/printf).
 */
void Console_PrintUint(uint32_t value)
{
    char buffer[CONSOLE_NUMBER_BUFFER];
    uint32_t index = CONSOLE_NUMBER_BUFFER;
    uint32_t remaining = value;

    do
    {
        --index;
        buffer[index] = (char)('0' + (char)(remaining % CONSOLE_BASE_10));
        remaining /= CONSOLE_BASE_10;
    } while ((remaining != 0U) && (index > 0U));

    Bsp_UartWrite(&buffer[index], (uint16_t)(CONSOLE_NUMBER_BUFFER - index));
}

/**
 * @brief Print a signed decimal number.
 */
void Console_PrintInt(int32_t value)
{
    uint32_t magnitude;

    if (value < 0)
    {
        Bsp_UartWrite("-", 1U);
        magnitude = (uint32_t)(-(value + 1)) + 1U;   /* safe for INT32_MIN */
    }
    else
    {
        magnitude = (uint32_t)value;
    }
    Console_PrintUint(magnitude);
}

/**
 * @brief Print a float with a fixed number of decimals (no printf).
 */
void Console_PrintFloat(float32_t value, uint8_t decimals)
{
    uint32_t scale = 1U;
    uint8_t digit_index;
    uint8_t limited = decimals;
    float32_t magnitude = value;
    uint32_t whole;
    uint32_t fraction;

    if (limited > CONSOLE_MAX_DECIMALS)
    {
        limited = (uint8_t)CONSOLE_MAX_DECIMALS;
    }
    if (value < 0.0F)
    {
        Bsp_UartWrite("-", 1U);
        magnitude = -value;
    }
    for (digit_index = 0U; digit_index < limited; ++digit_index)
    {
        scale *= CONSOLE_BASE_10;
    }

    {
        const uint32_t scaled = (uint32_t)((magnitude * (float32_t)scale) + 0.5F);

        whole = scaled / scale;
        fraction = scaled % scale;
    }
    Console_PrintUint(whole);

    if (limited > 0U)
    {
        uint32_t divisor = scale / CONSOLE_BASE_10;

        Bsp_UartWrite(".", 1U);
        while (divisor > 0U)
        {
            Console_PrintUint((fraction / divisor) % CONSOLE_BASE_10);
            divisor /= CONSOLE_BASE_10;
        }
    }
}

/**
 * @brief Print a value as hexadecimal.
 */
void Console_PrintHex(uint32_t value)
{
    char buffer[CONSOLE_NUMBER_BUFFER];
    uint32_t index = CONSOLE_NUMBER_BUFFER;
    uint32_t remaining = value;

    do
    {
        const uint32_t nibble = remaining % CONSOLE_BASE_16;

        --index;
        buffer[index] = (nibble < CONSOLE_BASE_10) ? (char)('0' + (char)nibble)
                                                   : (char)('A' + (char)(nibble - CONSOLE_BASE_10));
        remaining /= CONSOLE_BASE_16;
    } while ((remaining != 0U) && (index > 0U));

    Bsp_UartWrite(&buffer[index], (uint16_t)(CONSOLE_NUMBER_BUFFER - index));
}

/**
 * @brief Fetch one received character if available.
 */
bool Console_ReadChar(char *character)
{
    uint8_t byte = 0U;
    const bool available = Bsp_UartReadByte(&byte);

    if (available)
    {
        *character = (char)byte;
    }
    return available;
}
