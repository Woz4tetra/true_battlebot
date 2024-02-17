#include <motors/esc_command_lookup.h>

int frequency_to_command(float frequency)
{
    int index = 0;
    for (index = 0; index < ESC_COMMAND_LOOKUP_LEN; index++)
    {
        if (ESC_COMMAND_LOOKUP[index] > frequency)
        {
            break;
        }
    }
    return index - 256;
}