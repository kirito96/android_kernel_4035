
#include <platform/mt_gpt.h>
#include <platform/mt_sleep.h>

#include <printf.h>

void mtk_sleep(u32 timeout, kal_bool en_deep_idle)
{
    printf("enter mtk_sleep, timeout = %d\n", timeout);

    gpt_busy_wait_ms(timeout);
}

void sc_mod_init(void)
{
    // do nothing
}

void sc_mod_exit(void)
{
    // do nothing
}