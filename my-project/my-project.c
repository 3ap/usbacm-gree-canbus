#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "api.h"
#include "api-asm.h"

static void main_task(void *args __attribute__((unused))) {
       /* add your own code */
       uint32_t rev = 0xaabbccdd;
       rev = rev_bytes(rev);
       my_func(rev);
}

int main(void) {
       xTaskCreate(main_task, "MAIN", 100, NULL, configMAX_PRIORITIES-1, NULL);
       vTaskStartScheduler();
       for (;;);
}
