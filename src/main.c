#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static void main_task(void *args __attribute__((unused))) {
}

int main(void) {
       xTaskCreate(main_task, "MAIN", 100, NULL, configMAX_PRIORITIES-1, NULL);
       vTaskStartScheduler();
       for (;;);
}
