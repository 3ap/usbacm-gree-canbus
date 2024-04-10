#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/fdcan.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>

#include <alloca.h>
#include <sys/param.h>
#include <string.h>

/*
 * Constants
 */

/*
 * Structs
 */

typedef struct __attribute((packed))
{
    uint8_t x0; /* AA */
    uint8_t x1; /* AA */
    uint8_t sz   : 4;
    uint8_t type : 4;
    union
    {
        struct
        {
            uint16_t addr16_BE;
            uint8_t data_addr16[];
        };
        struct
        {
            uint32_t addr32_BE;
            uint8_t data_addr32[];
        };
    };
} gree_can_t;

#define CAN_MAX_PAYLOAD_SIZE_BYTES 8
#define GREE_CAN_MAX_SIZE (sizeof(gree_can_t) + CAN_MAX_PAYLOAD_SIZE_BYTES + 1/*CS*/ + 1/*END*/)

typedef struct __attribute((packed))
{
    uint8_t sz;
    uint32_t addr;
    uint8_t data[CAN_MAX_PAYLOAD_SIZE_BYTES];
} can_t;

/*
 * Global variables
 */
static QueueHandle_t queue_CAN_RX;
static uint8_t usbd_control_buffer[128];
static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,
    .idProduct = 0x5740,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
} };

static const struct usb_endpoint_descriptor data_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
} };

static const struct {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
    .header = {
        .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
        .bcdCDC = 0x0110,
    },
    .call_mgmt = {
        .bFunctionLength =
            sizeof(struct usb_cdc_call_management_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
        .bmCapabilities = 0,
        .bDataInterface = 1,
    },
    .acm = {
        .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_ACM,
        .bmCapabilities = 0,
    },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
     }
};

static const struct usb_interface_descriptor comm_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .iInterface = 0,

    .endpoint = comm_endp,

    .extra = &cdcacm_functional_descriptors,
    .extralen = sizeof(cdcacm_functional_descriptors)
} };

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,

    .endpoint = data_endp,
} };

static const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = comm_iface,
}, {
    .num_altsetting = 1,
    .altsetting = data_iface,
} };

static const struct usb_config_descriptor config[] = {
  {
      .bLength = USB_DT_CONFIGURATION_SIZE,
      .bDescriptorType = USB_DT_CONFIGURATION,
      .wTotalLength = 0,
      .bNumInterfaces = 2,
      .bConfigurationValue = 1,
      .iConfiguration = 0,
      .bmAttributes = 0x80,
      .bMaxPower = 0x32,

      .interface = ifaces,
  }
};

static const char * usb_strings[] = {
    "Sergey Nazaryev",
    "CDC-ACM Demo",
    "DEMO",
};

/*
 * Code
 */

static uint8_t calc_cs( uint8_t *raw, uint8_t sz )
{
    uint8_t i;
    uint8_t cs;

    cs=0;
    for( i=0; i<sz; i++ )
        cs ^= raw[i];

    return cs;
}

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev,
    struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
    void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    (void)complete;
    (void)buf;
    (void)usbd_dev;

    switch (req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
        /*
         * Linux cdc_acm driver requires this to be implemented
         * even though it's optional in the CDC spec, and we don't
         * advertise it in the ACM functional descriptor.
         */
        return USBD_REQ_HANDLED;
    case USB_CDC_REQ_SET_LINE_CODING:
        if (*len < sizeof(struct usb_cdc_line_coding))
            return USBD_REQ_NOTSUPP;

        return USBD_REQ_HANDLED;
    }
    return USBD_REQ_NOTSUPP;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    if (len)
    {
        while (usbd_ep_write_packet(usbd_dev, 0x82, buf, len) == 0);
    }
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
            cdcacm_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64,
            NULL);
    usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

    usbd_register_control_callback(
                usbd_dev,
                USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
                USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                cdcacm_control_request);
}

static void gree_can_send_event(usbd_device *usbd_dev, can_t *buf)
{
    uint8_t sz;
    gree_can_t *frame;

    sz = 2/*x0,x1*/ + 1/*sz,type*/
         + sizeof(frame->addr32_BE) + buf->sz
         + 1/*CRC*/ + 1/*x2*/;

    frame = alloca( sz );

    frame->x0 = 0xAA;
    frame->x1 = 0xAA;
    frame->sz = buf->sz;
    frame->type = 0x8;
    frame->addr32_BE = __bswap32(buf->addr);

    memcpy( frame->data_addr32, buf->data, buf->sz );
    frame->data_addr32[frame->sz] = calc_cs( ((uint8_t *) frame) + 2 /* skip x0, x1 */,
                                             sz - 2/*x0,x1*/ - 1/*CRC*/ - 1/*x2*/ );
    frame->data_addr32[frame->sz+1] = 0xFF;

    while (usbd_ep_write_packet(usbd_dev, 0x82, (char *) frame, sz) == 0);
}

static void can_rx_queue_poll(usbd_device *usbd_dev)
{
    can_t buf;
    while (xQueueReceive(queue_CAN_RX, &buf, 0) == pdTRUE)
    {
        gree_can_send_event(usbd_dev, &buf);
    }
}

static void usbcdc_acm_task(void *args __attribute__((unused)))
{
    usbd_device *usbd_dev;

    usbd_dev = usbd_init(&st_usbfs_v2_usb_driver, &dev, config,
        usb_strings, 5,
        usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

    while (1)
    {
        usbd_poll(usbd_dev);
        if(queue_CAN_RX)
            can_rx_queue_poll(usbd_dev);
    }
}

static void main_task(void *args __attribute__((unused)))
{
    queue_CAN_RX = xQueueCreate( 16, sizeof(can_t) );
    can_t buf = { .data = {0xDE, 0xAD, 0xBE, 0xEF}, .sz = 4, .addr = 0xAABBCCDD };
    while (1)
    {
        xQueueSend(queue_CAN_RX, &buf, pdMS_TO_TICKS(1 * 1000));

        gpio_clear(GPIOA, GPIO0); /* Turn on WORD LED */
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set(GPIOA, GPIO0);   /* Turn off WORD LED */

        vTaskDelay(pdMS_TO_TICKS(5 * 1000));
    }
}

static void can_setup(void)
{
    int rc;

    /* Configure CAN RX (PB8) and CAN TX (PB9) */
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8|GPIO9);
    gpio_set_af(GPIOB, GPIO_AF9, GPIO8|GPIO9);

    /* Enable FDCAN & use PCLK as a clock source for FDCAN */
    rcc_periph_clock_enable(RCC_FDCAN);
    RCC_CCIPR &= ~(RCC_CCIPR_SEL_MASK << RCC_CCIPR_FDCANSEL_SHIFT);
    RCC_CCIPR |= (RCC_CCIPR_FDCANSEL_PCLK << RCC_CCIPR_FDCANSEL_SHIFT);

    /* Switch to configuration mode */
    rc = fdcan_init(CAN1, 1000);
    if (rc != FDCAN_E_OK)
        return;

    /* 170 MHz base clock, SJW=1, TS1=14, TS=2, Prescaler=200 -> bitrate 50000 */
    fdcan_set_can(CAN1,
             true, true, false, false,
             0, 13, 1, 199 );

    /* Switch to running mode */
    fdcan_start(CAN1, FDCAN_CCCR_INIT_TIMEOUT);
    if (rc != FDCAN_E_OK)
        return;
}

int main(void) {
    /*
     * It's required to explicitly set the pointer to our vector
     * table at boot because the application could start in DFU mode
     * immediately after flashing firmware, and in this case SCB_VTOR
     * is affected by bootROM and points to its vector table instead
     * of ours.
     *
     * Moreover, it's incorrect to set it to 0x0000_0000, because in
     * DFU mode it's not aliased with flash memory.
     */

    /* Set Vector Table Offset to our memory based vector table */
    SCB_VTOR = (uint32_t)&vector_table;

    /* Configure HSI frequency to 170MHz and use it as a system clock */
    RCC_ICSCR = 0x409B0000; /* BootROM changes this register in DFU mode,
                               and because of it 170MHz becomes 180Mhz */
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_170MHZ]);

    /* Setup FDCAN */
    can_setup();

    /* Enable HSI48 to make USB IP core works */
    rcc_osc_on(RCC_HSI48);
    rcc_wait_for_osc_ready(RCC_HSI48);

    /* Enable GPIOA for STAT_LED & WORD_LED */
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15); /* STAT LED */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);  /* WORD LED */
    gpio_set(GPIOA, GPIO15); /* Turn off STAT LED */
    gpio_set(GPIOA, GPIO0);  /* Turn off WORD LED */

    // crs_autotrim_usb_enable();
    xTaskCreate(usbcdc_acm_task, "ACM", 1024, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(main_task, "MAIN", 50, NULL, configMAX_PRIORITIES-1, NULL);
    vTaskStartScheduler();
    for (;;);
}
