// Galaxy RS485 bus
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0
static const char TAG[] = "galaxybus";

#include "galaxybus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <driver/uart.h>
#include <driver/gpio.h>

#define	GROUP_TOUT 1
#define MASTER 0x11

struct galaxybus_s
{
   int8_t uart;
   uint8_t slave;
   uint8_t address;             // Us
   EventGroupHandle_t group;
   uart_dev_t u;
   uint8_t rx[64];
   uint8_t rxlen;
   uint8_t rxseq;
   uint8_t rxbusy:1;
};

static int icount = 0;;

void IRAM_ATTR
rx_int (void *gp)
{
   galaxybus_t *g = gp;
#if 0
   uint16_t status = g->u.int_st.val;   // read UART interrupt Status
   uint16_t i = 0,
      rx_fifo_len = g->u.status.rxfifo_cnt;     // read number of bytes in UART buffer
   uint8_t rxbuf[100];
   while (rx_fifo_len)
   {
      rxbuf[i] = g->u.fifo.rw_byte;   // read all bytes
      rx_fifo_len--;
   }
#endif
   uart_clear_intr_status (g->uart, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);
   //xEventGroupSetBits (g->group, GROUP_TOUT);
   icount++;
}

// Set up
galaxybus_t *
galaxybus_init (int8_t uart, int8_t tx, int8_t rx, int8_t de, int8_t re, uint8_t slave)
{
   if (uart < 0 || uart > 2 || tx < 0 || rx < 0 || de < 0 || tx == rx || tx == de || rx == de || tx == re || rx == re)
      return NULL;
   if (!GPIO_IS_VALID_OUTPUT_GPIO (tx) || !GPIO_IS_VALID_GPIO (rx) || !GPIO_IS_VALID_OUTPUT_GPIO (de)
       || (re >= 0 && !GPIO_IS_VALID_OUTPUT_GPIO (re)))
      return NULL;
   if (re >= 0 && re != de)
   {                            // Enable
      gpio_set_level (re, 0);
      gpio_set_direction (re, GPIO_MODE_OUTPUT);
   }
   galaxybus_t *g = malloc (sizeof (*g));
   if (!g)
      return g;
   memset (g, 0, sizeof (*g));
   if (uart == 0)
      g->u = UART0;
   else if (uart == 1)
      g->u = UART1;
   else if (uart == 2)
      g->u = UART2;
   else return NULL; // WTF
   g->slave=slave;
   g->uart = uart;
   g->group = xEventGroupCreate ();
   // Init UART
   uart_config_t uart_config = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
   };
   esp_err_t err;
   if ((err = uart_param_config (uart, &uart_config)) || (err = uart_set_pin (uart, tx, rx, de, -1))
       || (err = uart_driver_install (uart, 129, 129, 0, NULL, 0)) || (err = uart_set_mode (uart, UART_MODE_RS485_HALF_DUPLEX))
       || (err = uart_set_rx_timeout (uart, 1)))
   {
      ESP_LOGE (TAG, "UART fail %s", esp_err_to_name (err));
      free (g);
      return NULL;
   }
   //uart_isr_free (uart);
   uart_isr_register (uart, rx_int, g, ESP_INTR_FLAG_IRAM, NULL);
   uart_enable_rx_intr (uart);
   ESP_LOGD (TAG, "PN532 UART %d Tx %d Rx %d", uart, tx, rx);
   return g;
}

void *
galaxybus_end (galaxybus_t * g)
{
   if (g)
   {
      uart_disable_rx_intr (g->uart);
      uart_driver_delete (g->uart);
      free (g);
   }
   return NULL;
}

// Low level messaging
void
galaxybus_tx (galaxybus_t * g, int len, uint8_t * buf)
{
   uint32_t c = 0xAA,
      p;
   for (p = 0; p < len; p++)
      c += buf[p];
   while (c > 0xFF)
      c = (c >> 8) + (c & 0xFF);
   uint8_t csum = c;
   uart_write_bytes (g->uart, (char *) buf, len);
   uart_write_bytes (g->uart, (char *) &csum, 1);
   uart_wait_tx_done (g->uart, (len + 1) * 10 * 1000000 / 9600 / portTICK_PERIOD_MS);
}

int
galaxybus_rx (galaxybus_t * g, int max, uint8_t * buf)
{
   if (!xEventGroupWaitBits (g->group, GROUP_TOUT, pdTRUE, pdTRUE, 100 / portTICK_PERIOD_MS))
   {
      ESP_LOGI (TAG, "Rx Timeout (icount=%d)", icount);
#if 0                           // DoH
      uart_flush (g->uart);
      return -1;
#endif
   }
   size_t waiting;
   uart_get_buffered_data_len (g->uart, &waiting);      // TODO interrupt kick
   ESP_LOGI (TAG, "Waiting %d", waiting);
   if (waiting > max)
   {
      uart_flush (g->uart);
      return -1;
   }
   int len = uart_read_bytes (g->uart, buf, waiting, 0);
   // TODO address
   // TODO checksum
   return len;
}

int
galaxybus_poll (galaxybus_t * g, uint8_t address, int max, uint8_t * buf)
{
   uint8_t tx[2];
   tx[0] = address;
   tx[1] = 0x06;                // SIMple poll
   galaxybus_tx (g, 2, tx);
   return galaxybus_rx (g, max, buf);
}
