// Galaxy RS485 bus
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0
static const char TAG[] = "galaxybus";

#include "galaxybus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <driver/timer.h>
#include <driver/gpio.h>

#define MASTER 0x11

struct galaxybus_s
{
   EventGroupHandle_t group;
   int8_t timer;                // Which timer
   int8_t tx;                   // Tx pin
   int8_t rx;                   // Rx pin (can be same as tx)
   int8_t de;                   // DE pin
   int8_t re;                   // RE pin (optional, can be same as DE)
   int8_t clk;                  // Debug pin (optional)
   uint8_t address;             // Us
   uint32_t txpre;              // Pre tx drive(high) (bits)
   uint32_t txpost;             // Post tx drive(high) (bits)
   uint32_t gap;                // Gap after which we assume end of message (bits)
   uint32_t txgap;              // We are sending pre / post drive
   uint8_t txlen;               // The length of tx buf
   volatile uint8_t txpos;      // Position in tx buf we are sending(checked by app level)
   uint8_t txdata[GALAXYBUSMAX];        // The tx message
   int8_t rxerr;                //  The current in progress message rx error
   int8_t rxerrorreport;        // The rxerror of the last stored message
   uint8_t rxpos;               // Where we are in rx buf
   uint8_t rxlen;               // Length of last received mesage in rxbuf
   uint8_t rxdue;               // The expected message sequence
   uint8_t rxsum;               // The current checksum
   uint8_t rxgap;               // The remaining end of message timeout
   volatile uint8_t rxseq;      // The last received message sequence
   uint8_t rxdata[GALAXYBUSMAX];        // The Rx data
   uint8_t subbit;              // Sub bit count
   uint8_t bit;                 // Bit count
   uint8_t shift;               // Byte
   uint8_t slave:1;             // We are slave
   uint8_t started:1;           // Int handler started
   uint8_t txrx:1;              // Mode, true for rx , false for tx //Tx
   uint8_t txdue:1;             // We are due to send a message
   uint8_t txhold:1;            // We are in app Tx call and need to hold of sending as copying to buffer
   uint8_t rxignore:1;          // This message is not for us so being ignored
   uint8_t tick:1;              // clk tick
};

#define TIMER_DIVIDER         16        //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

#define	GROUP_RX_OK	1       // Rx is not busy
#define	GROUP_TX_OK	2       // Tx is not busy
#define	GROUP_RX_READY	4       // New Rx message ready

// Low level direct GPIO controls
static inline void
gpio_in (int8_t r)
{
   if (r >= 32)
      GPIO_REG_WRITE (GPIO_ENABLE1_W1TC_REG, 1 << (r - 32));
   else if (r >= 0)
      GPIO_REG_WRITE (GPIO_ENABLE_W1TC_REG, 1 << r);
}

static inline void
gpio_out (int8_t r)
{
   if (r >= 32)
      GPIO_REG_WRITE (GPIO_ENABLE1_W1TS_REG, 1 << (r - 32));
   else if (r >= 0)
      GPIO_REG_WRITE (GPIO_ENABLE_W1TS_REG, 1 << r);
}

static inline void
gpio_set (int8_t r)
{
   if (r >= 32)
      GPIO_REG_WRITE (GPIO_OUT1_W1TS_REG, 1 << (r - 32));
   else if (r >= 0)
      GPIO_REG_WRITE (GPIO_OUT_W1TS_REG, 1 << r);
}

static inline void
gpio_clr (int8_t r)
{
   if (r >= 32)
      GPIO_REG_WRITE (GPIO_OUT1_W1TC_REG, 1 << (r - 32));
   else if (r >= 0)
      GPIO_REG_WRITE (GPIO_OUT_W1TC_REG, 1 << r);
}

static inline uint32_t
gpio_get (int8_t r)
{
   if (r >= 32)
      return (GPIO_REG_READ (GPIO_IN1_REG) >> (r - 32)) & 1;
   else if (r >= 0)
      return (GPIO_REG_READ (GPIO_IN_REG) >> r) & 1;
   else
      return 0;
}

static inline void
rs485_mode_rx (galaxybus_t * g)
{                               // Switch to rx mode
   if (g->tx == g->rx)
      gpio_in (g->rx);          // Input
   gpio_clr (g->de);
   g->txrx = 1;                 // Rx mode
}

static inline void
rs485_mode_tx (galaxybus_t * g)
{                               // Switch to tx mode
   gpio_set (g->de);
   if (g->tx == g->rx)
      gpio_out (g->rx);
   g->txgap = g->txpre;
   g->txdue = 0;
   g->txrx = 0;                 // Tx mode
}

void IRAM_ATTR
timer_isr (void *gp)
{
   galaxybus_t *g = gp;
   timer_group_intr_clr_in_isr (TIMER_GROUP_0, g->timer);
   timer_group_enable_alarm_in_isr (TIMER_GROUP_0, g->timer);
   if (g->clk >= 0)
   {
      g->tick = 1 - g->tick;
      if (g->tick)
         gpio_set (g->clk);
      else
         gpio_clr (g->clk);
   }
   if (g->txrx)
   {                            // Rx
      uint8_t v = gpio_get (g->rx);
      if (!v && !g->bit)
      {                         // Idle, and low, so this is start of start bit
         g->subbit = 1;         // Centre of start bit (+/- 1/6 of a bit)
         g->bit = 10;
      }
      if (g->subbit--)
         return;
      g->subbit = 2;            // Three sub bits per bit
      if (!g->bit)
      {                         // Idle
         if (g->rxgap)
            g->rxgap--;
         else
         {                      // End of rx
            g->rxignore = 0;
            if (g->rxpos)
            {
               // Message received
               if (g->rxsum != g->rxdata[g->rxpos - 1])
                  g->rxerr = GALAXYBUSCHECKSUM;
               g->rxlen = g->rxpos;
               g->rxerrorreport = g->rxerr;
               g->rxseq++;
               if (g->slave)
                  g->txdue = 1; // Send reply as we are slave
               g->rxpos = 0;    // ready for next message
               BaseType_t xHigherPriorityTaskWoken = pdFALSE;
               xEventGroupSetBitsFromISR (g->group, GROUP_RX_READY, &xHigherPriorityTaskWoken);
               xEventGroupSetBitsFromISR (g->group, GROUP_RX_OK, &xHigherPriorityTaskWoken);
            }
            if (g->txdue)
               rs485_mode_tx (g);       // Can start tx
         }
         return;
      }
      g->bit--;
      if (g->bit == 9)
      {                         // Start bit
         if (v)
         {                      // Missing start bit
            g->rxerr = GALAXYBUSSTARTBIT;
            g->bit = 0;         // Back to idle
         }
         return;
      }
      if (g->bit)
      {                         // Shift in
         g->shift >>= 1;
         if (v)
            g->shift |= 0x80;
         return;
      }
      // Stop bit
      if (!v)
         g->rxerr = (g->shift ? GALAXYBUSSTOPBIT : GALAXYBUSBREAK);     // Missing stop bit
      g->rxgap = g->gap;        // Look for end of message
      if (!g->rxpos)
         g->rxerr = 0;          // Clear errors, new message
      // Checksum logic
      if (!g->rxpos)
         g->rxsum = 0xAA;
      else
      {
         uint8_t l = g->rxdata[g->rxpos - 1];
         if ((int) g->rxsum + l > 0xFF)
            g->rxsum++;         // 1's comp
         g->rxsum += l;
      }
      if (!g->rxpos && !g->shift)
         return;                // Ignore zero leading
      if (!g->rxpos && g->shift != g->address && g->address != 0xFF && g->shift != 0xFF)
         g->rxignore = 1;       // Not addressed to us, ignore
      if (g->rxignore)
         return;                // Not for us
      if (!g->rxpos)
      {
         xEventGroupClearBitsFromISR (g->group, GROUP_RX_OK);
      }
      // End of byte
      if (g->rxpos >= GALAXYBUSMAX)
         g->rxerr = GALAXYBUSTOOBIG;
      else
         g->rxdata[g->rxpos++] = g->shift;
      return;
   }
   // Tx
   if (g->subbit--)
      return;
   g->subbit = 2;               // Three sub bits per bit
   uint8_t t = 1;
   if (g->txgap)
   {
      t = 1;
      g->txgap--;
      if (!g->txgap)
      {                         // End of gap
         if (g->txpos)
         {                      // End of message
            g->txpos = 0;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xEventGroupSetBitsFromISR (g->group, GROUP_TX_OK, &xHigherPriorityTaskWoken);
            rs485_mode_rx (g);  // Switch back to rx
            return;
         }
         // Start of message
         if (g->txhold)
            g->txgap++;         // Wait, app is writing new message
         else
         {                      // Start sending
            xEventGroupClearBitsFromISR (g->group, GROUP_RX_OK);
            g->bit = 9;
            g->shift = g->txdata[g->txpos++];
         }
      }
   } else if (g->bit)
   {
      if (g->bit == 9)
         t = 0;                 // Start bit
      else
      {                         // Data bit
         t = (g->shift & 1);
         g->shift >>= 1;
      }
      g->bit--;
   } else
   {                            // Stop bit and prep next byte
      if (g->txpos < g->txlen)
      {
         g->shift = g->txdata[g->txpos++];
         g->bit = 9;
      } else
         g->txgap = g->txpost;  // End of message
   }
   if (t)
      gpio_set (g->tx);
   else
      gpio_clr (g->tx);
}

// Set up
galaxybus_t *
galaxybus_init (int8_t timer, int8_t tx, int8_t rx, int8_t de, int8_t re, int8_t clk, uint8_t slave)
{
   if (timer < 0 || tx < 0 || rx < 0 || de < 0 || tx == de || rx == de || tx == re || rx == re)
      return NULL;
   if (!GPIO_IS_VALID_OUTPUT_GPIO (tx)  //
       || !GPIO_IS_VALID_GPIO (rx)      //
       || !GPIO_IS_VALID_OUTPUT_GPIO (de)       //
       || (re >= 0 && !GPIO_IS_VALID_OUTPUT_GPIO (re))  //
       || (clk >= 0 && !GPIO_IS_VALID_OUTPUT_GPIO (clk))        //
      )
      return NULL;
   galaxybus_t *g = malloc (sizeof (*g));
   if (!g)
      return g;
   memset (g, 0, sizeof (*g));
   g->txpre = 2;                // defaults
   g->txpost = 2;
   g->gap = 10;
   g->de = de;
   g->re = re;
   g->tx = tx;
   g->rx = rx;
   g->clk = clk;
   g->timer = timer;
   g->slave = (slave ? 1 : 0);
   g->address = (slave ? : MASTER);
   g->group = xEventGroupCreate ();
   return g;
}

void
galaxybus_set_timing (galaxybus_t * g, uint32_t pre, uint32_t post, uint32_t gap)
{
   if (pre)
      g->txpre = pre;
   if (post)
      g->txpost = post;
   if (gap)
      g->gap = gap;
}

void
galaxybus_start (galaxybus_t * g)
{
   g->started = 1;
   // GPIO (does nothing for -ve port IDs)
   gpio_clr (g->de);
   gpio_out (g->de);
   gpio_clr (g->re);
   gpio_out (g->re);
   gpio_out (g->clk);
   if (g->tx != g->rx)
      gpio_out (g->tx);
   // Set up timer
   timer_config_t config;
   config.divider = TIMER_DIVIDER;
   config.counter_dir = TIMER_COUNT_UP;
   config.counter_en = TIMER_PAUSE;
   config.alarm_en = TIMER_ALARM_EN;
   config.intr_type = TIMER_INTR_LEVEL;
   config.auto_reload = 1;
   timer_init (TIMER_GROUP_0, g->timer, &config);
   timer_set_counter_value (TIMER_GROUP_0, g->timer, 0x00000000ULL);
   timer_set_alarm_value (TIMER_GROUP_0, g->timer, TIMER_SCALE / 9600 / 3);
   timer_enable_intr (TIMER_GROUP_0, g->timer);
   timer_isr_register (TIMER_GROUP_0, g->timer, timer_isr, g, ESP_INTR_FLAG_IRAM, NULL);
   timer_start (TIMER_GROUP_0, g->timer);
   xEventGroupSetBits (g->group, GROUP_TX_OK);
   xEventGroupSetBits (g->group, GROUP_RX_OK);
   rs485_mode_rx (g);
}

void *
galaxybus_end (galaxybus_t * g)
{
   if (g)
   {
      if (g->started)
      {
         timer_disable_intr (TIMER_GROUP_0, g->timer);
      }
      free (g);
   }
   return NULL;
}

// Low level messaging
int
galaxybus_tx (galaxybus_t * g, int len, uint8_t * data)
{
   if (len >= GALAXYBUSMAX)
      return GALAXYBUSTOOBIG;
   g->txhold = 1;               // Stop sending starting whilst we are loading
   if (!xEventGroupWaitBits (g->group, GROUP_TX_OK, pdFALSE, pdTRUE, 100 / portTICK_PERIOD_MS))
   {
      g->txhold = 0;
      return GALAXYBUSBUSY;
   }
   uint8_t c = 0xAA;
   int p;
   for (p = 0; p < len; p++)
   {
      g->txdata[p] = data[p];
      if ((int) c + data[p] > 0xFF)
         c++;                   // 1 's comp
      c += data[p];
   }
   g->txdata[p++] = c;          // Checksum
   g->txlen = p;
   if (!g->slave)
      g->txdue = 1;             // Send now (if slave we send when polled)
   g->txhold = 0;               // Allow sending
   return len;
}

int
galaxybus_ready (galaxybus_t * g)
{
   if (g->rxdue == g->rxseq)
      return 0;                 // Nothing ready
   return 1;
}

int
galaxybus_rx (galaxybus_t * g, int max, uint8_t * data)
{
   if (!xEventGroupWaitBits (g->group, GROUP_RX_OK, pdFALSE, pdTRUE, 10 / portTICK_PERIOD_MS))
      return 0;
   if (g->rxdue == g->rxseq)
      return 0;                 // Nothing ready
   g->rxdue++;
   if (g->rxdue != g->rxseq)
      return GALAXYBUSMISSED;   // Missed one
   if (!g->rxlen)
      return 0;                 // Uh?
   if (g->rxerrorreport)
      return g->rxerrorreport;  // Bad rx
   if (g->rxlen > max)
      return GALAXYBUSTOOBIG;   // No space
   int p;
   for (p = 0; p < g->rxlen - 1; p++)
      data[p] = g->rxdata[p];
   if (g->rxpos || g->rxdue != g->rxseq)
      return GALAXYBUSMISSED;   // Missed one whilst reading data !
   return p;
}
