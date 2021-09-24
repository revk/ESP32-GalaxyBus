// Galaxy RS485 bus
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0
//static const char TAG[] = "galaxybus";

#include "galaxybus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <driver/timer.h>
#include <driver/gpio.h>

#define MASTER 0x11

struct galaxybus_s {
   int8_t timer;                // Which timer
   int8_t tx;                   // Tx pin
   int8_t rx;                   // Rx pin (can be same as tx)
   int8_t de;                   // DE pin
   int8_t re;                   // RE pin (optional, can be same as DE)
   int8_t clk;                  // Debug pin (optional)
   uint8_t address;             // Us
   uint8_t txpre;               // Pre tx drive(high) (bits)
   uint8_t txpost;              // Post tx drive(high) (bits)
   uint8_t rxpre;               // Pre rx gap (bits)
   uint8_t rxpost;              // Post rx gap (bits) for end of message
   uint8_t gap;                 // Pre/post gap count
   uint8_t txlen;               // The length of tx buf
   volatile uint8_t txpos;      // Position in tx buf we are sending(checked by app level)
   uint8_t txdata[GALAXYBUSMAX];        // The tx message
   uint8_t rxerr;               //  The current in progress message rx error
   uint8_t rxerrorreport;       // The rxerror of the last stored message
   uint8_t rxpos;               // Where we are in rx buf
   uint8_t rxlen;               // Length of last received mesage in rxbuf
   uint8_t rxdue;               // The expected message sequence
   uint8_t rxsum;               // The current checksum
   volatile uint8_t rxseq;      // The last received message sequence
   uint8_t rxdata[GALAXYBUSMAX];        // The Rx data
   uint8_t subbit;              // Sub bit count
   uint8_t bit;                 // Bit count
   uint8_t shift;               // Byte
    uint8_t:0;                  //      Bits set from int
   uint8_t txrx:1;              // Mode, true for rx , false for tx //Tx
   uint8_t rxignore:1;          // This message is not for us so being ignored
   uint8_t tick:2;              // clk tick
   uint8_t rxbrk:1;             // rx break condition
    uint8_t:0;                  //      Bits set from non int
   uint8_t slave:1;             // We are slave
   uint8_t started:1;           // Int handler started
   uint8_t txhold:1;            // We are in app Tx call and need to hold of sending as copying to buffer
   volatile uint8_t txdue:1;    // We are due to send a message
};

#define TIMER_DIVIDER         4 //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

#define	GROUP_RX_OK	1       // Rx is not busy
#define	GROUP_TX_OK	2       // Tx is not busy
#define	GROUP_RX_READY	4       // New Rx message ready


static const char *const galaxybus_err_str[GALAXYBUS_ERR_MAX + 1] = {
#define p(n) [GALAXYBUS_ERR_##n]="GALAXYBUS_ERR_"#n,
#define s(v,n) [GALAXYBUS_ERR_STATUS_##n]="GALAXYBUS_ERR_STATUS_"#n,
   galaxybus_errs
#undef p
#undef s
};


// Low level direct GPIO controls - inlines were not playing with some optimisation modes
#define gpio_in(r) do{if ((r) >= 32)GPIO_REG_WRITE(GPIO_ENABLE1_W1TC_REG, 1 << ((r) - 32));else if ((r) >= 0)GPIO_REG_WRITE(GPIO_ENABLE_W1TC_REG, 1 << (r));}while(0)
#define gpio_out(r) do{if ((r) >= 32)GPIO_REG_WRITE(GPIO_ENABLE1_W1TS_REG, 1 << ((r) - 32)); else if ((r) >= 0)GPIO_REG_WRITE(GPIO_ENABLE_W1TS_REG, 1 << (r));}while(0)
#define gpio_set(r) do{if ((r) >= 32)GPIO_REG_WRITE(GPIO_OUT1_W1TS_REG, 1 << ((r) - 32)); else if ((r) >= 0)GPIO_REG_WRITE(GPIO_OUT_W1TS_REG, 1 << (r));}while(0)
#define gpio_clr(r) do{if ((r) >= 32)GPIO_REG_WRITE(GPIO_OUT1_W1TC_REG, 1 << ((r) - 32));else if ((r) >= 0)GPIO_REG_WRITE(GPIO_OUT_W1TC_REG, 1 << (r));}while(0)
#define gpio_get(r) (((r) >= 32)?((GPIO_REG_READ(GPIO_IN1_REG) >> ((r) - 32)) & 1):((r) >= 0)?((GPIO_REG_READ(GPIO_IN_REG) >> (r)) & 1):0)
#define rs485_mode_rx(g) do{if ((g)->tx == (g)->rx)gpio_in((g)->rx);gpio_clr((g)->de);(g)->rxerr = 0;(g)->gap = (g)->rxpre;(g)->txrx = 1;}while(0)
#define rs485_mode_tx(g) do{gpio_set((g)->de);if ((g)->tx == (g)->rx)gpio_out((g)->rx);(g)->gap = (g)->txpre;(g)->txdue = 0;(g)->txrx = 0;}while(0)

bool IRAM_ATTR timer_isr(void *gp)
{
   galaxybus_t *g = gp;
   if (g->clk >= 0)
   {
      if (g->tick++ == 2)
         g->tick = 0;
      if (g->tick == 0 || (g->tick == 1 && g->txrx))
         gpio_clr(g->clk);
      else
         gpio_set(g->clk);
   }
   if (g->txrx)
   {                            // Rx
      uint8_t v = gpio_get(g->rx);
      if (v && g->rxbrk)
         g->rxbrk = 0;          // Not a break condition
      if (!v && !g->bit)
      {                         // Idle, and low, so this is start of start bit
         g->subbit = 1;         // Centre of start bit (+/- 1/6 of a bit)
         g->bit = 10;
      }
      if (g->subbit--)
         return false;
      g->subbit = 2;            // Three sub bits per bit
      if (!g->bit)
      {                         // Idle
         if (g->gap)
            g->gap--;
         else
         {                      // End of rx
            g->rxignore = 0;
            char send = g->txdue;
            if (g->rxpos)
            {                   // Message received
               if (g->rxsum != g->rxdata[g->rxpos - 1])
                  g->rxerr = GALAXYBUS_ERR_CHECKSUM;
               g->rxlen = g->rxpos;
               g->rxerrorreport = g->rxerr;
               g->rxerr = 0;
               g->rxseq++;
               if (g->slave)
                  send = 1;     // Send reply as we are slave
               else
                  gpio_set(g->de);      // Take bus anyway as we are master - saves it idling while task thinks about what to do next
               g->rxpos = 0;    // ready for next message
            }
            if (send)
               rs485_mode_tx(g);        // Can start tx now
         }
         return false;
      }
      g->bit--;
      if (g->bit == 9)
      {                         // Start bit
         if (v)
         {                      // Missing start bit
            g->rxerr = GALAXYBUS_ERR_STARTBIT;
            g->bit = 0;         // Back to idle
         }
         return false;
      }
      if (g->bit)
      {                         // Shift in
         g->shift >>= 1;
         if (v)
            g->shift |= 0x80;
         return false;
      }
      // Stop bit
      if (!v)
      {
         g->rxerr = GALAXYBUS_ERR_STOPBIT;
         if (!g->shift)
            g->rxbrk = 1;       // Break condition
      }
      g->gap = g->rxpost;       // Look for end of message
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
      {
         g->rxerr = 0;
         return false;          // Ignore zero leading
      }
      if (!g->rxpos && g->shift != g->address && g->address != 0xFF && g->shift != 0xFF)
         g->rxignore = 1;       // Not addressed to us, ignore
      if (g->rxignore)
         return false;          // Not for us
      // End of byte
      if (g->rxpos >= GALAXYBUSMAX)
         g->rxerr = GALAXYBUS_ERR_TOOBIG;
      else
         g->rxdata[g->rxpos++] = g->shift;
      return false;
   }
   // Tx
   if (g->subbit--)
      return false;
   g->subbit = 2;               // Three sub bits per bit
   uint8_t t = 1;
   if (g->gap)
   {
      t = 1;
      g->gap--;
      if (!g->gap)
      {                         // End of gap
         if (g->txpos)
         {                      // End of message
            g->txpos = 0;
            rs485_mode_rx(g);   // Switch back to rx
            return false;
         }
         // Start of message
         if (g->txhold)
            g->gap++;           // Wait, app is writing new message
         else
         {                      // Start sending
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
         g->gap = g->txpost;    // End of message
   }
   if (t)
      gpio_set(g->tx);
   else
      gpio_clr(g->tx);
   return false;
}

// Set up
galaxybus_t *galaxybus_init(int8_t timer, int8_t tx, int8_t rx, int8_t de, int8_t re, int8_t clk, uint8_t slave)
{
   if (timer < 0 || tx < 0 || rx < 0 || de < 0 || re < 0 || tx == de || rx == de || tx == re || rx == re)
      return NULL;
   if (!GPIO_IS_VALID_OUTPUT_GPIO(tx)   //
       || !GPIO_IS_VALID_GPIO(rx)       //
       || !GPIO_IS_VALID_OUTPUT_GPIO(de)        //
       || (re >= 0 && !GPIO_IS_VALID_OUTPUT_GPIO(re))   //
       || (clk >= 0 && !GPIO_IS_VALID_OUTPUT_GPIO(clk)) //
       )
      return NULL;
   galaxybus_t *g = malloc(sizeof(*g));
   if (!g)
      return g;
   memset(g, 0, sizeof(*g));
   g->txpre = 10;               // defaults
   g->txpost = 10;
   g->rxpre = 50;
   g->rxpost = 10;
   g->de = de;
   g->re = re;
   g->tx = tx;
   g->rx = rx;
   g->clk = clk;
   g->timer = timer;
   g->slave = (slave ? 1 : 0);
   g->address = (slave ? : MASTER);
   return g;
}

void galaxybus_set_timing(galaxybus_t * g, uint8_t txpre, uint8_t txpost, uint8_t rxpre, uint8_t rxpost)
{
   if (txpre)
      g->txpre = txpre;
   if (txpost)
      g->txpost = txpost;
   if (rxpre)
      g->rxpre = rxpre;
   if (rxpost)
      g->rxpost = rxpost;
}

void galaxybus_start(galaxybus_t * g)
{
   g->started = 1;
   // Tx
   gpio_reset_pin(g->tx);
   gpio_set(g->tx);
   gpio_set_direction(g->tx, GPIO_MODE_OUTPUT);
   // DE
   gpio_reset_pin(g->de);
   gpio_clr(g->de);
   gpio_set_direction(g->de, GPIO_MODE_OUTPUT);
   if (g->de != g->re && g->re >= 0)
   {  // If RE is separate, set RE permanently
      gpio_reset_pin(g->re);
      gpio_clr(g->re);
      gpio_set_direction(g->re, GPIO_MODE_OUTPUT);
   }
   if (g->clk >= 0)
   { // Option clock output
      gpio_reset_pin(g->clk);
      gpio_set_direction(g->clk, GPIO_MODE_OUTPUT);
   }
   if (g->tx != g->rx)
   { // If Tx and Rx separate, then set Rx input
      gpio_reset_pin(g->rx);
      gpio_set_direction(g->rx, GPIO_MODE_INPUT);
   }
   // Set up timer
   timer_config_t config;
   config.divider = TIMER_DIVIDER;
   config.counter_dir = TIMER_COUNT_UP;
   config.counter_en = TIMER_PAUSE;
   config.alarm_en = TIMER_ALARM_EN;
   config.intr_type = TIMER_INTR_LEVEL;
   config.auto_reload = 1;
   rs485_mode_rx(g);
   timer_init(0, g->timer, &config);
   timer_set_counter_value(0, g->timer, 0x00000000ULL);
   timer_set_alarm_value(0, g->timer, TIMER_SCALE / 9600 / 3);
   timer_isr_callback_add(0, g->timer, timer_isr, g, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM);
   timer_enable_intr(0, g->timer);
   timer_start(0, g->timer);
}

void *galaxybus_end(galaxybus_t * g)
{
   if (g)
   {
      if (g->started)
      {
         timer_disable_intr(0, g->timer);
      }
      free(g);
   }
   return NULL;
}

// Low level messaging
int galaxybus_tx(galaxybus_t * g, int len, uint8_t * data)
{
   if (len >= GALAXYBUSMAX)
      return -GALAXYBUS_ERR_TOOBIG;
   if (g->rxbrk)
      return -GALAXYBUS_ERR_BREAK;      // Can't Tx if BREAK stuck
   g->txhold = 1;               // Stop sending starting whilst we are loading
   int try = 0;
   while (g->txpos || g->txdue)
   {
      usleep(1000);
      if (try++ > 1000) break; // Should not take this long
   }
   if (g->txpos)
   {
      g->txhold = 0;
      return -GALAXYBUS_ERR_BUSY;
   }
   if (g->txdue)
   {
      g->txhold = 0;
      return -GALAXYBUS_ERR_PENDING;
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

int galaxybus_ready(galaxybus_t * g)
{
   if (g->rxdue == g->rxseq)
      return 0;                 // Nothing ready
   return 1;
}

int galaxybus_rx(galaxybus_t * g, int max, uint8_t * data)
{
   if (g->rxdue == g->rxseq)
   {
      if (g->rxbrk)
         return -GALAXYBUS_ERR_BREAK;   // Nothing ready, but we are in a break condition, so something wrong
      return 0;                 // Nothing ready
   }
   g->rxdue++;
   if (g->rxdue != g->rxseq)
      return -GALAXYBUS_ERR_MISSED;     // Missed one
   if (!g->rxlen)
      return 0;                 // Uh?
   if (g->rxerrorreport)
      return -g->rxerrorreport; // Bad rx
   if (g->rxlen > max)
      return -GALAXYBUS_ERR_TOOBIG;     // No space
   int p;
   for (p = 0; p < g->rxlen - 1; p++)
      data[p] = g->rxdata[p];
   if (g->rxpos || g->rxdue != g->rxseq)
      return -GALAXYBUS_ERR_MISSED;     // Missed one whilst reading data !
   return p;
}

const char *galaxybus_err_to_name(int e)
{
   if (e < 0)
      e = 0 - e;
   if (e > GALAXYBUS_ERR_MAX)
      return "GALAXYBUS_ERR_UNKNOWN";
   return galaxybus_err_str[e];
}
