
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "nrf52833.h"
#include "nrf52833_bitfields.h"

#include "radio.h"


static volatile uint32_t systick_counter;

void
SysTick_Handler(void)
{
  ++systick_counter;
}

static void
systick_init()
{
  extern uint32_t SystemCoreClock; // From system_nrf52.c
  SysTick->LOAD = SystemCoreClock/1000 - 1;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void
busy_wait_ms(uint32_t ms)
{
  uint32_t target = systick_counter + ms + 1;
  while ((int32_t) (target - systick_counter) > 0);
}




static void
gpio_make_output(NRF_GPIO_Type *port, int pin)
{
  port->DIRSET = 1 << pin;
}

static void
gpio_write(NRF_GPIO_Type *port, int pin, int value)
{
  if (value) {
    port->OUTSET = 1 << pin;
  } else {
    port->OUTCLR = 1 << pin;
  }
}


#define LED0    NRF_P0, 13

int
main(void)
{
  systick_init();
  radio_init();
  gpio_make_output(LED0);

  while (1) {
    // Note (Morten, 2022-04-27) The waits here control the advertising interval. The interval is supposed to be a multiple of
    // 0.625ms, but I think that only is relevant when writing the HCI part of a bluetooth stack.
    // See bluetooth spec 5.1, vol 6, part B, section 4.4.2.2.1.
    gpio_write(LED0, 0); // LED is active low!
    busy_wait_ms(70);
    gpio_write(LED0, 1);
    busy_wait_ms(30);

    struct RadioAdvertisement cte_ad = {
      .address = { 0x01, 0x02, 0x03, 0x04, 0x05, 0xc6 }, // this is the address the dongle from InsightSiPs AoA demo uses.
      .payload_len = 0,
      .cte_length = 20,
    };
    radio_advertise(cte_ad);
  }

  return 0;
}