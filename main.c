
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "nrf52833.h"
#include "nrf52833_bitfields.h"


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




static void
radio_init(void)
{
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED);

  // Power-cycle to be sure registers are zeroed
  NRF_RADIO->POWER = 0;
  NRF_RADIO->POWER = 1;

  NRF_RADIO->TXPOWER = 0;
  NRF_RADIO->MODE = RADIO_MODE_MODE_Ble_1Mbit; // See bluetooth spec 5.1, vol 6, part B, section 2.3, table 2.3
  
  // Configure CRC
  // See bluetooth spec 5.1, vol 6, part B, section 3.1.1.
  // See bluetooth spec 5.1, vol 6, part B, section 2.1.3.
  NRF_RADIO->CRCCNF =
    (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
    (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
  NRF_RADIO->CRCPOLY = 0x00065b;
  NRF_RADIO->CRCINIT = 0x555555;
  
  NRF_RADIO->TIFS = 150; // TODO not sure if this is necessary...

  // Set access address
  NRF_RADIO->BASE0   = 0x89bed600; // See bluetooth spec 5.1, vol 6, part B, section 2.1.2.
  NRF_RADIO->PREFIX0 = 0x0000008e;
  NRF_RADIO->TXADDRESS = 0; // Use logical address zero, made up of BASE0 and PREFIX0.AP0

  // Configure CTE
  int cte_length_in_8us = 20; // must be >=2 and <=20 corresponding to >=16µs and <=160µs
  NRF_RADIO->DFEMODE = RADIO_DFEMODE_DFEOPMODE_AoA;
  NRF_RADIO->CTEINLINECONF = RADIO_CTEINLINECONF_CTEINLINECTRLEN_Disabled;
  NRF_RADIO->DFECTRL1 =
    ((cte_length_in_8us << RADIO_DFECTRL1_NUMBEROF8US_Pos) & RADIO_DFECTRL1_NUMBEROF8US_Msk) |
    (RADIO_DFECTRL1_DFEINEXTENSION_CRC << RADIO_DFECTRL1_DFEINEXTENSION_Pos) |
    (RADIO_DFECTRL1_TSWITCHSPACING_2us << RADIO_DFECTRL1_TSWITCHSPACING_Pos) | // I don't think this matters because we are not doing antenna switching
    (RADIO_DFECTRL1_TSAMPLESPACINGREF_1us << RADIO_DFECTRL1_TSAMPLESPACINGREF_Pos) | // ditto
    (RADIO_DFECTRL1_TSAMPLESPACING_1us << RADIO_DFECTRL1_TSAMPLESPACING_Pos); // ditto

  // Configure packet data format
  NRF_RADIO->PCNF0 =
    (1 << RADIO_PCNF0_S0LEN_Pos) |
    (8 << RADIO_PCNF0_LFLEN_Pos) |
    (0 << RADIO_PCNF0_S1LEN_Pos) |
    (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos) |
    (RADIO_PCNF0_CRCINC_Exclude << RADIO_PCNF0_CRCINC_Pos);
  NRF_RADIO->PCNF1 =
    ((255 << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |
    (3 << RADIO_PCNF1_BALEN_Pos) | // 3 bytes base address + 1 byte prefix gives 4 bytes of access address
    (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
    (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos); // Whitening is also configured just before sending because it is frequency dependent

  // Configure shortcuts to simplify the transmit sequence
  NRF_RADIO->SHORTS =
    RADIO_SHORTS_READY_START_Msk |
    RADIO_SHORTS_END_DISABLE_Msk;
}


static void
radio_send(int channel_index)
{
  int frequency = 0; // 2.4GHz + frequency*1MHz
  if (0 <= channel_index && channel_index <= 10) {
    frequency = 4 + 2*channel_index;
  } else if (11 <= channel_index && channel_index <= 36) {
    frequency = 28 + 2*(channel_index - 11);
  } else if (channel_index == 37) {
    frequency = 2;
  } else if (channel_index == 38) {
    frequency = 26;
  } else if (channel_index == 39) {
    frequency = 80;
  }

  // Configure transmitter parameters
  NRF_RADIO->FREQUENCY = frequency;
  NRF_RADIO->DATAWHITEIV = channel_index; // See bluetooth spec 5.1, vol 6, part B, section 3.2.

  /*

  The packet I want to send:
    LE UNCODED PHY DATA
      Preamble (1 byte when 1M, 01010101 or 10101010, last bit shall be oposite of first bit of access address)
      Access address (4 bytes, wild rules)
      Advertising physical channel PDU, see bluetooth spec 5.1, vol 6, part B, section 2.3, table 2.3
        Header
          PDU Type (4 bits, 0110b = ADV_SCAN_IND, 0000b = ADV_IND, etc.)
          RFU (1 bit, not sure what this is...)
          ChSel (1 bit, unused, should be 0)
          TxAdd (1 bit, TxAdd=1 if AdvA is a random address)
          RxAdd (1 bit, unused, should be 0)
          Length (8 bits, length of remaining payload in bytes)
        Payload
          AdvA (6 bytes, C6:05:04:03:02:01, endiannes?)
          AdvData (0 bytes)
      CRC (3 bytes)
      CTE (160µs)

  The RADIO peripheral has some half-structured way of transmitting data:
    PREAMBLE  (0x55 or 0xaa chosen automatically based on address, size depends on MODE register)

    BLE access address gets split across two fields:
      BASE      (length controlled by PCNF1.BALEN)
      PREFIX    (1 byte)

    The next bytes are read from PACKETPTR via DMA
      S0        (configured via S0LEN, use for first 8 bits of the header)
      LENGTH    (configured via LFLEN, used for length field. RADIO reads this value from PACKETPTR and uses it to decide how long PAYLOAD should be)
      S1        (configured via S1LEN to not be used)
      PAYLOAD

    CRC       (Automatically computed and transmitted)

  */

  // See bluetooth spec 5.1, vol 6, part B, section 2.3, figure 2.5
  struct AdvPdu
  {
    uint8_t header;
    uint8_t length;
    uint8_t advertiser_address[6]; // AdvA
    // no AdvData
  };
  struct AdvPdu pdu = {
    .header =
        //(6 << 0) | // PDU Type = ADV_SCAN_IND
        (2 << 0) | // PDU Type = ADV_NONCONN_IND
        (0 << 4) | // RFU, not sure what this is for
        (0 << 5) | // ChSel bit
        (1 << 6) | // TxAdd bit, set to 1 because AdvA is a random address. See bluetooth spec 5.1, vol 6, part B, section 2.3.1.4.
        (0 << 7),  // RxAdd bit
    .length = 6,
    .advertiser_address = { 0x01, 0x02, 0x03, 0x04, 0x05, 0xc6 },
  };

  NRF_RADIO->PACKETPTR = (uint32_t) (void *) &pdu;

  // See section 6.18.5, "Radio states", of nRF52833 product sheet, note the shortcuts configured above in the SHORTS register.
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_TXEN = 1;
  while (!NRF_RADIO->EVENTS_DISABLED);
}



#define LED0    NRF_P0, 13

void
main(void)
{
  systick_init();
  radio_init();
  gpio_make_output(LED0);

  printf("Poggers\n");

  while (1) {
    gpio_write(LED0, 0); // LED is active low!
    busy_wait_ms(140);
    gpio_write(LED0, 1);
    busy_wait_ms(60);

    radio_send(37);
    radio_send(38);
    radio_send(39);
  }
}