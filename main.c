
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

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



enum { MAC_STRING_LENGTH = 18 };

static void
mac_to_string(uint8_t mac[6], char string[MAC_STRING_LENGTH])
{
  const char *HEX = "0123456789abcdef";
  for (int i = 0; i < 5; ++i) {
    string[3*i + 2] = ':';
  }
  for (int i = 0; i < 6; ++i) {
    uint8_t byte = mac[5 - i];
    string[3*i + 0] = HEX[byte >> 4];
    string[3*i + 1] = HEX[byte & 0xf];
  }
  string[MAC_STRING_LENGTH - 1] = 0;
}



enum {
  MAX_PACKET_LEN = 64,
  MAX_CTE_SAMPLES = 256,
};

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
  
  NRF_RADIO->TIFS = 150; // See bluetooth spec 5.1, vol 6, part B, section 4.1.1.
  // Note (Morten, 2022-04-27) My understanding is that TIFS only matters if I try to send multiple packets consecutively at the
  // same frequency, which I don't do in this program.

  // Set access address
  NRF_RADIO->BASE0   = 0x89bed600; // See bluetooth spec 5.1, vol 6, part B, section 2.1.2.
  NRF_RADIO->PREFIX0 = 0x0000008e;
  NRF_RADIO->TXADDRESS = 0; // Use logical address zero when sending, made up of BASE0 and PREFIX0.AP0
  NRF_RADIO->RXADDRESSES = 1; // Only receive packets with logical address zero

  // Configure DFE/CTE
  int cte_length_in_8us = 20; // must be >=2 and <=20 corresponding to >=16µs and <=160µs

  NRF_RADIO->DFEMODE = RADIO_DFEMODE_DFEOPMODE_AoA;
  NRF_RADIO->CTEINLINECONF =
    RADIO_CTEINLINECONF_CTEINLINECTRLEN_Disabled;
  NRF_RADIO->DFECTRL1 =
    ((cte_length_in_8us << RADIO_DFECTRL1_NUMBEROF8US_Pos) & RADIO_DFECTRL1_NUMBEROF8US_Msk) | // TODO is this even considered during reception??
    (RADIO_DFECTRL1_DFEINEXTENSION_CRC << RADIO_DFECTRL1_DFEINEXTENSION_Pos) |
    (RADIO_DFECTRL1_TSWITCHSPACING_1us << RADIO_DFECTRL1_TSWITCHSPACING_Pos) | // Switch antenna every 1µs.
    (RADIO_DFECTRL1_TSAMPLESPACINGREF_1us << RADIO_DFECTRL1_TSAMPLESPACINGREF_Pos) | // In reference period, capture a sample every 1µs.
    (RADIO_DFECTRL1_SAMPLETYPE_IQ << RADIO_DFECTRL1_SAMPLETYPE_Pos) | // Capture IQ samples.
    (RADIO_DFECTRL1_TSAMPLESPACING_1us << RADIO_DFECTRL1_TSAMPLESPACING_Pos) | // Sample every 1µs after the reference period.
    (RADIO_DFECTRL1_REPEATPATTERN_Msk); // Repeat pattern as many times as possible/needed.
  NRF_RADIO->DFECTRL2 = // TODO not sure how to configure these values...
    (0 << RADIO_DFECTRL2_TSWITCHOFFSET_Pos) |
    (0 << RADIO_DFECTRL2_TSAMPLEOFFSET_Pos);
  for (int i = 0; i < 3; ++i) NRF_RADIO->SWITCHPATTERN = 0; // One entry for normal operation, one for reference period, one for each switch (this is repeated, doesn't matter because we have just one antenna).
  NRF_RADIO->DFEPACKET.MAXCNT = MAX_CTE_SAMPLES;

  // Configure packet data format
  NRF_RADIO->PCNF0 =
    (1 << RADIO_PCNF0_S0LEN_Pos) |
    (8 << RADIO_PCNF0_LFLEN_Pos) |
    (0 << RADIO_PCNF0_S1LEN_Pos) |
    (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos) |
    (RADIO_PCNF0_CRCINC_Exclude << RADIO_PCNF0_CRCINC_Pos);
  NRF_RADIO->PCNF1 =
    ((MAX_PACKET_LEN << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |
    (3 << RADIO_PCNF1_BALEN_Pos) | // 3 bytes base address + 1 byte prefix gives 4 bytes of access address
    (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
    (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos); // Whitening is also configured just before sending because it is frequency dependent
}


static void
radio_set_channel(int channel_index)
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

  NRF_RADIO->FREQUENCY = frequency;
  NRF_RADIO->DATAWHITEIV = channel_index; // See bluetooth spec 5.1, vol 6, part B, section 3.2.
}


static uint32_t g_cte_buffer[MAX_CTE_SAMPLES];

static void
radio_receive(int channel_index)
{
  radio_set_channel(channel_index);

  uint8_t receive_buffer[2 + MAX_PACKET_LEN]; // allocate extra space for S0 byte and LEN byte
  memset(receive_buffer, 0, sizeof(receive_buffer));
  NRF_RADIO->PACKETPTR = (uint32_t) (void *) receive_buffer;

  memset(g_cte_buffer, 0, sizeof(g_cte_buffer));
  NRF_RADIO->DFEPACKET.PTR = (uint32_t) (void *) g_cte_buffer;

  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_PHYEND_DISABLE_Msk; // END triggers at start of CTE, PHYEND at end of CTE, so we need PHYEND here!

  // I guess you normally want to have a timeout here in case there actually is no traffic.
  int wait_count = 0;
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_RXEN = 1;
  while (!NRF_RADIO->EVENTS_DISABLED) ++wait_count;

  uint8_t dongle_mac[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0xc6 };
  if (NRF_RADIO->CRCSTATUS &&
      receive_buffer[0] == 0x46 &&
      receive_buffer[1] == 6 &&
      memcmp(dongle_mac, receive_buffer + 2, 6) == 0)
  {
    char address_string[MAC_STRING_LENGTH];
    mac_to_string(receive_buffer + 2, address_string);

    int cte_count = NRF_RADIO->DFEPACKET.AMOUNT;
    printf("%s sent %i cte samples:\n", address_string, cte_count);
    for (int i = 0; i < cte_count; ++i) {
      if (i == 0) printf("reference:\n");
      if (i == 8) printf("data:\n");
      int sample_i = (int) (int16_t) (uint16_t) (g_cte_buffer[i] & 0xffff);
      int sample_q = (int) (int16_t) (uint16_t) ((g_cte_buffer[i] >> 16) & 0xffff);
      printf(" %i,%i,%i\n", i, sample_i, sample_q);
    }
  }
}


static void
radio_send(int channel_index)
{
  radio_set_channel(channel_index);

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
        (6 << 0) | // PDU Type = ADV_SCAN_IND, only the InsightSIP AoA board recognizes this.
        //(2 << 0) | // PDU Type = ADV_NONCONN_IND, both the AoA board and the nRF Connect app on my phone recognize this.
        (0 << 4) | // RFU, not sure what this is for
        (0 << 5) | // ChSel bit
        (1 << 6) | // TxAdd bit, set to 1 because AdvA is a random address. See bluetooth spec 5.1, vol 6, part B, section 2.3.1.4.
        (0 << 7),  // RxAdd bit
    .length = 6,
    .advertiser_address = { 0x01, 0x02, 0x03, 0x04, 0x05, 0xc6 }, // Same as the InsightSIP AoA dongle.
  };

  NRF_RADIO->PACKETPTR = (uint32_t) (void *) &pdu;


  // See section 6.18.5, "Radio states", of nRF52833 product sheet, note the shortcuts configured above in the SHORTS register.

  //NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_PHYEND_DISABLE_Msk;

  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_TXEN = 1;
  while (!NRF_RADIO->EVENTS_DISABLED);
}



#define LED0    NRF_P0, 13

int
main(void)
{
  // Note (Morten, 2022-04-27) If this is commented out, you have to mass-erase the board to disable it. Until then, debugging and flashing is disabled.
  /*if ((NRF_UICR->APPROTECT & 0xff) != 0x00) {
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
    NRF_UICR->APPROTECT = 0x00;
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
    __NVIC_SystemReset();
  }*/

  systick_init();
  radio_init();
  gpio_make_output(LED0);

  #if 0
  while (1) {
    radio_receive(37);
  }
  #endif

  #if 1
  while (1) {
    // Note (Morten, 2022-04-27) The waits here control the advertising interval. The interval is supposed to be a multiple of
    // 0.625ms, but I think that only is relevant when writing the HCI part of a bluetooth stack.
    // See bluetooth spec 5.1, vol 6, part B, section 4.4.2.2.1.
    gpio_write(LED0, 0); // LED is active low!
    busy_wait_ms(140);
    gpio_write(LED0, 1);
    busy_wait_ms(60);

    // Note (Morten, 2022-04-27) My understanding is that it is normal for bluetooth devices to advertise on all three primary
    // advertising channels (37, 38, 39) in sequence each advertising interval, since scanners only can listen to one channel at the time.
    // If an advertiser only advertises on one channel per interval the chance of the packet being seen by a scanner drops to 1/3.
    // I'm not sure how much of advertising/scanning channel selection is convention-based, since the bluetooth spec says:
    //    "There are no strict timing or advertising channel index selection rules for scanning" (spec 5.1, vol 6, part B, section 4.4.3)

    radio_send(37);
    radio_send(38);
    radio_send(39);
  }
  #endif

  return 0;
}