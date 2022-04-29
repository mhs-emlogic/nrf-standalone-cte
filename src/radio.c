
#include "radio.h"

#include <string.h>

#include "nrf52833.h"
#include "nrf52833_bitfields.h"


/*

We want to send a LE Uncoded PHY packet (bluetooth spec 5.1, vol 6, part B, section 2.1) containing advertising data.

We are sending ADV_NONCONN_IND, because that is recognized by both the InsightSIP antenna board and the nRF-connect app
on my phone. The format of these packets is equal to ADV_SCAN_IND (which is what the InsightSiP dongle sends) and ADV_IND.

We are transmitting at 1Mbit/s, which is required for ADV_NONCONN_IND packets. See bluetooth spec 5.1, vol 6, part B, table 2.3.

The full transmission consists of:
  - Preamble
      1 byte, either 01010101 or 10101010.
      Last bit shall be oposite of first bit of access address).
      The RADIO peripheral automatically sends this.
  - Access address
      4 bytes.
      Must be 0x8e89bed6 for advertising packets, see bluetooth spec 5.1, vol 6, part B, section 2.1.2.
      This is set in the BASE and PREFIX registers of the RADIO peripheral.
  - Packet PDU
      Contains advertising packet.
      Data is passed to RADIO peripheral via DMA through the PACKETPTR register.
      - Header (refered to as S0 and LENGTH bytes by RADIO peripheral)
        - PDU Type (4 bits, 0110b = ADV_SCAN_IND, 0000b = ADV_IND, etc.)
        - RFU (1 bit, not sure what this is...)
        - ChSel (1 bit, unused, should be 0)
        - TxAdd (1 bit, TxAdd=1 if AdvA is a random address (which the dongle address is))
        - RxAdd (1 bit, unused, should be 0)
        - Length (8 bits, length of remaining payload in bytes)
      - Payload
        - AdvA
            6 bytes.
            The InsightSiP dongle uses c6:05:04:03:02:01 (which is transmitted little endian, so the 0x01 byte goes first).
            RadioAdvertisement.address configures this.
        - AdvData
            Up to 31 bytes.
            Described in bluetooth spec 5.1, vol 3, part C, section 11.
            RadioAdvertisement.payload configures this.
  - CRC
      3 bytes.
      Only covers the packet PDU.
      We configure the RADIO peripheral to compute and transmit this.
  - CTE
      Optional, between 16µs and 160µs when present.

*/

void
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
  // Note (Morten, 2022-04-27) My understanding is that TIFS only matters if I try to send multiple packets consecutively at the same frequency, which I don't do here.

  // Set access address
  NRF_RADIO->BASE0   = 0x89bed600; // See bluetooth spec 5.1, vol 6, part B, section 2.1.2.
  NRF_RADIO->PREFIX0 = 0x0000008e;
  NRF_RADIO->TXADDRESS = 0; // Use logical address zero when sending, made up of BASE0 and PREFIX0.AP0

  // Configure packet data format, see comment at start of file.
  NRF_RADIO->PCNF0 =
    (1 << RADIO_PCNF0_S0LEN_Pos) |
    (8 << RADIO_PCNF0_LFLEN_Pos) |
    (0 << RADIO_PCNF0_S1LEN_Pos) |
    (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos) |
    (RADIO_PCNF0_CRCINC_Exclude << RADIO_PCNF0_CRCINC_Pos);

  int max_packet_len = 2 + 6 + RadioAdvertisement_MAX_PAYLOAD_LEN; // This is just used as an upper limit for DMA.
  NRF_RADIO->PCNF1 =
    ((max_packet_len << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |
    (3 << RADIO_PCNF1_BALEN_Pos) | // 3 bytes base address + 1 byte prefix gives 4 bytes of access address
    (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
    (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos); // Whitening is also configured just before sending because it is frequency dependent
}


static void
radio_transmit(int channel_index)
{
  int frequency = 0; // MHz relative to 2.4GHz
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

  // See section 6.18.5, "Radio states", of nRF52833 product sheet.
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_PHYEND_DISABLE_Msk;
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_TXEN = 1;
  while (!NRF_RADIO->EVENTS_DISABLED);
}

void
radio_advertise(struct RadioAdvertisement advertisement)
{
  if (advertisement.payload_len < 0 || advertisement.payload_len > RadioAdvertisement_MAX_PAYLOAD_LEN) {
    return; // TODO report error to user somehow?
  }

  // Set up packet data for DMA.
  uint8_t buffer[2 + 6 + RadioAdvertisement_MAX_PAYLOAD_LEN]; // 2 bytes for header, 6 for address
  buffer[0] =
    (2 << 0) | // PDU Type = ADV_NONCONN_IND, both the AoA board and the nRF Connect app on my phone recognize this.
    (0 << 4) | // RFU, not sure what this is for
    (0 << 5) | // ChSel bit
    (1 << 6) | // TxAdd bit, set to 1 because AdvA is a random address. See bluetooth spec 5.1, vol 6, part B, section 2.3.1.4.
    (0 << 7);  // RxAdd bit
  buffer[1] = 6 + advertisement.payload_len;
  memcpy(&buffer[2], advertisement.address, 6);
  memcpy(&buffer[8], advertisement.payload, advertisement.payload_len);

  NRF_RADIO->PACKETPTR = (uint32_t) (void *) buffer;

  // Configure DFE/CTE.
  if (advertisement.cte_length > 0) {
    NRF_RADIO->DFEMODE = RADIO_DFEMODE_DFEOPMODE_AoA;
    NRF_RADIO->CTEINLINECONF = RADIO_CTEINLINECONF_CTEINLINECTRLEN_Disabled;
    NRF_RADIO->DFECTRL1 =
      ((advertisement.cte_length << RADIO_DFECTRL1_NUMBEROF8US_Pos) & RADIO_DFECTRL1_NUMBEROF8US_Msk) |
      (RADIO_DFECTRL1_DFEINEXTENSION_CRC << RADIO_DFECTRL1_DFEINEXTENSION_Pos); // Place CTE after CRC
  } else {
    NRF_RADIO->DFEMODE = RADIO_DFEMODE_DFEOPMODE_Disabled;
  }

  // Note (Morten, 2022-04-27) My understanding is that it is normal for bluetooth devices to advertise on all three primary
  // advertising channels (37, 38, 39) in sequence each advertising interval, since scanners only can listen to one channel at the time.
  // If an advertiser only advertises on one channel per interval the chance of the packet being seen by a scanner drops to 1/3.
  // I'm not sure how much of advertising/scanning channel selection is convention-based, since the bluetooth spec says:
  //    "There are no strict timing or advertising channel index selection rules for scanning" (spec 5.1, vol 6, part B, section 4.4.3)
  radio_transmit(37);
  radio_transmit(38);
  radio_transmit(39);
}