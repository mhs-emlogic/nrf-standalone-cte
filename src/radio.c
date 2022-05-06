
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


struct AdData
{
  uint8_t header;
  uint8_t length;
  uint8_t address[6];
  uint8_t payload[RadioAdvertisement_MAX_PAYLOAD_LEN];
};

enum {
  AD_HEADER =
    (2 << 0) | // PDU Type = ADV_NONCONN_IND, both the AoA board and the nRF Connect app on my phone recognize this.
    (0 << 4) | // RFU, not sure what this is for
    (0 << 5) | // ChSel bit
    (1 << 6) | // TxAdd bit, set to 1 because AdvA is a random address. See bluetooth spec 5.1, vol 6, part B, section 2.3.1.4.
    (0 << 7)  // RxAdd bit
};
 


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

  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_PHYEND_DISABLE_Msk;

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
  NRF_RADIO->RXADDRESSES = 1;

  // Configure packet data format, see comment at start of file.
  NRF_RADIO->PCNF0 =
    (1 << RADIO_PCNF0_S0LEN_Pos) |
    (8 << RADIO_PCNF0_LFLEN_Pos) |
    (0 << RADIO_PCNF0_S1LEN_Pos) |
    (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos) |
    (RADIO_PCNF0_CRCINC_Exclude << RADIO_PCNF0_CRCINC_Pos);

  // This is just used as an upper limit for DMA, 'header' and 'length' are mapped to S0 and LENGTH, and these are apparently not counted.
  // See nRF52833 PS v1.5, note at the top of page 304.
  int max_packet_len = sizeof(struct AdData) - 2;
  

  NRF_RADIO->PCNF1 =
    ((max_packet_len << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |
    (3 << RADIO_PCNF1_BALEN_Pos) | // 3 bytes base address + 1 byte prefix gives 4 bytes of access address
    (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
    (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos); // Whitening is also configured just before sending because it is frequency dependent

  // Set DFE switching pattern, doesn't matter what it is because we just have one antenna.
  // One entry for normal operation, one for reference period, one for each switch. The last entry is repeated.
  for (int i = 0; i < 3; ++i) NRF_RADIO->SWITCHPATTERN = 0;
}

static void
radio_set_channel(int channel_index)
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
}

static void
radio_transmit()
{
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_TXEN = 1;
  while (!NRF_RADIO->EVENTS_DISABLED);
}

void
radio_advertise(struct RadioAdvertisement advertisement)
{
  if (advertisement.payload_len < 0 || advertisement.payload_len > RadioAdvertisement_MAX_PAYLOAD_LEN) {
    return; // TODO report error to user somehow? I think just clamping the array length is easier
  }

  // Set up packet data for DMA.
  struct AdData buffer;
  buffer.header = AD_HEADER;
  buffer.length = advertisement.payload_len;
  memcpy(buffer.address, advertisement.address, 6);
  memcpy(buffer.payload, advertisement.payload, advertisement.payload_len);

  NRF_RADIO->PACKETPTR = (uint32_t) (void *) &buffer;

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
  radio_set_channel(37);
  radio_transmit();
  radio_set_channel(38);
  radio_transmit();
  radio_set_channel(39);
  radio_transmit();
}


void
radio_receive(struct RadioAdvertisementReception *reception)
{
  radio_set_channel(reception->channel_index);

  struct AdData buffer;
  memset(&buffer, 0, sizeof(buffer));
  NRF_RADIO->PACKETPTR = (uint32_t) (void *) &buffer;

  static uint32_t reception_address;
  reception_address = (uint32_t) (void *) reception;

  // Configure CTE reception
  NRF_RADIO->DFEMODE = RADIO_DFEMODE_DFEOPMODE_AoA;
  NRF_RADIO->CTEINLINECONF = RADIO_CTEINLINECONF_CTEINLINECTRLEN_Disabled;
  NRF_RADIO->DFECTRL1 =
    ((20 << RADIO_DFECTRL1_NUMBEROF8US_Pos) & RADIO_DFECTRL1_NUMBEROF8US_Msk) | // TODO is this even considered during reception??
    (RADIO_DFECTRL1_DFEINEXTENSION_CRC << RADIO_DFECTRL1_DFEINEXTENSION_Pos) |
    (RADIO_DFECTRL1_TSAMPLESPACINGREF_125ns << RADIO_DFECTRL1_TSAMPLESPACINGREF_Pos) | // In reference period, capture a sample every 1µs.
    (RADIO_DFECTRL1_TSWITCHSPACING_1us << RADIO_DFECTRL1_TSWITCHSPACING_Pos) | // Antenna switch frequency, if this is > than sample frequency we get multiple samples per antenna + samples during switching
    (RADIO_DFECTRL1_TSAMPLESPACING_125ns << RADIO_DFECTRL1_TSAMPLESPACING_Pos) |
    (RADIO_DFECTRL1_SAMPLETYPE_IQ << RADIO_DFECTRL1_SAMPLETYPE_Pos); // Capture IQ samples instead of amplitude/phase samples

  NRF_RADIO->DFEPACKET.MAXCNT = reception->cte_cap;
  NRF_RADIO->DFEPACKET.PTR = (uint32_t) (void *) reception->cte;

  // I guess you normally want to have a timeout here in case there actually is no traffic.
  while (1) {
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_ADDRESS = 0;
    NRF_RADIO->EVENTS_PAYLOAD = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->EVENTS_CRCOK = 0;
    NRF_RADIO->EVENTS_CRCERROR = 0;
    NRF_RADIO->EVENTS_FRAMESTART = 0;
    NRF_RADIO->EVENTS_RXREADY = 0;
    NRF_RADIO->EVENTS_MHRMATCH = 0;
    NRF_RADIO->EVENTS_PHYEND = 0;

    //NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while (!NRF_RADIO->EVENTS_DISABLED);

    if (NRF_RADIO->CRCSTATUS &&
      buffer.header == 0x42 && // ADV_NONCONN_IND with TxAdd set to 1, just like in 'radio_advertise' above.
      buffer.header >= sizeof(buffer.address) &&
      memcmp(reception->address, buffer.address, sizeof(buffer.address)) == 0)
    {
      break;
    }
  }

  int payload_len = buffer.length - 6;
  if (payload_len > RadioAdvertisement_MAX_PAYLOAD_LEN) {
    payload_len = RadioAdvertisement_MAX_PAYLOAD_LEN; // TODO can this happen? not sure exactly how the peripheral works...
  }
  reception->payload_len = payload_len;
  memcpy(reception->payload, buffer.payload, payload_len);

  reception->cte_len = NRF_RADIO->DFEPACKET.AMOUNT;
}