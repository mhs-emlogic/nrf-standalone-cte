
#include <stdint.h>

// Enable RADIO peripheral, configure it for transmission of advertisements.
void radio_init(void);



enum {
  RadioAdvertisement_MAX_PAYLOAD_LEN = 31,
};

struct RadioAdvertisement
{
  uint8_t address[6]; // BLE advertiser address.

  int payload_len;
  uint8_t payload[RadioAdvertisement_MAX_PAYLOAD_LEN]; // See bluetooth spec 5.1, vol 3, part C, section 11.

  int cte_length; // In multiples of 8µs. Set to zero to disable, or between 2 and 20 to enable. Other values are not permitted!
};

// Send an advertisement on all three primary advertisement channels (37, 38, 39) sequentially.
// This function only returns after all three transmissions are done.
// The advertisement has packet type ADV_NONCONN_IND.
void radio_advertise(struct RadioAdvertisement advertisement);


struct IQ
{
  int16_t i __attribute__((aligned(4)));
  int16_t q;

  // Note (Morten, 2022-05-05) I'm not sure whether the alignment attribute is necessary. However, the 'cte' array in 'RadioAdvertisementReception'
  // is written to through DMA, and MAXCNT/AMOUNT are both specified in multiples of 32 bits, which makes me believe that this struct also should
  // be aligned on 32 bits.
  // Note (Morten, 2022-05-06) In practice this also seems to work without the alignment attribute, even when I forcefully try to misalign the IQ array...
};

struct RadioAdvertisementReception
{
  // Used to filter advertisements
  int channel_index;
  uint8_t address[6];

  int payload_len; // Set during reception
  uint8_t payload[RadioAdvertisement_MAX_PAYLOAD_LEN]; // Filled during reception

  // TODO CTE sampling parameters!

  int cte_cap; // This, as well as 'cte' has to be set before calling the receive function
  int cte_len; // Set during reception
  struct IQ *cte; // Filled during reception
};

// TODO This is a terrible API, if this is actually going to be used we want to use interrupts so we can do other work while receiving packets.
// At that point, we are moving in the direction of writing a full BLE stack, which I don't really want to be in the business of doing...
void radio_receive(struct RadioAdvertisementReception *reception);