
#include <stdint.h>

enum {
  RadioAdvertisement_MAX_PAYLOAD_LEN = 31,
};

struct RadioAdvertisement
{
  uint8_t address[6]; // BLE advertiser address.

  int payload_len; // See RadioAdvertisement_MAX_PAYLOAD_LEN!
  uint8_t *payload; // See bluetooth spec 5.1, vol 3, part C, section 11.

  int cte_length; // In multiples of 8µs. Set to zero to disable, or between 2 and 20 to enable. Other values are not permitted!
};


// Enable RADIO peripheral, configure it for transmission of advertisements.
void radio_init(void);

// Send an advertisement on all three primary advertisement channels (37, 38, 39) sequentially.
// This function only returns after all three transmissions are done.
// The advertisement has packet type ADV_NONCONN_IND.
void radio_advertise(struct RadioAdvertisement advertisement);
