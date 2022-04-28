
#include <stdint.h>

// TODO docs for functions
// TODO docs for functions
// TODO docs for functions
// TODO docs for functions
// TODO docs for functions
// TODO docs for functions
// TODO docs for functions
// TODO docs for functions

enum {
  RadioAdvertisement_MAX_PAYLOAD_LEN = 31,
};

struct RadioAdvertisement
{
  uint8_t address[6]; // BLE advertiser address.

  int payload_len;
  uint8_t *payload;

  int cte_length; // In multiples of 8µs. Set to zero to disable, or between 2 and 20 to enable. Other values are not permitted!
};

void radio_init(void);
void radio_advertise(struct RadioAdvertisement advertisement); // Blocks until the full advertisement is sent
