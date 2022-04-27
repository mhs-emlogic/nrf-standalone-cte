
This code is supposed to emulate the tag board shipped with the AoA demo kit from InsightSIP.
It is intended to run on Nordics nRF52833 devkit.

The code sends a ADV_SCAN_IND with no payload but with CTE enabled on BLE channels 37, 38 and 39 once every 200ms.

Compiled with SEGGER embedded studio 6.22a using the nRF CPU support package version 8.44.