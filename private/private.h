#define ABP

// testdevice
// BAL app
// ABP cnx
#define DEVICE_ID 30

#ifdef ABP
#define SECRET_NWKSKEY  { 0x... }
#define SECRET_APPKEY   { 0x27,.... }
#define SECRET_DEVADDR  0x........
#else
#define SECRET_APPEUI   { 0x70, ... }    // MSB
#define SECRET_DEVEUI   { 0x49, ... }    // LSB
#define SECRET_APPKEY   { 0x70, ... }    // MSB
#endif
