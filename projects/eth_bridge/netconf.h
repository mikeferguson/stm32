#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif  

void LwIP_Init(void);
void LwIP_Pkt_Handle(void);
void LwIP_Periodic_Handle(uint32_t localtime);
void Ethernet_Init(void);

void send_raw_packet();

#ifdef __cplusplus
}
#endif
