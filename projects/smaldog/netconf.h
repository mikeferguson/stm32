#ifndef NETCONF_H_
#define NETCONF_H_

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif  

/** The PHY address on the MDIO interface
 *  The default internal pullup/pulldowns on the KSZ8051MLL PHY
 *  should set the address to 1 after reset */
enum {PHY_ADDRESS = 1};

void LwIP_Init(void);
void LwIP_Pkt_Handle(void);
void LwIP_Periodic_Handle(uint32_t localtime);
uint32_t Ethernet_Init(void);

void send_raw_packet();

#ifdef __cplusplus
}
#endif

#endif  // NETCONF_H_