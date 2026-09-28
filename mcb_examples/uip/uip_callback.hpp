#ifndef includeguard_uip_callback_hpp_includeguard
#define includeguard_uip_callback_hpp_includeguard

struct uip_udp_conn;
struct uip_conn;
struct uip_udpip_hdr;

#ifdef __cplusplus

extern "C" {
#endif

#include "uipopt.h"

unsigned int uip_tcp_retransmit (struct uip_conn* c, void* uip_appdata);

void uip_tcp_no_more_free_connections (void);

void uip_tcp_abort (struct uip_conn* c);
void uip_tcp_close (struct uip_conn* c);
void uip_tcp_timed_out (struct uip_conn* c);
void uip_tcp_connected (struct uip_conn* c);
void uip_tcp_ackdata (struct uip_conn* c);
void uip_tcp_newdata (struct uip_conn* c,
		      void* uip_appdata, unsigned int uip_len);
unsigned int uip_tcp_senddata (struct uip_conn* c, void* uip_appdata);


// uses direct send.
// to get the udp application data (payload) advance the header pointer by
// the header size.
void uip_appcall_udp (int flags, struct uip_udp_conn* c,
		      struct uip_udpip_hdr* hdr, unsigned int uip_len);

#ifndef UIP_UDP_APPCALL
#define UIP_UDP_APPCALL uip_appcall_udp
#endif

#ifdef __cplusplus
}
#endif

typedef struct uip_connection_state
{
  void* socket;

} uip_tcp_appstate_t;

typedef struct uip_connection_state_udp
{
  void* socket;
} uip_udp_appstate_t;

#endif // includeguard_uip_callback_hpp_includeguard
