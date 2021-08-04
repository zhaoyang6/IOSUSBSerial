#include "udpserver.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#define UDP_SERVER_PORT 8000 /*define the UDP local connection port*/

/**
 * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
 * @param arg user supplied argument (udp_pcb.recv_arg)
 * @param pcb the udp_pcb which received data
 * @param p the packet buffer that was received
 * @param addr the remote IP address from which the packet was received
 * @param port the remote port from which the packet was received
 * @retval None
 */
static void udp_echoserver_receive_callback(__attribute__((unused)) void *arg, struct udp_pcb *upcb, struct pbuf *p, const struct ip4_addr *addr, u16_t port)
{
    /*Connect to the remote client*/
    udp_connect(upcb, addr, port);

    /*Tell the client that we have accepted it */
    udp_send(upcb, p);

    /* free the UDP connection, so we can accept new clients */
    udp_disconnect(upcb);
    pbuf_free(p);
}

/**
 * @brief Initialize the server application.
 * @param None
 * @param None
 */
void udp_echoserver_init(void)
{
    struct udp_pcb *upcb;
    err_t err;

    /* Create a new UDP control block */
    upcb = udp_new();

    if(upcb) {
        /* Bind the upcb to the UDP_PORT port */
        /* Using IP_ADDR_ANY allow the upcb to be used by any local interface*/
        err = udp_bind(upcb, IP_ADDR_ANY, UDP_SERVER_PORT);

        if (err == ERR_OK) {
            udp_recv(upcb, udp_echoserver_receive_callback, NULL);
        } else {
            //printf("can not bind pcb");
        }
    } else {
        //printf("can not create pcb");
    }
}
