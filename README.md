# ETH-com
Ethernet interface configuration for the STM32 NUCLEO-H745ZI-Q Board

##Implemented Functions (so far):

###1) Periodic transmission of UDP messages (incremented couter).
###2) UDP Echo Server (any UDP messages sent to Port 7777 will be echoed by the NUCLEO board).
###3) DHCP Configuration (the board requests an IP address to the DHCP server and uses the given address in the ETH interface).
###4) TCP Echo Server (on port 7)
###5) Retransmission of TCP data via Serial interface (UART)
