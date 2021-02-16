# ETH-com
This project provides a Serial Over LAN implementation on the STM32H745ZI NUCLEO Board.
The software uses LwIP to establish a TCP server. Upon connection from a remote host (using standard telnet Port 23), the TCP
data received will be retransmitted using the USART3 (connected to the USB port on the NUCLEO board). Also, all the characters received
on the serial port will be transmitted, using the LwIP TCP functions, to the connected Host.

The SoL driver provided here also support multiple instances of TCP connections, each connection can be assigned to a serial port on the MCU.
To create an new instance, user must use the function telnet_create(), providing the UART handle (STM32 HAL struct) and the TCP port wich will be used.
The function will create the TCP server, connecting to any Host on the defined Port, and link the serial port to the transmission and retransmission of the data.
The maximum amount of simultaneous instances is defined in the telnet_server.h file. In the example, 2 instances were created, using Port 23 and Port 24, 
both redirecting data to the USART3 on the NUCLEO board.

The telnet driver used to implement the SoL functionality was added as a submodule of this project. If desired the updated source codes can be obtained from the
dedicated repository for the telnet driver.


