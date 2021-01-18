 /**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of and a contribution to the lwIP TCP/IP stack.
 *
 * Credits go to Adam Dunkels (and the current maintainers) of this software.
 *
 * Christiaan Simons rewrote this file to get a more stable echo application.
 *
 **/

/* This file was modified by ST */

/* Modified by Bruno Casu (2021 - SPRACE, São Paulo BR) */

#include "telnet_server.h"

#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"

#include <string.h>
#include "main.h"

#if LWIP_TCP

// maximum number of telnet connections
#define MAX_NUM_TELNET_INST   4

enum tcp_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};

// structure to maintain connection information to be passed as argument to LwIP callbacks
struct tcp_mng_struct
{
  uint8_t state;                        /* current connection state */
  uint8_t retries;
  struct tcp_pcb *pcb;                  /* pointer on the current tcp_pcb */
  uint8_t telnet_instance;              /* telnet instance identifier */
  UART_HandleTypeDef* serial_handler;   /* handler of the UART peripheral */
  struct pbuf *p;                       /* pointer on the received/to be transmitted pbuf */
};

// functions based in the tcp echosever example
void telnet_init(uint8_t telnet_inst);
static err_t tcp_com_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_com_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_com_error(void *arg, err_t err);
static void tcp_com_connection_close(struct tcp_pcb *tpcb, struct tcp_mng_struct *es);

// functions created for the telnet connection
static void tcp_recv_reset(struct tcp_pcb *tpcb, struct tcp_mng_struct *es);
void tcp_pbuf_to_serial (struct pbuf* p, UART_HandleTypeDef* serial_handler);

// globals to store TCP data for the different instances
static struct tcp_pcb* telnet_pcb[MAX_NUM_TELNET_INST];
static struct tcp_pcb* host_pcb[MAX_NUM_TELNET_INST];
static uint16_t tcp_port[MAX_NUM_TELNET_INST] = {0};
static UART_HandleTypeDef* tcp_serial_handler[MAX_NUM_TELNET_INST];

// serial to tcp variables
char single_character; // store received char from UART
StreamBufferHandle_t serial_input_stream; // stream buffer handler

char serial_msg_buff[1024] = {0};
uint16_t serial_msg_size = 0;
int next_char_timeout = 0;

// serial to tcp task handler and attributes
osThreadId_t serial_to_tcp_TaskHandle;
const osThreadAttr_t serial_to_tcp_TaskAttributes = {
  .name = "serial to tcp",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 8
};
// serial to tcp task function
void serial_to_tcp_Task (void *argument);
// custom callback for UART recv
void telnet_serial_RxCpltCallback(UART_HandleTypeDef *UartHandle);

/**
 * @brief create a new instance of telnet connection in a defined port
 * @param port Number of the TCP connection Port
 * @param serial_handler handler for the UART peripheral
 * @retval None
 *
 * @note This function can be called multiple times on the program.
 * 		 For each call a new telnet instance will be created and its parameters
 * 		 will be stored in the global arrays accordingly
 */
void telnet_create (uint16_t port, UART_HandleTypeDef* serial_handler)
{
  static uint8_t telnet_instance = 0;
  
  // register the UART callback for the recv mode
  // HAL_TIM_RegisterCallback(&htim2, HAL_TIM_PERIOD_ELAPSED_CB_ID, Telnet_Timer_Callback);
  HAL_UART_RegisterCallback(serial_handler, HAL_UART_RX_COMPLETE_CB_ID, telnet_serial_RxCpltCallback);

  // check if new instance reaches the maximum
  if (telnet_instance < MAX_NUM_TELNET_INST)
  {
    // add the port of the TCP connection to the global array
	tcp_port[telnet_instance] = port;

    // add the handler of the serial peripheral to the global array
    tcp_serial_handler[telnet_instance] = serial_handler;

    // create new TCP connection for the given instance
    telnet_init(telnet_instance);

    if(telnet_instance == 0)
    {
      // create serial recv task
      serial_to_tcp_TaskHandle = osThreadNew(serial_to_tcp_Task, NULL, &serial_to_tcp_TaskAttributes);
    }

    // set counter for next instance
    telnet_instance++;
  }
}

/**
  * @brief Initializes the tcp server
  * @param telnet_inst Number of the telnet instance
  * @retval None
  */
void telnet_init(uint8_t telnet_inst)
{
  // create new tcp protocol control block for the given instance
  telnet_pcb[telnet_inst] = tcp_new();
  
  if (telnet_pcb[telnet_inst] != NULL)
  {
    err_t err;
    
    // bind to defined port
    err = tcp_bind(telnet_pcb[telnet_inst], IP_ADDR_ANY, tcp_port[telnet_inst]);
    
    if (err == ERR_OK)
    {
      // start tcp listening
      telnet_pcb[telnet_inst] = tcp_listen(telnet_pcb[telnet_inst]);
      
      // set tcp_accept callback function
      tcp_accept(telnet_pcb[telnet_inst], tcp_com_accept);
    }
    else 
    {
      // deallocate the pcb if binding failed
      memp_free(MEMP_TCP_PCB, telnet_pcb[telnet_inst]);
    }
  }
  else
  {
	  // fail to create new protocol control block
	  while (1);
  }
}

/**
  * @brief  This function is the implementation of tcp_accept LwIP callback
  * @param  arg: not used
  * @param  newpcb: pointer on tcp_pcb struct for the newly created tcp connection
  * @param  err: not used 
  * @retval err_t: error status
  */
static err_t tcp_com_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  err_t ret_err;
  struct tcp_mng_struct *es;
  int inst_located = 0;
  uint8_t inst = 0;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  // find the correct instance for this connection
  while (inst_located == 0)
  {
    // match the received connection port with the telnet instance port
    if((newpcb->local_port == tcp_port[inst]) && (inst < MAX_NUM_TELNET_INST))
    {
      inst_located = 1;
    }
    // increment instance value
    inst++;
    
    // if received port does not match with any instance return error
    if(inst >= MAX_NUM_TELNET_INST)
      return ERR_VAL;
  }
  inst--; // fix addition from the loop

  // set priority for the newly accepted tcp connection newpcb
  tcp_setprio(newpcb, TCP_PRIO_MIN);
  
  // save pcb data for transmission tp the connect host
  host_pcb[inst] = (struct tcp_pcb *)newpcb;

  // allocate structure es to maintain tcp connection informations
  es = (struct tcp_mng_struct *)mem_malloc(sizeof(struct tcp_mng_struct));
  if (es != NULL)
  {
    es->state = ES_ACCEPTED; // update tcp state
    es->pcb = newpcb; // save connection information
    es->telnet_instance = inst; // save connection instance
    es->serial_handler = tcp_serial_handler[inst]; // save serial handler of this instance
    es->retries = 0;
    es->p = NULL;
    
    // pass newly allocated es structure as argument to newpcb
    tcp_arg(newpcb, es);
    
    // initialize lwip tcp_recv callback function for newpcb
    tcp_recv(newpcb, tcp_com_recv);
    
    // initialize lwip tcp_err callback function for newpcb
    tcp_err(newpcb, tcp_com_error);
    
    // initialize serial peripheral in recv mode
    HAL_UART_Receive_IT(tcp_serial_handler[inst], &single_character, 1);

    ret_err = ERR_OK;
  }
  else
  {
    // close tcp connection
    tcp_com_connection_close(newpcb, es);

    // return memory error
    ret_err = ERR_MEM;
  }
  return ret_err;  
}


/**
  * @brief  This function is the implementation for tcp_recv LwIP callback
  * @param  arg: pointer on a argument for the tcp_pcb connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  pbuf: pointer on the received pbuf
  * @param  err: error information regarding the reveived pbuf
  * @retval err_t: error code
  */
static err_t tcp_com_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct tcp_mng_struct *es;
  err_t ret_err;

  LWIP_ASSERT("arg != NULL",arg != NULL);
  
  es = (struct tcp_mng_struct *)arg;
  
  // if we receive an empty tcp frame from client => close connection
  if (p == NULL)
  {
    // remote host closed connection
    es->state = ES_CLOSING;
    if(es->p == NULL)
    {
       // we're done sending, close connection
       tcp_com_connection_close(tpcb, es);
    }
    else // if there are still data to be sent
    {
      tcp_pbuf_to_serial(p, es->serial_handler);
      
      // clear for new reception
      tcp_recv_reset(tpcb, es);
    }
    ret_err = ERR_OK;
  }   
  // else : a non empty frame was received from client but for some reason err != ERR_OK
  else if(err != ERR_OK)
  {
    // free received pbuf
    if (p != NULL)
    {
      es->p = NULL;
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == ES_ACCEPTED)
  {
    // first data chunk in p->payload
    es->state = ES_RECEIVED;
    
    // store reference to incoming pbuf (chain)
    es->p = p;

    // forward pkt data to serial interface
    tcp_pbuf_to_serial(p, es->serial_handler);

    // clear for new reception
    tcp_recv_reset(tpcb, es);

    ret_err = ERR_OK;
  }
  else if(es->state == ES_RECEIVED)
  {
    // more data received from client and previous data has been already sent
    if(es->p == NULL)
    {
      es->p = p;
  
      // forward pkt data to serial interface
      tcp_pbuf_to_serial(p, es->serial_handler);

      // clear for new reception
      tcp_recv_reset(tpcb, es);
    }
    else
    {
      struct pbuf *ptr;

      // chain pbufs to the end of what we recv'ed previously
      ptr = es->p;
      pbuf_chain(ptr,p);

    }
    ret_err = ERR_OK;
  }
  else if(es->state == ES_CLOSING)
  {
    // odd case, remote side closing twice, trash data
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  else
  {
    // unkown es->state, trash data
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_err callback function (called
  *         when a fatal tcp_connection error occurs. 
  * @param  arg: pointer on argument parameter 
  * @param  err: not used
  * @retval None
  */
static void tcp_com_error(void *arg, err_t err)
{
  struct tcp_mng_struct *es;

  LWIP_UNUSED_ARG(err);

  es = (struct tcp_mng_struct *)arg;
  if (es != NULL)
  {
    //  free es structure
    mem_free(es);
  }
}

/**
 * @brief clear tcp struct to enable new reception
 *
 */
static void tcp_recv_reset(struct tcp_pcb *tpcb, struct tcp_mng_struct *es)
{
	struct pbuf *ptr;

    // get pointer on pbuf from es structure
	ptr = es->p;

	uint16_t plen;
	uint8_t freed;

	plen = ptr->len;

	// continue with next pbuf in chain (if any)
	es->p = ptr->next;

	if(es->p != NULL)
	{
	  // increment reference count for es->p
	  pbuf_ref(es->p);
  }

	// chop first pbuf from chain
	do
	{
	  // try hard to free pbuf
	  freed = pbuf_free(ptr);
	}
	while(freed == 0);
	// we can read more data now
	tcp_recved(tpcb, plen);
}


/**
  * @brief  This functions closes the tcp connection
  * @param  tcp_pcb: pointer on the tcp connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
static void tcp_com_connection_close(struct tcp_pcb *tpcb, struct tcp_mng_struct *es)
{
  // remove all callbacks
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);
  
  // delete es structure
  if (es != NULL)
  {
    mem_free(es);
  }  

  // close tcp connection
  tcp_close(tpcb);
}


/**
 * @brief Send the received packet data (TCP) via serial port (UART)
 * @param p TCP packet information struct
 * @param serial_handler peripheral information struct used to send the data
 * 
 * @note this function is called at the callback set in the tcp_recv() function
 */
void tcp_pbuf_to_serial (struct pbuf* p, UART_HandleTypeDef* serial_handler)
{
	char* buff_ptr;
	char* tcp_data;
	uint16_t tcp_data_size = 0;

	tcp_data_size = p->len;

	if (tcp_data_size > 0)
	{
	  tcp_data = pvPortMalloc(tcp_data_size);
	  buff_ptr = (char*)p->payload;
	
	  for (int i = 0; i<tcp_data_size; i++){
		tcp_data[i] = buff_ptr[i];}
		  
      // send TCP data to UART
      for (int k = 0; k < tcp_data_size; k++){
	    HAL_UART_Transmit(serial_handler, &tcp_data[k], 1, 1000);}

	  // send carriage return byte - REMOVE IN FINAL IMPLEMENTATION
      HAL_UART_Transmit(serial_handler, 0x0D, 1, 1000);
	    
      // clear the buffer for next transmission
      vPortFree(tcp_data);
      tcp_data_size = 0;
    }
}


/**
 *
 *
 */
void serial_to_tcp_Task (void *argument)
{
  char c;

  // create the stream buffer
  serial_input_stream = xStreamBufferCreate(10, 1);

  for(;;)
  {
	  xStreamBufferReceive(serial_input_stream, &c, 1, portMAX_DELAY);

	  if (c == 'g')
	  {
		  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); // orange led debug
	  }

  }

}

/**
 * @brief set callback function for the interrupt handler of UART
 * 
 */
void telnet_serial_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xStreamBufferSendFromISR(serial_input_stream, &single_character, 1, &xHigherPriorityTaskWoken);

  // reset UART recv
  HAL_UART_Receive_IT(UartHandle, &single_character, 1);

  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); // red led debug
}



//WARNING Must enable USE_HAL_TIM_REGISTER_CALLBACKS from the file stm32h7xx_hal_conf.h to use specific callbacks
//Timer Callback is called once every 2ms
// void Telnet_Timer_Callback(TIM_HandleTypeDef *htim)
// {
//   if(htim->Instance == TIM2)
//   {
//     //check if 20 ms timeout is reached
//     if(serial_msg_size>0)
//     {
//       next_char_timeout++;
//       
//       if(next_char_timeout>10)
//       {
//         //send TCP pkt
//         tcp_write(host_pcb[0], serial_msg_buff, serial_msg_size, 1);
//         serial_msg_size = 0;
//         next_char_timeout = 0;
//       }
//       HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); // orange led debug
//       HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); // red led
//     }
//   }
// }
#endif /* LWIP_TCP */
