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

/**** Modified by Bruno Casu (SPRACE, São Paulo BR) ****/

#include "telnet_server.h"

#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"

#include <string.h>
#include "main.h"

#if LWIP_TCP

static struct tcp_pcb *tcp_echoserver_pcb;

/* ECHO protocol states */
enum tcp_echoserver_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};

/* structure for maintaing connection infos to be passed as argument 
   to LwIP callbacks*/
struct tcp_echoserver_struct
{
  u8_t state;             /* current connection state */
  u8_t retries;
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
};

void telnet_init(void);
static err_t tcp_echoserver_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_echoserver_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_echoserver_error(void *arg, err_t err);
static err_t tcp_echoserver_poll(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_echoserver_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_echoserver_send(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es);
static void tcp_echoserver_connection_close(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es);


/**** added global pcb to send tcp pkts ****/
static struct tcp_pcb* host_tpcb;
static struct tcp_echoserver_struct* tn;

/**** telnet task handlers ****/
osThreadId_t telnet_recv_task_handle;
const osThreadAttr_t telnet_recv_task_attributes = {
  .name = "telnet recv",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 256 * 4
};

osThreadId_t telnet_tx_task_handle;
const osThreadAttr_t telnet_tx_task_attributes = {
  .name = "telnet tx",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 256 * 4
};

osThreadId_t telnet_serial_recv_task_handle;
const osThreadAttr_t telnet_serial_recv_task_attributes = {
  .name = "serial recv",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 256 * 4
};

/**** semaphore to enable the serial transmission if received data from the TCP connection ****/
static SemaphoreHandle_t serial_send_release_semphr = NULL;

/**** semaphore used in the ISR of the serial port to inform character reception ****/
static SemaphoreHandle_t char_recv_semphr = NULL;

static SemaphoreHandle_t send_tcp_pkt_semphr  = NULL;

void telnet_tx_task(void *argument);
void telnet_serial_recv_task (void *argument);

/**** function to clear the buffers for a new pkt reception ****/
static void telnet_recv_reset(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es);

/**** tcp to serial buffers ****/
char* tcp_data;
uint16_t tcp_data_size = 0;

/**** serial to tcp buffers ****/
char single_character;
char serial_to_tcp_buff[1024] = {0};
uint16_t serial_to_tcp_buff_size = 0;

/**** add UART handler ****/
//#define huart3 huart3;

/**
  * @brief  Initializes the tcp echo server
  * @param  None
  * @retval None
  * @note the echo server driver was altered to implement the telnet protocol operations
  */
void telnet_init(void)
{
  /* create new tcp pcb */
  tcp_echoserver_pcb = tcp_new();
  int k = 0;
  if (tcp_echoserver_pcb != NULL)
  {
    err_t err;
    
    /**** bind to telet default port (PORT 23) ****/
    err = tcp_bind(tcp_echoserver_pcb, IP_ADDR_ANY, 23);
    
    if (err == ERR_OK)
    {
      /* start tcp listening for echo_pcb */
      tcp_echoserver_pcb = tcp_listen(tcp_echoserver_pcb);
      
      /* initialize LwIP tcp_accept callback function */
      tcp_accept(tcp_echoserver_pcb, tcp_echoserver_accept);
    }
    else 
    {
      /* deallocate the pcb */
      memp_free(MEMP_TCP_PCB, tcp_echoserver_pcb);
    }
  }
  else
  {
	  k = MEMP_NUM_SYS_TIMEOUT;
    /**** function should not block ****/
	  // while (1); // tcp_new() returning NULL pointer
  }
}

/**
  * @brief  This function is the implementation of tcp_accept LwIP callback
  * @param  arg: not used
  * @param  newpcb: pointer on tcp_pcb struct for the newly created tcp connection
  * @param  err: not used 
  * @retval err_t: error status
  */
static err_t tcp_echoserver_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  err_t ret_err;
  struct tcp_echoserver_struct *es;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  /* set priority for the newly accepted tcp connection newpcb */
  tcp_setprio(newpcb, TCP_PRIO_MIN);

  /* allocate structure es to maintain tcp connection informations */
  es = (struct tcp_echoserver_struct *)mem_malloc(sizeof(struct tcp_echoserver_struct));
  if (es != NULL)
  {
    es->state = ES_ACCEPTED;
    es->pcb = newpcb;
    es->retries = 0;
    es->p = NULL;
    
    /* pass newly allocated es structure as argument to newpcb */
    tcp_arg(newpcb, es);
    
    /* initialize lwip tcp_recv callback function for newpcb  */ 
    tcp_recv(newpcb, tcp_echoserver_recv);
    
    /* initialize lwip tcp_err callback function for newpcb  */
    tcp_err(newpcb, tcp_echoserver_error);
    
    /* initialize lwip tcp_poll callback function for newpcb */
    //tcp_poll(newpcb, tcp_echoserver_poll, 0);
    
    ret_err = ERR_OK;
  }
  else
  {
    /*  close tcp connection */
    tcp_echoserver_connection_close(newpcb, es);

    /* return memory error */
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
static err_t tcp_echoserver_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct tcp_echoserver_struct *es;
  err_t ret_err;

  LWIP_ASSERT("arg != NULL",arg != NULL);
  
  es = (struct tcp_echoserver_struct *)arg;
  
  /* if we receive an empty tcp frame from client => close connection */
  if (p == NULL)
  {
    /* remote host closed connection */
    es->state = ES_CLOSING;
    if(es->p == NULL)
    {
       /* we're done sending, close connection */
       tcp_echoserver_connection_close(tpcb, es);

    }
    else
    {
      /* we're not done yet */
      /* acknowledge received packet */
      //tcp_sent(tpcb, tcp_echoserver_sent);
      
      /**** forward pkt data to serial interface ****/
      tcp_pbuf_to_serial(p);
      
      /**** clear for new reception ****/
      telnet_recv_reset(tpcb, es);
      
      /* send remaining data*/
      //tcp_echoserver_send(tpcb, es);
    }
    ret_err = ERR_OK;
  }   
  /* else : a non empty frame was received from client but for some reason err != ERR_OK */
  else if(err != ERR_OK)
  {
    /* free received pbuf*/
    if (p != NULL)
    {
      es->p = NULL;
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == ES_ACCEPTED)
  {
    /* first data chunk in p->payload */
    es->state = ES_RECEIVED;
    
    /* store reference to incoming pbuf (chain) */
    es->p = p;

    /**** forward pkt data to serial interface ****/
    tcp_pbuf_to_serial(p);

    /**** clear for new reception ****/
    telnet_recv_reset(tpcb, es);
    
    /**** skip re-transmission of tcp message (echo) ****/
    /* send back the received data (echo) */
    // tcp_echoserver_send(tpcb, es);
    
    /**** store tpcb struct for transsmission ****/
    //telnet_send(); TODO send ACK msg as connection is established
    //host_tpcb = tpcb;
    //tn = es;
    
    ret_err = ERR_OK;
  }
  else if (es->state == ES_RECEIVED)
  {
    /* more data received from client and previous data has been already sent*/
    if(es->p == NULL)
    {
      es->p = p;
  
      /**** forward pkt data to serial interface ****/
      tcp_pbuf_to_serial(p);

      /**** clear for new reception ****/
      telnet_recv_reset(tpcb, es);

      /**** skip re-transmission of tcp message (echo) ****/
      /* send back received data */
      //tcp_echoserver_send(tpcb, es);
    }
    else
    {
      struct pbuf *ptr;

      /* chain pbufs to the end of what we recv'ed previously  */
      ptr = es->p;
      pbuf_chain(ptr,p);

    }
    ret_err = ERR_OK;
  }
  else if(es->state == ES_CLOSING)
  {
    /* odd case, remote side closing twice, trash data */
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  else
  {
    /* unkown es->state, trash data  */
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
static void tcp_echoserver_error(void *arg, err_t err)
{
  struct tcp_echoserver_struct *es;

  LWIP_UNUSED_ARG(err);

  es = (struct tcp_echoserver_struct *)arg;
  if (es != NULL)
  {
    /*  free es structure */
    mem_free(es);
  }
}

/**
  * @brief  This function implements the tcp_poll LwIP callback function
  * @param  arg: pointer on argument passed to callback
  * @param  tpcb: pointer on the tcp_pcb for the current tcp connection
  * @retval err_t: error code
  */
static err_t tcp_echoserver_poll(void *arg, struct tcp_pcb *tpcb)
{
  err_t ret_err;
  struct tcp_echoserver_struct *es;

  es = (struct tcp_echoserver_struct *)arg;
  if (es != NULL)
  {
    if (es->p != NULL)
    {
      tcp_sent(tpcb, tcp_echoserver_sent);
      /* there is a remaining pbuf (chain) , try to send data */
      //tcp_echoserver_send(tpcb, es);
    }
    else
    {
      /* no remaining pbuf (chain)  */
      if(es->state == ES_CLOSING)
      {
        /*  close tcp connection */
        tcp_echoserver_connection_close(tpcb, es);
      }
    }
    ret_err = ERR_OK;
  }
  else
  {
    /* nothing to be done */
    tcp_abort(tpcb);
    ret_err = ERR_ABRT;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_sent LwIP callback (called when ACK
  *         is received from remote host for sent data) 
  * @param  None
  * @retval None
  */
static err_t tcp_echoserver_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct tcp_echoserver_struct *es;

  LWIP_UNUSED_ARG(len);

  es = (struct tcp_echoserver_struct *)arg;
  es->retries = 0;
  
  if(es->p != NULL)
  {
    /* still got pbufs to send */
    tcp_sent(tpcb, tcp_echoserver_sent);
    //tcp_echoserver_send(tpcb, es);
  }
  else
  {
    /* if no more data to send and client closed connection*/
    if(es->state == ES_CLOSING)
      tcp_echoserver_connection_close(tpcb, es);
  }
  return ERR_OK;
}


/**
  * @brief  This function is used to send data for tcp connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
static void tcp_echoserver_send(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;
 
  while ((wr_err == ERR_OK) &&
         (es->p != NULL) && 
         (es->p->len <= tcp_sndbuf(tpcb)))
  {
    
    /* get pointer on pbuf from es structure */
    ptr = es->p;

    /* enqueue data for transmission */
    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);
    
    if (wr_err == ERR_OK)
    {
      u16_t plen;
      u8_t freed;

      plen = ptr->len;
     
      /* continue with next pbuf in chain (if any) */
      es->p = ptr->next;
      
      if(es->p != NULL)
      {
        /* increment reference count for es->p */
        pbuf_ref(es->p);
      }
      
     /* chop first pbuf from chain */
      do
      {
        /* try hard to free pbuf */
        freed = pbuf_free(ptr);
      }
      while(freed == 0);
     /* we can read more data now */
     tcp_recved(tpcb, plen);
   }
   else if(wr_err == ERR_MEM)
   {
      /* we are low on memory, try later / harder, defer to poll */
     es->p = ptr;
   }
   else
   {
     /* other problem ?? */
   }
  }
}

/**
 * @brief send TCP pkt with characters from the serial port 
 * 
 */
void telnet_send (char* message, uint16_t len)
{
  //TODO implement tx function
  
}

/**
 * @brief clear tcp struct to enable new reception
 *
 */
static void telnet_recv_reset(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es)
{
	struct pbuf *ptr;

  /* get pointer on pbuf from es structure */
	ptr = es->p;

	uint16_t plen;
	uint8_t freed;

	plen = ptr->len;

	/* continue with next pbuf in chain (if any) */
	es->p = ptr->next;

	if(es->p != NULL)
	{
	  /* increment reference count for es->p */
	  pbuf_ref(es->p);
  }

	/* chop first pbuf from chain */
	do
	{
	  /* try hard to free pbuf */
	  freed = pbuf_free(ptr);
	}
	while(freed == 0);
	/* we can read more data now */
	tcp_recved(tpcb, plen);
}

/**
  * @brief  This functions closes the tcp connection
  * @param  tcp_pcb: pointer on the tcp connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
static void tcp_echoserver_connection_close(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es)
{
  
  /* remove all callbacks */
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);
  
  /* delete es structure */
  if (es != NULL)
  {
    mem_free(es);
  }  

  /* close tcp connection */
  tcp_close(tpcb);
}

/**
 * @brief This task works implements the reception of TCP pkts via telnet, and the
 *        transmission of the received characters via serial port (UART) 
 * @note TCP connection is binding on default telnet port (PORT 23)
 */
void telnet_recv_task (void *argument)
{
  char carriage_return = 0x0D;
  
  // create receiver tasks
  telnet_tx_task_handle = osThreadNew(telnet_tx_task, NULL, &telnet_tx_task_attributes);
  telnet_serial_recv_task_handle = osThreadNew(telnet_serial_recv_task, NULL, &telnet_serial_recv_task_attributes);
  
  // create semaphores
  serial_send_release_semphr = xSemaphoreCreateBinary();
  
  // Start TCP server
  telnet_init(); // echoserver has been modified to send pkt data through serial port (UART)
  
  for(;;)
  {
    // block until tcp_pbuf_to_serial() function releases the semaphore (TCP pkt received)
    xSemaphoreTake ( serial_send_release_semphr, portMAX_DELAY );
		    
    // send TCP data to UART
    if (tcp_data_size > 0)
	  {
	    // transmit received data to the serial port
	    for (int k = 0; k < tcp_data_size; k++){
	      HAL_UART_Transmit(&huart3, &tcp_data[k], 1, 1000);}

	    HAL_UART_Transmit(&huart3, &carriage_return, 1, 1000);
	    
      // clear the buffer for next transmission
      vPortFree(tcp_data);
	    tcp_data_size = 0;
	  }
  }
}

/**
 * @brief Retrieve data from the TCP pkg and store in a global variable. Releases the semaphore to complete the transmission on the serial port.
 * @note this function is called at the interruption callback set for the tcp_recv() function
 */
void tcp_pbuf_to_serial (struct pbuf* p)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	char* buff_ptr;

	tcp_data_size = p->len;
	if (tcp_data_size > 0)
	{
		tcp_data = pvPortMalloc(tcp_data_size);
		buff_ptr = (char*)p->payload;
	}

	for (int i = 0; i<tcp_data_size; i++){
		tcp_data[i] = buff_ptr[i];}

  // TODO replace se,aphore with a queue
	xSemaphoreGiveFromISR(serial_send_release_semphr, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
 * @brief Send the data obtained from the serial port via TCP connection
 * @note this task uses a queue to organize the messages that are captured from the UART ISR
 */
void telnet_tx_task(void *argument)
{
  char test_buff[4] = {0};
//   QueueHandle_t serial_to_pbuf_queue;
//   serial_to_pbuf_queue = xQueueCreate( 10, sizeof(*char) );
  send_tcp_pkt_semphr = xSemaphoreCreateBinary();
  //TODO implement queue system
  
//   if(serial_to_pbuf_queue != NULL)
//   {
    for(;;)
    {
      xSemaphoreTake (send_tcp_pkt_semphr, portMAX_DELAY);
      //telnet_send (serial_to_tcp_buff, serial_to_tcp_buff_size);
      
      test_buff[0] = serial_to_tcp_buff_size;
      test_buff[1] = serial_to_tcp_buff[1];
      test_buff[2] = serial_to_tcp_buff[2];
    
      test_buff[3] = serial_to_tcp_buff[serial_to_tcp_buff_size -1];
      osDelay(1);
    }
//   }
//   else
//   {
//     for(;;)
//     {
//       osDelay(1000);
//     }
//   }
}

/**
 * @brief
 * 
 */
void telnet_serial_recv_task (void *argument)
{
  char_recv_semphr = xSemaphoreCreateBinary();
  
  for (;;)
  {
    // set UART receiver
    HAL_UART_Receive_IT(&huart3, &single_character, 1);
  
    xSemaphoreTake (char_recv_semphr, portMAX_DELAY);
  }
}

/**
 * @brief set callback function for the interrupt handler of UART
 * 
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	static BaseType_t xHigherPriorityTaskWoken;
	static uint16_t msg_len = 0;
	xHigherPriorityTaskWoken = pdFALSE;

  // check if interruption was called by the correct UART set for the telnet interface 
  if (huart3.Instance == UartHandle->Instance)
  {
    // check the character received: if is a NULL or Cariage ret. send data to telnet tx queue (end of msg)
    //                               if maximum size of the buffer is reached, send data to telnet tx queue
    //                               in the other cases, store the received char on the buffer
    if ((single_character == 0x00) || (single_character == 0x0d) || (serial_to_tcp_buff_size == 1023))  
    {
      serial_to_tcp_buff[msg_len] = single_character; // last character from the message
      serial_to_tcp_buff_size = msg_len+1; // total message size
      msg_len = 0; // reset buffer position
      // queue send to back from ISR
      xSemaphoreGiveFromISR(send_tcp_pkt_semphr, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
    else
    {
      // store received char on the buffer
      serial_to_tcp_buff[msg_len] = single_character;
      // increment recv buffer position
      msg_len++;
    }
    
    // release the semaphore to reset UART recv
    xSemaphoreGiveFromISR(char_recv_semphr, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );    
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
  
  // Error Handler
  
  xSemaphoreGiveFromISR(char_recv_semphr, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

#endif /* LWIP_TCP */
