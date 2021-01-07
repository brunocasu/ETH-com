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

#include "stream_buffer.h"

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

osThreadId_t telnet_serial_recv_task_handle;
const osThreadAttr_t telnet_serial_recv_task_attributes = {
  .name = "serial recv",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 256 * 4
};

/**** semaphore to enable the serial transmission if received data from the TCP connection ****/
static SemaphoreHandle_t serial_send_release_semphr = NULL;

void telnet_serial_recv_task (void *argument);

/**** function to clear the buffers for a new pkt reception ****/
static void telnet_recv_reset(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es);

/**** global to store host pcb data ****/
struct tcp_pcb * host_pcb;

/**** tcp to serial ****/
char* tcp_data;
uint16_t tcp_data_size = 0;

/**** serial to tcp ****/
char single_character;
char serial_msg_buff[1024] = {0};
uint16_t serial_msg_size = 0;
int timer_on_flag = 0;
int next_char_timeout = 0;
static SemaphoreHandle_t timer_stop_semphr = NULL;


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
  
  /**** keep pcb data for transmission ****/
  host_pcb = (struct tcp_pcb *)newpcb;

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
  err_t ret_val;
  
  ret_val = tcp_write(host_pcb, message, len, 1);
//   if (ret_val == ERR_OK)
//   {
//     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET); // orange led
//   }
//   else
//   {
//     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
//   }
    
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
  
  //stream buffer implementation
  //serial_to_tcp_stream_h = xStreamBufferCreate(100, 1);
  
  // create semaphores
  serial_send_release_semphr = xSemaphoreCreateBinary();
  
  // Start TCP server
  telnet_init(); // echoserver has been modified to send pkt data through serial port (UART)
    
  HAL_UART_Receive_IT(&huart3, &single_character, 1);
  // create receiver task
  //telnet_serial_recv_task_handle = osThreadNew(telnet_serial_recv_task, NULL, &telnet_serial_recv_task_attributes);
  
  HAL_TIM_Base_Start_IT(&htim2);
  
  for(;;){osDelay(1000);}
  
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
	
	  for (int i = 0; i<tcp_data_size; i++){
		  tcp_data[i] = buff_ptr[i];}
		  
    // send TCP data to UART
    for (int k = 0; k < tcp_data_size; k++){
	    HAL_UART_Transmit(&huart3, &tcp_data[k], 1, 1000);}

	  // send carriage retunr byte - REMOVE IN FINAL IMPLEMENTATION  
    HAL_UART_Transmit(&huart3, 0x0D, 1, 1000);
	    
    // clear the buffer for next transmission
    vPortFree(tcp_data);
    tcp_data_size = 0;
  }
	//xSemaphoreGiveFromISR(serial_send_release_semphr, &xHigherPriorityTaskWoken);
	//portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


/**
 * @brief
 * 
 */
void telnet_serial_recv_task (void *argument)
{   
  // create semaphore to stop the timer
  //timer_stop_semphr = xSemaphoreCreateBinary();
  
  // set UART to recv mode
	HAL_UART_Receive_IT(&huart3, &single_character, 1);
  
  for (;;)
  {
    osDelay(1000);

  }
}

/**
 * @brief set callback function for the interrupt handler of UART
 * 
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  // reset UART recv
  HAL_UART_Receive_IT(&huart3, &single_character, 1);
  
  // store character received in UART
  serial_msg_buff[serial_msg_size] = single_character;
  serial_msg_size++;
  
  next_char_timeout = 0;
  
  // check if max size of the buffer is reached
  if (serial_msg_size>=(sizeof(serial_msg_buff)-1))
  {
    // send TCP pkt
    telnet_send(serial_msg_buff, serial_msg_size);
    serial_msg_size = 0;

  }
  
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); // red led
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  
  HAL_UART_Receive_IT(&huart3, &single_character, 1);
}

//WARNING Must enable USE_HAL_TIM_REGISTER_CALLBACKS from the file stm32h7xx_hal_conf.h to use specific callbacks
//Timer Callback is called once every 2ms
void Telnet_Timer_Callback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
    //check if 20 ms timeout is reached
    if(serial_msg_size>0)
    {
      next_char_timeout++;
      
      if(next_char_timeout>10)
      {
        //send TCP pkt
        telnet_send(serial_msg_buff, serial_msg_size);
        serial_msg_size = 0;
        next_char_timeout = 0;
      }
      HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); // orange led debug
    }
  }
}
#endif /* LWIP_TCP */
