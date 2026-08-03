#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"
#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "app_util.h"
#include "sdk_common.h"

#define UART_TX_BUF_SIZE 2048       //串口发送缓存大小（字节数）
#define UART_RX_BUF_SIZE 2048       //串口接收缓存大小（字节数）
#define DATA_LENGTH 174             //一个包44
//定义载荷结构体，用于存储接收的数据
static nrf_esb_payload_t        rx_payload;
static nrf_esb_payload_t        tx_payload;
static uint8_t   m_tx_buf[NRF_ESB_MAX_PAYLOAD_LENGTH];
uint8_t send_flag = 0; //设置中断标志位

uint8_t ges_result_received[1] = {10};
uint8_t ges_result_flag = 0;

//串口事件回调函数，该函数中判断事件类型并进行处理
void uart_error_handle(app_uart_evt_t * p_event)
{
    
		if (p_event->evt_type ==  APP_UART_DATA_READY)
    {
			ges_result_flag = 1;
			UNUSED_VARIABLE(app_uart_get(&ges_result_received[0]));
		}
		//通讯错误事件
    else if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    //FIFO错误事件
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
//串口配置
void uart_config(void)
{
	uint32_t err_code;
	
	//定义串口通讯参数配置结构体并初始化
  const app_uart_comm_params_t comm_params =
  {
    RX_PIN_NUMBER,//定义uart接收引脚
    TX_PIN_NUMBER,//定义uart发送引脚
    RTS_PIN_NUMBER,//定义uart RTS引脚
    CTS_PIN_NUMBER,//定义uart CTS引脚
    APP_UART_FLOW_CONTROL_DISABLED,//关闭uart硬件流控
    false,//禁止奇偶检验
    NRF_UART_BAUDRATE_1000000//uart波特率设置
  };
  //初始化串口，注册串口事件回调函数
  APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

  APP_ERROR_CHECK(err_code);
	
}
ret_code_t err_code_receive;
//ESB事件回调函数，该函数中判断事件类型并进行处理
void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
  	uint32_t err_code;
	
	  switch (p_event->evt_id)
    {
        
			case NRF_ESB_EVENT_TX_SUCCESS://ESB发送成功事件
            break;
        case NRF_ESB_EVENT_TX_FAILED://ESB发送失败事件
					   //清理发送缓存
            (void) nrf_esb_flush_tx();//清理发送缓存
            break;
        case NRF_ESB_EVENT_RX_RECEIVED://ESB数据接收事件
            //读取数据并使用ACK返回手势识别结果
				    while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
              if (rx_payload.length > 0)
              {
                send_flag = 1;
              }
            }
						if(ges_result_flag == 1)
						{
							tx_payload.length = 1;
							tx_payload.data[0] = ges_result_received[0];
							tx_payload.pipe = 0;
							err_code_receive = nrf_esb_write_payload(&tx_payload);
							ges_result_flag = 0;
						}
            break;
    }
}
//初始化配置ESB
uint32_t esb_base_init(void)
{
    uint32_t err_code;
    //ESB配置结构体初始化为默认参数
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
		//重写需要修改的参数
	  nrf_esb_config.payload_length           = NRF_ESB_MAX_PAYLOAD_LENGTH;//设置载荷长度
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;//动态数据长度
    nrf_esb_config.retransmit_delay         = 500;                     //重发延时600us
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;   //数据速率2MBPS
    nrf_esb_config.event_handler            = nrf_esb_event_handler;   //ESB事件处理函数
    nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;        //主接收
    nrf_esb_config.selective_auto_ack       = false;                   //应答所有数据包
    //初始化ESB
    err_code = nrf_esb_init(&nrf_esb_config);
    return err_code;
}
uint32_t esb_set_additional_parm(void)
{
	  uint32_t err_code;

    uint8_t base_addr_0[4] = {0xE2, 0xE3, 0xE4, 0xE5};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
   uint8_t addr_prefix[8] = {0xE1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };    //主机
		
    //设置基础地址（通道0的基础地址）
    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);
    //设置基础地址（通道1~7的基础地址）
    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);
    //设置前缀地址
    err_code = nrf_esb_set_prefixes(addr_prefix, NRF_ESB_PIPE_COUNT);
//	    err_code = nrf_esb_set_prefixes(addr_prefix, 6);	
		
    VERIFY_SUCCESS(err_code);
		
		//设置无线信道
		err_code = nrf_esb_set_rf_channel(70);    
		VERIFY_SUCCESS(err_code);
		
		return err_code;
}

//启动64MHz时钟
void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;//清零高频时钟启动事件
    NRF_CLOCK->TASKS_HFCLKSTART = 1;   //启动高频时钟
    //等待高频时钟启动完成
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

int main(void)
{
  ret_code_t err_code;
	
  //初始化开发板上的4个LED，即将驱动LED的GPIO配置为输出，
  bsp_board_init(BSP_INIT_LEDS);
	clocks_start();
  //初始化串口
  uart_config();
	//初始化ESB基础参数
	err_code = esb_base_init();
  APP_ERROR_CHECK(err_code);
	//配置ESB地址、信道和发射功率
	(void)esb_set_additional_parm();
	//启动ESB接收
	err_code = nrf_esb_start_rx();
  APP_ERROR_CHECK(err_code);
	

  while(true)
  {
		if(send_flag == 1)
		{
				//将接收到的数据帧通过串口透传到PC
				 for(uint32_t i=0;i<rx_payload.length;i++)
            {									
						 printf("%c",rx_payload.data[i]);							
             if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    APP_ERROR_CHECK(err_code);
                 }
																												
             }								
				nrf_gpio_pin_toggle(LED_1);
				send_flag = 0 ;
		
		}

  }
}
/********************************************END FILE**************************************/
