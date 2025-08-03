#include "thread_proto.h"
#include "cmsis_os.h"
#include <string.h>

UART_HandleTypeDef *g_data_logger = NULL;
UART_HandleTypeDef *g_local_monitor = NULL;
I2C_HandleTypeDef *g_driver_iface = NULL;
I2C_HandleTypeDef *g_mux_iface = NULL;
DMA_HandleTypeDef *g_logger_dma = NULL;
CAN_HandleTypeDef *g_can_iface = NULL;
USART_TypeDef *Current_Target = NULL;

GPIO_TypeDef *can_loader = NULL;
uint16_t *loader_gpio = NULL;

bool can_gpio_verification;

#define CAN_GPIO_TRIGGERED 0x008
#define CAN_GPIO_NON_TRIGGERED 0x009

max6650_data_t *g_driver_data = NULL;
bmp280_sensors_data_t *g_sensor_data = NULL;

#define rx_buff_size 10
#define main_buff_size 20
uint8_t rx_buff[rx_buff_size];
uint8_t main_buff[main_buff_size];
int rx_validation =0;

CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t TxData[8];

void split_uint16(uint16_t value, uint8_t *high_byte, uint8_t *low_byte) {
    *high_byte = (value >> 8) & 0xFF; 
    *low_byte  = value & 0xFF; 
}

void split_int16(int16_t value, uint8_t *high_byte, uint8_t *low_byte) {
    *high_byte = (value >> 8) & 0xFF; 
    *low_byte  = value & 0xFF; 
}

void can_logger_cycle(void const *argument){
  if (can_gpio_verification){
    TxHeader.StdId = CAN_GPIO_TRIGGERED;
  }else {
    TxHeader.StdId = CAN_GPIO_NON_TRIGGERED;
  }
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 6;
  TxHeader.TransmitGlobalTime = DISABLE;
  for(;;) {
    if(g_sensor_data!=NULL){
        split_int16(g_sensor_data->delta_pres_0,&TxData[0],&TxData[1]);
        split_int16(g_sensor_data->delta_pres_0,&TxData[2],&TxData[3]);
        split_int16(g_sensor_data->delta_pres_0,&TxData[4],&TxData[5]);
    }else {
         bmp280_sensors_data_t tmp_placeholder = {
            .delta_pres_0 = -100,
			.delta_pres_1 = 100,
			.delta_pres_2 = 365
         };
        split_int16(tmp_placeholder.delta_pres_0,&TxData[0],&TxData[1]);
        split_int16(tmp_placeholder.delta_pres_0,&TxData[2],&TxData[3]);
        split_int16(tmp_placeholder.delta_pres_0,&TxData[4],&TxData[5]);
    }
    if(g_driver_data!=NULL){
        split_uint16(g_driver_data->fan_rpm,&TxData[6],&TxData[7]);
    }else {
        max6650_data_t tmp_placeholder = {
            .fan_rpm = 1000,
			      .status_flag = HAL_OK
        };
        split_uint16(tmp_placeholder.fan_rpm,&TxData[6],&TxData[7]);
    }
    HAL_StatusTypeDef Current_Action = HAL_CAN_AddTxMessage(g_can_iface, &TxHeader, TxData, &TxMailbox);
    if (Current_Action==HAL_OK){
    	 NOTIFY_MESSAGE("Sent CAN Message\n");
    }
    HAL_Delay(5);
    osDelay(1);
  }
};
void driver_coll_cycle(void const *argument){
  for(;;){
    osDelay(1);
  }
}
void mux_pres_cycle(void const *argument){
  
  max6650_handle_t fan_controller;
  max6650_config_t fan_config;
  max6650_data_t fan_data;

  fan_config.hi2c = g_driver_iface;
  fan_config.add_line_connection = MAX6650_ADD_LINE_GND;
  fan_config.operating_mode = MAX6650_MODE_CLOSED_LOOP;
  fan_config.fan_voltage = MAX6650_FAN_VOLTAGE_12V;
  fan_config.k_scale = MAX6650_KSCALE_4;
  fan_config.rpm_max = 3000; // Maximum expected RPM for normal 12V motor
  fan_config.i2c_timeout = 1000;
  bool init_succ = false;
  if (MAX6650_Init(&fan_controller, &fan_config)){
    MAX6650_ControlFan(&fan_controller, CFF_AUTO);
    init_succ = true;
  }
  uint8_t sensor_addresses[6] = {0x76, 0x76, 0x76, 0x76, 0x76, 0x76};
  uint8_t sensor_channels[6] = {0, 1, 2, 3, 4, 5};
  bmp280_multi_system_t sensor_system;
  bmp280_sensors_data_t pressure_data;
  bool init_success = bmp280_multi_init(&sensor_system, g_mux_iface,NULL,0,0,sensor_addresses,sensor_channels);
  if (!init_success) {
    NOTIFY_MESSAGE("ERROR: Failed to initialize BMP280 multi-sensor system!");
  }

  for(;;) {
    if(init_success){
      if (bmp280_multi_force_measurement(&sensor_system)) {
        HAL_Delay(100);            
        while (bmp280_multi_is_measuring(&sensor_system)) {
          HAL_Delay(10);
        }            
        if (bmp280_multi_read_differences(&sensor_system, &pressure_data)) {
          g_sensor_data->delta_pres_0 = pressure_data.delta_pres_0;
          g_sensor_data->delta_pres_1 = pressure_data.delta_pres_1;
          g_sensor_data->delta_pres_2 = pressure_data.delta_pres_2;
        } else {
          NOTIFY_MESSAGE("ERROR: Failed to read sensor differences");
        }
      }else {
        NOTIFY_MESSAGE("ERROR: Failed to force measurements");
      }
    }else {
          g_sensor_data->delta_pres_0 = -200; //Mock Data
          g_sensor_data->delta_pres_1 = 200; //Mock Data
          g_sensor_data->delta_pres_2 = 730; //Mock Data
    }
    if(init_succ){
      if (MAX6650_ReadData(&fan_controller, &fan_data)) {
        g_driver_data->fan_rpm = fan_data.fan_rpm; 
        g_driver_data->status_flag = fan_data.status_flag; 
      }
    }else {
        g_driver_data->fan_rpm = 1200; //Mock Data
        g_driver_data->status_flag = HAL_OK;  //Mock Data
    }
    osDelay(1);
  }
};
void uart_logger_cycle(void const *argument){
  for(;;) {
    raw_data_t raw_data_placeholder;
    if(g_sensor_data!=NULL){
        raw_data_placeholder.delta_pres_0 = g_sensor_data->delta_pres_0;
        raw_data_placeholder.delta_pres_1 = g_sensor_data->delta_pres_1;
        raw_data_placeholder.delta_pres_2 = g_sensor_data->delta_pres_2;
    }else {
         bmp280_sensors_data_t tmp_placeholder = { //Mock Data
          .delta_pres_0 = -100, //Mock Data
          .delta_pres_1 = 100, //Mock Data
          .delta_pres_2 = 365 //Mock Data
         };
        raw_data_placeholder.delta_pres_0 = tmp_placeholder.delta_pres_0;
        raw_data_placeholder.delta_pres_1 = tmp_placeholder.delta_pres_1;
        raw_data_placeholder.delta_pres_2 = tmp_placeholder.delta_pres_2;
    }
    if(g_driver_data!=NULL){
        raw_data_placeholder.fan_rpm = g_driver_data->fan_rpm;
        raw_data_placeholder.status_flag = g_driver_data->status_flag;
    }else {
        max6650_data_t tmp_placeholder = { //Mock Data
            .fan_rpm = 1000, //Mock Data
			      .status_flag = HAL_OK //Mock Data
        };
        raw_data_placeholder.fan_rpm = tmp_placeholder.fan_rpm;
        raw_data_placeholder.status_flag = tmp_placeholder.status_flag;
    }
    uart_tx_logging_t temp_uart_data_placeholder = create_uart_tx_packet(&raw_data_placeholder);
    uint8_t encoded_tx_buffer[64];
    size_t encoded_tx_size = encode_uart_tx_packet(&temp_uart_data_placeholder, encoded_tx_buffer, sizeof(encoded_tx_buffer));
    uint8_t original_data[sizeof(uart_tx_logging_t) + sizeof(uint16_t)];
    memcpy(original_data, &temp_uart_data_placeholder, sizeof(uart_tx_logging_t));
    uint16_t packet_crc = calculate_crc16((uint8_t*)&temp_uart_data_placeholder, sizeof(uart_tx_logging_t));
    memcpy(original_data + sizeof(uart_tx_logging_t), &packet_crc, sizeof(uint16_t));
    bool encoding_valid = verify_cobs_encoding(original_data, sizeof(original_data),encoded_tx_buffer, encoded_tx_size);
    if (encoding_valid){
      LOG_DATA_ENCODED(encoded_tx_buffer,encoded_tx_size);
      NOTIFY_MESSAGE("Verificated Encoding Data\n");
    }else{
      NOTIFY_MESSAGE("Invalid Message Generation, Rejected no verification\n");
    }
    HAL_Delay(500);
    osDelay(1);
  }
};

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	LOG_MESSAGE("Notification\n");
	if (huart -> Instance == Current_Target)
	{
		memcpy(main_buff,rx_buff,Size);
		HAL_UARTEx_ReceiveToIdle_DMA(g_data_logger, rx_buff, rx_buff_size);
		rx_validation = 1;
	}
}
void handle_uart_cmd(uart_rx_command_t* current_command){
  switch(current_command->commandvalue){
    case CFF_ON:
      NOTIFY_MESSAGE("Command Target ON");
      return;
    case CFF_OFF:
      NOTIFY_MESSAGE("Command Target OFF");
      return;
    default :
      NOTIFY_MESSAGE("Command Target AUTO");
      return;
  }
}
void uart_notifications_cycle(void const *argument){
      for(;;)
	  {
	   HAL_UARTEx_ReceiveToIdle_DMA(g_data_logger, (uint8_t *)rx_buff, sizeof(rx_buff));
		__HAL_DMA_DISABLE_IT(g_logger_dma,DMA_IT_HT);
	   if (rx_validation){
		   NOTIFY_MESSAGE("New Message, performing verification...");
		   uart_rx_command_t decoded_rx_command;
		   if (decode_uart_rx_command(main_buff, 8, &decoded_rx_command)){
			   NOTIFY_MESSAGE("Captured Valid rx_Command");
			   handle_uart_cmd(&decoded_rx_command);
		   }else {
			   NOTIFY_MESSAGE("Invalid Message Captured, Rejected no verification");
		   }
		   rx_validation=0;
	   }
	}
    osDelay(1);
}
void perform_setup_threads(UART_HandleTypeDef *data_logger_place_holder,
			UART_HandleTypeDef *local_monitor_place_holder,
			I2C_HandleTypeDef *driver_iface_placeholder,
			I2C_HandleTypeDef *mux_iface_placeholder,
			DMA_HandleTypeDef *logger_dma_placeholder,
			USART_TypeDef * target_placeholder,
			CAN_HandleTypeDef * can_placeholder){
    g_data_logger = data_logger_place_holder;
    g_local_monitor = local_monitor_place_holder;
    g_driver_iface = driver_iface_placeholder;
    g_mux_iface = mux_iface_placeholder;
    g_logger_dma =logger_dma_placeholder;
    g_can_iface =can_placeholder;
    Current_Target = target_placeholder;
};
