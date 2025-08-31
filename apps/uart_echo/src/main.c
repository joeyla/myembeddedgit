
/*
===============================================================================
Module:        apps.uart_echo
Description:   Minimal echo app wiring UART via interface
Author:        Starter Repo
Created:       2025-08-28
===============================================================================
*/
/* Includes ------------------------------------------------------------------*/
#include "core/err.h"
#include "core/log.h"
#include "core/uart_iface.h"
#include "at_engine.h"
#include "core/clock_iface.h"
#include "board.h"

#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_pwr_ex.h"
#include "stm32f4xx_hal_flash_ex.h"

/* Defines -------------------------------------------------------------------*/
#define MODULE_ID  0x601
#ifdef MODULE_TAG
#undef MODULE_TAG
#endif
#define MODULE_TAG "apps.uart_echo"

void SystemClock_Config(void);

uint32_t clock_millis(void)
{
    return HAL_GetTick();
}
/* Public API ----------------------------------------------------------------*/
int main(void){
    
  HAL_Init();
  SystemClock_Config();
    
    uart_config_t cfg = { .baud=115200, .databits=8, .stopbits=1, .parity=0 };
    uart_handle_t* u = 0;
    
    board_init_clocks();
    board_init_gpio();
    
    
    status_t st = uart_open(0, &cfg, &u);
    if(st < 0){
        LOGE("uart_open failed"); log_status(st);
        for(;;);
    }

    at_engine_t eng; at_engine_init(&eng, u);
    at_engine_send(&eng, "UART echo demo ready\r\n");
    (void)uart_set_tx_coalesce_min(u, 32); /* coalesce tiny writes */
    (void)uart_flush(u, 100); /* ensure greeting is out */

    for(;;){
        uint8_t buf[64];
        uint16_t n = 0;
        st = uart_read(u, buf, sizeof buf, &n);
        if(st < 0){
            LOGE("uart_read failed"); log_status(st);
        } else if(n){
            (void)uart_write(u, buf, n, 0, 0);
            (void)uart_flush(u, 10); /* example: keep latency low for interactive echo */
        }
        clock_delay_ms(1);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
//////void SystemClock_Config(void)
//////{
//////  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//////  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//////  /** Configure the main internal regulator output voltage
//////  */
//////  __HAL_RCC_PWR_CLK_ENABLE();
//////  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

//////  /** Initializes the RCC Oscillators according to the specified parameters
//////  * in the RCC_OscInitTypeDef structure.
//////  */
//////  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
//////  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//////  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//////  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//////  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//////  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//////  RCC_OscInitStruct.PLL.PLLM = 16;
//////  RCC_OscInitStruct.PLL.PLLN = 192;
//////  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
//////  RCC_OscInitStruct.PLL.PLLQ = 4;
//////  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//////  {
//////    //Error_Handler();
//////  }

//////  /** Initializes the CPU, AHB and APB buses clocks
//////  */
//////  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//////                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//////  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//////  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//////  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//////  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//////  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
//////  {
//////    //Error_Handler();
//////  }
//////}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef o = {0};
  RCC_ClkInitTypeDef c = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  o.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  o.HSIState = RCC_HSI_ON;
  o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  o.PLL.PLLState = RCC_PLL_OFF;
  HAL_RCC_OscConfig(&o);

  c.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  c.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  c.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  c.APB1CLKDivider = RCC_HCLK_DIV1;
  c.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&c, FLASH_LATENCY_0);
}

