/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   LoRa SX1278 TX-only (poll TxDone) + Watch debug + FIFO snapshot
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h"
#include <string.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Thanh ghi SX1278 dùng cho TX */
#ifndef RegFifo
#define RegFifo              0x00
#endif
#ifndef RegOpMode
#define RegOpMode            0x01
#endif
#ifndef RegFifoAddrPtr
#define RegFifoAddrPtr       0x0D
#endif
#ifndef RegFifoTxBaseAddr
#define RegFifoTxBaseAddr    0x0E
#endif
#ifndef RegIrqFlagsMask
#define RegIrqFlagsMask      0x11
#endif
#ifndef RegIrqFlags
#define RegIrqFlags          0x12
#endif
#ifndef RegPayloadLength
#define RegPayloadLength     0x22
#endif
#ifndef RegDioMapping1
#define RegDioMapping1       0x40
#endif
#ifndef RegVersion
#define RegVersion           0x42
#endif

#define LORA_MODE_SLEEP      0x00
#define LORA_MODE_STDBY      0x01
#define LORA_MODE_TX         0x03
#define LORA_IRQ_TXDONE      0x08
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
LoRa myLoRa;

/* Biến debug để xem trong Watch */
volatile uint8_t  g_ver    = 0;     // RegVersion (0x12 nếu đúng chip)
volatile uint16_t g_init   = 0;     // LoRa_init() (LORA_OK = 0x00C8)
volatile uint8_t  g_txst   = 0;     // 0xC8 OK, 0x01 timeout (quy ước)
volatile uint8_t  g_irq    = 0;     // RegIrqFlags sau TX (bit3 = TxDone)
volatile uint32_t g_tx_cnt = 0;     // số gói TX ok
volatile uint8_t  g_beat   = 0;     // heartbeat LED

/* đảm bảo g_echo không bị tối ưu loại bỏ */
#if defined(__GNUC__) || defined(__clang__)
  #define USED_ATTR __attribute__((used))
#else
  #define USED_ATTR
#endif

volatile uint8_t  g_echo[32] USED_ATTR = {0};  // snapshot FIFO để xem trên Watch
volatile uint8_t  g_fifo_ok = 0;               // so khớp FIFO với TxPayload
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
static void LoRa_ConfigDefaults(LoRa *l)
{
  l->frequency             = 433;        // các thông số an toàn
  l->spredingFactor        = SF_7;
  l->bandWidth             = BW_125KHz;
  l->crcRate               = CR_4_5;
  l->power                 = POWER_17db;
  l->overCurrentProtection = 100;
  l->preamble              = 8;
}

/* Ghi payload vào FIFO khi thư viện không có burst write */
static void fifo_write_bytes(LoRa *l, const uint8_t *buf, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++) {
    LoRa_write(l, RegFifo, buf[i]);
  }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();    // tùy chọn

  /* USER CODE BEGIN 2 */
  // LED PC13 active-low: SET = tắt lúc boot
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  // Khởi tạo đối tượng LoRa + map chân
  myLoRa = newLoRa();
  myLoRa.CS_port    = NSS_GPIO_Port;
  myLoRa.CS_pin     = NSS_Pin;
  myLoRa.reset_port = RST_GPIO_Port;
  myLoRa.reset_pin  = RST_Pin;
  myLoRa.DIO0_port  = DIO0_GPIO_Port;   // TX-only: không cần EXTI
  myLoRa.DIO0_pin   = DIO0_Pin;
  myLoRa.hSPIx      = &hspi1;
  LoRa_ConfigDefaults(&myLoRa);

  // Reset chip -> đọc version (0x12)
  LoRa_reset(&myLoRa);
  HAL_Delay(5);
  g_ver = LoRa_read(&myLoRa, RegVersion);
  if (g_ver != 0x12) { while (1) { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(250); } }

  // Init SX1278
  g_init = LoRa_init(&myLoRa);
  if (g_init != LORA_OK) { while (1) { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(500); } }
  for (uint8_t i=0;i<3;i++){ HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(120); HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(120); }

  // Gói cần gửi
  static const uint8_t TxPayload[] = { 45, 0x12, 'G' };
  const uint8_t TX_LEN = (uint8_t)sizeof(TxPayload);
  const uint8_t FIFO_TX_BASE = 0x80;
  /* USER CODE END 2 */

  /* Infinite loop */
  uint32_t last = HAL_GetTick();
  while (1)
  {
    if (HAL_GetTick() - last >= 1000) {       // gửi mỗi 1 giây
      last += 1000;

      /* Heartbeat: LED + biến g_beat */
      g_beat ^= 1;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, g_beat ? GPIO_PIN_RESET : GPIO_PIN_SET);

      /* ====== TX (poll TxDone) + snapshot FIFO ====== */
      // 1) LoRa Sleep -> Standby
      uint8_t op = LoRa_read(&myLoRa, RegOpMode);
      op = (op & 0x07) | 0x80;                  // LoRa + Sleep
      LoRa_write(&myLoRa, RegOpMode, op);
      HAL_Delay(1);
      op = (op & 0xF8) | LORA_MODE_STDBY;       // Standby
      LoRa_write(&myLoRa, RegOpMode, op);

      // 2) Clear IRQ và unmask
      LoRa_write(&myLoRa, RegIrqFlags, 0xFF);
      LoRa_write(&myLoRa, RegIrqFlagsMask, 0x00);

      // 3) FIFO TX base & con trỏ
      LoRa_write(&myLoRa, RegFifoTxBaseAddr, FIFO_TX_BASE);
      LoRa_write(&myLoRa, RegFifoAddrPtr,    FIFO_TX_BASE);

      // 4) Ghi payload & đặt độ dài
      fifo_write_bytes(&myLoRa, TxPayload, TX_LEN);
      LoRa_write(&myLoRa, RegPayloadLength, TX_LEN);

      /* --- SNAPSHOT FIFO về g_echo để xem trên Watch --- */
      LoRa_write(&myLoRa, RegFifoAddrPtr, FIFO_TX_BASE);       // về đầu FIFO TX
      for (uint8_t i = 0; i < TX_LEN && i < sizeof(g_echo); i++) {
        g_echo[i] = LoRa_read(&myLoRa, RegFifo);               // đọc ra RAM
      }
      g_fifo_ok = (memcmp((const void*)g_echo, TxPayload, TX_LEN) == 0);
      // KHÔI PHỤC con trỏ để TX sử dụng
      LoRa_write(&myLoRa, RegFifoAddrPtr, FIFO_TX_BASE);
      /* --- HẾT SNAPSHOT --- */

      // 5) Map DIO0 = TxDone (b7..b6=01)
      uint8_t map1 = LoRa_read(&myLoRa, RegDioMapping1);
      map1 = (map1 & 0x3F) | 0x40;
      LoRa_write(&myLoRa, RegDioMapping1, map1);

      // 6) Vào TX mode
      op = LoRa_read(&myLoRa, RegOpMode);
      op = (op & 0xF8) | LORA_MODE_TX;
      LoRa_write(&myLoRa, RegOpMode, op);

      // 7) Đợi TxDone hoặc timeout (~300 ms)
      uint32_t t0 = HAL_GetTick();
      uint8_t irq;
      do {
        irq = LoRa_read(&myLoRa, RegIrqFlags);
      } while (((irq & LORA_IRQ_TXDONE) == 0) && (HAL_GetTick() - t0 < 300));

      g_irq = irq;
      if (irq & LORA_IRQ_TXDONE) {
        g_txst = 0xC8;                           // OK
        g_tx_cnt++;
        LoRa_write(&myLoRa, RegIrqFlags, 0xFF);  // clear flags
      } else {
        g_txst = 0x01;                           // timeout / fail
      }
      /* ====== HẾT TX ====== */
    }
  }
}

/* System Clock 64 MHz (x8). x9=72 MHz cũng chạy tốt */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType   = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState         = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue   = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState         = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState     = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource    = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL       = RCC_PLL_MUL8;   // 64 MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   // ≤36 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  (void)GPIO_Pin; // TX-only: không dùng EXTI
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
