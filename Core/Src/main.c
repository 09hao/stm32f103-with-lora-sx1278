/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   LoRa SX1278 TX-only + USB CDC log (non-blocking) + Watch vars
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h"
#include "usbd_cdc_if.h"     // USB CDC
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* SX1278 registers we use */
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

/* Watch/debug variables */
volatile uint8_t  g_ver    = 0;      // RegVersion, expect 0x12
volatile uint16_t g_init   = 0;      // LoRa_init() status (LORA_OK=0x00C8)
volatile uint8_t  g_txst   = 0;      // 0xC8 OK, 0x01 timeout (convention)
volatile uint8_t  g_irq    = 0;      // RegIrqFlags after TX (bit3=TxDone)
volatile uint32_t g_tx_cnt = 0;      // TX success counter
volatile uint8_t  g_beat   = 0;      // heartbeat LED state

/* Snapshot FIFO (for Watch) */
volatile uint8_t  g_echo[32] = {0};  // first bytes of FIFO payload
volatile uint8_t  g_fifo_ok  = 0;    // 1 if matches RAM payload

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */

/* ---------- USB CDC helpers (NON-BLOCKING) ---------- */
extern USBD_HandleTypeDef hUsbDeviceFS;

static inline uint8_t USB_IsConfigured(void)
{
  return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}

/* Try to send once; if BUSY or not configured => just skip (no blocking) */
static void CDC_TrySend(const uint8_t *data, uint16_t len)
{
  if (!USB_IsConfigured()) return;
  (void)CDC_Transmit_FS((uint8_t*)data, len);  // ignore USBD_BUSY
}

/* Pretty print one TX line to Hercules */
static void CDC_SendTxLine(const uint8_t *payload, uint8_t len,
                           uint32_t seq, uint8_t ok, uint8_t irq)
{
  char line[128];
  int n = snprintf(line, sizeof(line), "TX #%lu: ", (unsigned long)seq);
  for (uint8_t i = 0; i < len && n < (int)sizeof(line)-4; i++) {
    n += snprintf(&line[n], sizeof(line)-n, "%02X ", payload[i]);
  }
  n += snprintf(&line[n], sizeof(line)-n, "[%s]  irq=0x%02X\r\n",
                ok ? "OK" : "FAIL", irq);
  if (n > 0) CDC_TrySend((const uint8_t*)line, (uint16_t)n);
}

/* ---------- Small utilities ---------- */
static void Blink_OK(uint8_t n) {
  for (uint8_t i=0;i<n;i++){
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(120);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(120);
  }
}
static void Blink_ERR(void) {
  while (1) { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(500); }
}

static void LoRa_ConfigDefaults(LoRa *l)
{
  l->frequency             = 433;
  l->spredingFactor        = SF_7;
  l->bandWidth             = BW_125KHz;
  l->crcRate               = CR_4_5;
  l->power                 = POWER_17db;
  l->overCurrentProtection = 100;
  l->preamble              = 8;
}

/* Write payload bytes into FIFO (standby) */
static void fifo_write_bytes(LoRa *l, const uint8_t *buf, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++) {
    LoRa_write(l, RegFifo, buf[i]);
  }
}

/* Read back FIFO bytes (standby) — for Watch */
static void fifo_read_back(LoRa *l, uint8_t base, uint8_t *out, uint8_t len)
{
  LoRa_write(l, RegFifoAddrPtr, base);
  for (uint8_t i = 0; i < len; i++) {
    out[i] = LoRa_read(l, RegFifo);
  }
}
/* USER CODE END 0 */

int main(void)
{
  /* Reset & HAL */
  HAL_Init();
  SystemClock_Config();

  /* Init BSP/HAL drivers */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();    // optional
  MX_USB_DEVICE_Init();     // USB CDC (non-blocking helpers will check state)

  /* LED off at boot (PC13 active-low) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /* LoRa object & pins */
  myLoRa = newLoRa();
  myLoRa.CS_port    = NSS_GPIO_Port;
  myLoRa.CS_pin     = NSS_Pin;
  myLoRa.reset_port = RST_GPIO_Port;
  myLoRa.reset_pin  = RST_Pin;
  myLoRa.DIO0_port  = DIO0_GPIO_Port;   // not using EXTI for TX-only
  myLoRa.DIO0_pin   = DIO0_Pin;
  myLoRa.hSPIx      = &hspi1;
  LoRa_ConfigDefaults(&myLoRa);

  /* Reset & check version */
  LoRa_reset(&myLoRa);
  HAL_Delay(5);
  g_ver = LoRa_read(&myLoRa, RegVersion);
  if (g_ver != 0x12) Blink_ERR();

  /* Init radio */
  g_init = LoRa_init(&myLoRa);
  if (g_init != LORA_OK) Blink_ERR();
  Blink_OK(3);

  /* Payload (<= sizeof(g_echo)) */
  static uint8_t TxPayload[] = { 0x2D, 0x12, 0x47 }; // "2D 12 47"
  const uint8_t TX_LEN = (uint8_t)sizeof(TxPayload);
  const uint8_t FIFO_TX_BASE = 0x80;

  uint32_t last = HAL_GetTick();
  while (1)
  {
    /* Send once per second */
    if (HAL_GetTick() - last >= 1000) {
      last += 1000;

      /* Heartbeat LED */
      g_beat ^= 1;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, g_beat ? GPIO_PIN_RESET : GPIO_PIN_SET);

      /* --- Prepare TX (Sleep->Standby) --- */
      uint8_t op = LoRa_read(&myLoRa, RegOpMode);
      op = (op & 0x07) | 0x80;                   // LoRa + Sleep
      LoRa_write(&myLoRa, RegOpMode, op);
      HAL_Delay(1);
      op = (op & 0xF8) | LORA_MODE_STDBY;        // Standby
      LoRa_write(&myLoRa, RegOpMode, op);

      /* Clear IRQs and unmask */
      LoRa_write(&myLoRa, RegIrqFlags, 0xFF);
      LoRa_write(&myLoRa, RegIrqFlagsMask, 0x00);

      /* FIFO base & pointer */
      LoRa_write(&myLoRa, RegFifoTxBaseAddr, FIFO_TX_BASE);
      LoRa_write(&myLoRa, RegFifoAddrPtr,    FIFO_TX_BASE);

      /* Write payload & length */
      fifo_write_bytes(&myLoRa, TxPayload, TX_LEN);
      LoRa_write(&myLoRa, RegPayloadLength, TX_LEN);

      /* Read back a snapshot for Watch */
      uint8_t echo[sizeof(TxPayload)] = {0};
      fifo_read_back(&myLoRa, FIFO_TX_BASE, echo, TX_LEN);
      for (uint8_t i=0; i<TX_LEN && i<sizeof(g_echo); i++) g_echo[i] = echo[i];
      g_fifo_ok = (memcmp(echo, TxPayload, TX_LEN) == 0) ? 1 : 0;

      /* Restore pointer before TX */
      LoRa_write(&myLoRa, RegFifoAddrPtr, FIFO_TX_BASE);

      /* Map DIO0 = TxDone (for future EXTI usage) */
      uint8_t map1 = LoRa_read(&myLoRa, RegDioMapping1);
      map1 = (map1 & 0x3F) | 0x40;
      LoRa_write(&myLoRa, RegDioMapping1, map1);

      /* Enter TX mode */
      op = LoRa_read(&myLoRa, RegOpMode);
      op = (op & 0xF8) | LORA_MODE_TX;
      LoRa_write(&myLoRa, RegOpMode, op);

      /* Wait TxDone or timeout (~300 ms) */
      uint32_t t0 = HAL_GetTick();
      uint8_t irq;
      do {
        irq = LoRa_read(&myLoRa, RegIrqFlags);
      } while (((irq & LORA_IRQ_TXDONE) == 0) && (HAL_GetTick() - t0 < 300));

      g_irq = irq;
      if (irq & LORA_IRQ_TXDONE) {
        g_txst = 0xC8;                // OK (convention)
        g_tx_cnt++;
        LoRa_write(&myLoRa, RegIrqFlags, 0xFF);  // clear flags
      } else {
        g_txst = 0x01;                // timeout / fail
      }

      /* USB CDC log — NON-BLOCKING (will not stall main loop) */
      CDC_SendTxLine((const uint8_t*)g_echo, TX_LEN, g_tx_cnt,
                     (g_txst == 0xC8), g_irq);
    }
  }
}

/* Clock = 72 MHz, USB clock = 48 MHz (PLL/1.5) */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;       // 72 MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;      // ≤36 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }

  /* USB clock 48 MHz = PLL / 1.5 */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection    = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  (void)GPIO_Pin; // not used
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
