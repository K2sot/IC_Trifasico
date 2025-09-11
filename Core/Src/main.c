/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Optimized Version
  ******************************************************************************
  * OPTIMIZATION NOTES:
  * - Consolidated redundant timestamp variables into unified tracking
  * - Created configuration tables for ADC management to reduce code duplication
  * - Unified ADC callback processing with single function handling all instances
  * - Consolidated error handling with table-driven approach
  * - Simplified frequency and phase calculation logic
  * - Reduced repetitive ADC initialization and monitoring code
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TWO_PI_F  6.28318530718f
#define FS_F      7680.0f     // sampling rate (Hz)
#define F0_HZ     60.0f       // nominal line frequency (Hz)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// --- Defines para buffers e filas ---
#define FS_HZ               7680u
#define CYCLE_SAMPLES       128u          // 1 ciclo @ 60 Hz
#define IEC12C_SAMPLES      1536u         // 12 ciclos (IEC 200 ms)
#define DMA_TRIG_HALF       128u          // amostras por meia-interrupt de DMA
#define FRAME_BYTES         250
#define TXQ_SLOTS           96            // 96*250 = 24.0 kB buffer (ajuste se quiser)

typedef enum { CH_VA=0, CH_VB, CH_VC, CH_IA, CH_IB, CH_IC, CH_IN, CH_COUNT } acq_channel_t;
// Buffers DMA intercalados por ADC (half x2, canais x N)
static uint16_t adc1_dma[2 * 2 * DMA_TRIG_HALF]; // ADC1: Va, Ia
static uint16_t adc2_dma[2 * 2 * DMA_TRIG_HALF]; // ADC2: Vb, Ib
static uint16_t adc3_dma[2 * 2 * DMA_TRIG_HALF]; // ADC3: Vc, Ic
static uint16_t adc4_dma[2 * 1 * DMA_TRIG_HALF]; // ADC4: In

static volatile uint32_t adc1_cb_count = 0;
static volatile uint32_t adc2_cb_count = 0; 
static volatile uint32_t adc3_cb_count = 0;
static volatile uint32_t adc4_cb_count = 0;


typedef struct {
  uint16_t cycle[CYCLE_SAMPLES];
  uint16_t iec12[IEC12C_SAMPLES];
  uint16_t i_cycle, i_iec;
  uint32_t seq_cycle, seq_iec12;
} acq_buf_t;
static acq_buf_t chbuf[CH_COUNT];

// Fila de transmissão UART4 DMA (frames de 200 B)
static uint8_t  txq[TXQ_SLOTS][FRAME_BYTES];
static volatile uint16_t tx_head=0, tx_tail=0;
static volatile uint8_t  tx_dma_busy=0;

// Different headers for different data types and channels
static const uint8_t HDR_CYCLE_VA[2] = {0xAA, 0x01}; // Voltage A - 1 cycle data
static const uint8_t HDR_CYCLE_VB[2] = {0xAA, 0x02}; // Voltage B - 1 cycle data  
static const uint8_t HDR_CYCLE_VC[2] = {0xAA, 0x03}; // Voltage C - 1 cycle data
static const uint8_t HDR_CYCLE_IA[2] = {0xAA, 0x11}; // Current A - 1 cycle data
static const uint8_t HDR_CYCLE_IB[2] = {0xAA, 0x12}; // Current B - 1 cycle data
static const uint8_t HDR_CYCLE_IC[2] = {0xAA, 0x13}; // Current C - 1 cycle data
static const uint8_t HDR_CYCLE_IN[2] = {0xAA, 0x1F}; // Neutral current - 1 cycle data
static const uint8_t HDR_IEC12[2]    = {0xAA, 0xFF}; // IEC 12-cycle data
static const uint8_t HDR_DEBUG[2]    = {0xAA, 0x00}; // Debug/status messages
static const uint8_t HDR_FREQ_INFO[2] = {0xAA, 0xF0}; // Frequency and timing info

static uint16_t seq16[CH_COUNT] = {0}; // todos os elementos inicializados com zero

// Unified timestamp tracking - removed redundant variables
static volatile uint32_t sample_tick = 0;           // Global sample counter (main reference)
static volatile uint32_t cycle_start_sample = 0;    // Sample when cycle starts
static volatile uint8_t  tick_overflow = 0;         // Overflow flag
static uint16_t freq_info_seq = 0;                  // Frequency info sequence


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;
DMA_HandleTypeDef hdma_adc4;

CORDIC_HandleTypeDef hcordic;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC4_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

void TIM1_Config_TRGO_7680Hz(void);   // reconfigure TIM1 to 7.68 kHz TRGO

// PLL API
typedef struct {
  float kp, ki;
  float phase;        // [0, 2π)
  float omega;        // rad/sample
  float omega_nom;    // 2π * f0 / Fs
  float integ;        // integrator state
  float e_filt;       // LPF of phase detector
  float beta;         // LPF coeff ~ 2π*BW/Fs
  float omega_min;    // clamp
  float omega_max;    // clamp
} pll_t;

void pll_init(pll_t* p, float fs, float f0, float kp, float ki, float loop_bw_hz, float freq_pct);
bool pll_step(pll_t* p, float v_norm, float* out_phase);

// helpers
static inline float pll_inst_freq_hz(const pll_t* p) { return (p->omega * FS_F) / TWO_PI_F; }

// --- ADC → volts, then to per-unit (±1 if full-scale) ---
#define ADC_FULLSCALE_COUNTS 4095.0f
#define VREF_F                3.3f
#define VMID_F                1.65f      // calibrate per channel if needed

static inline float adc12_to_volts(uint16_t raw12) {
    return ( (float)raw12 * (VREF_F / ADC_FULLSCALE_COUNTS) );
}

// If you have a better measured mid-rail (per channel), pass it in instead of VMID_F
static inline float volts_to_norm(float v, float v_mid) {
    return (v - v_mid) / v_mid;          // full-scale sine → ~±1
}

// Drop-in replacement for your current helper
static inline float adc12_to_norm(uint16_t raw12) {
    const float v = adc12_to_volts(raw12);
    return volts_to_norm(v, VMID_F);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ===== CORDIC sine using Q31 ===== */
extern CORDIC_HandleTypeDef hcordic;

static inline float wrap_2pi(float x){
  if (x >= TWO_PI_F) x -= TWO_PI_F;
  if (x <  0.0f)     x += TWO_PI_F;
  return x;
}

static inline float cordic_sin(float angle_rad)
{
  float a = angle_rad;
  if (a >  (float)M_PI) a -= TWO_PI_F;
  if (a <= -(float)M_PI) a += TWO_PI_F;

  int32_t in_q31  = (int32_t)(a * (2147483648.0f / (float)M_PI)); // rad * 2^31/π
  int32_t out_q31 = 0;
  HAL_CORDIC_Calculate(&hcordic, &in_q31, &out_q31, 1, 1);
  return ((float)out_q31) / 2147483648.0f; // ~[-1,1)
}

// You already have cordic_sin(); this gives cos via a +π/2 shift
static inline float cordic_cos(float angle_rad){
  return cordic_sin(angle_rad + (float)M_PI_2);
}

/* ===== PLL implementation ===== */
// fs [Hz], f0 [Hz], kp/ki per-sample gains,
// loop_bw_hz ~ 10..30 for grids, freq_pct e.g. 0.08 (=±8%)
void pll_init(pll_t* p, float fs, float f0, float kp, float ki,
              float loop_bw_hz, float freq_pct)
{
  p->kp = kp; p->ki = ki;
  p->phase = 0.0f;
  p->integ = 0.0f;
  p->e_filt = 0.0f;
  p->omega_nom = TWO_PI_F * (f0 / fs);
  p->omega     = p->omega_nom;
  p->beta      = (float)(2.0 * M_PI) * (loop_bw_hz / fs);  // IIR LPF coeff
  if (p->beta > 0.5f) p->beta = 0.5f;                      // sanity
  float wspan  = p->omega_nom * freq_pct;
  p->omega_min = p->omega_nom - wspan;
  p->omega_max = p->omega_nom + wspan;
}

// returns true on wrap (new cycle)
bool pll_step(pll_t* p, float v_norm, float* out_phase)
{
  float prev = p->phase;

  // Proper synchronous detector (quadrature): e ≈ sin(phase_error)
  float e  = v_norm * cordic_cos(p->phase);

  // LPF the mixer output to remove double-frequency term
  p->e_filt += p->beta * (e - p->e_filt);

  // PI + anti-windup
  p->integ += p->e_filt;
  // clamp integrator (units of "e", not rad)
  if (p->integ >  9.2f) p->integ =  9.2f;
  if (p->integ < -9.2f) p->integ = -9.2f;

  // frequency correction
  p->omega = p->omega_nom + p->kp * p->e_filt + p->ki * p->integ;
  if (p->omega > p->omega_max) p->omega = p->omega_max;
  if (p->omega < p->omega_min) p->omega = p->omega_min;

  // integrate phase
  p->phase = wrap_2pi(p->phase + p->omega);

  if (out_phase) *out_phase = p->phase;
  return (p->phase < prev);
}

/* ===== Global PLLs (debug-visible) ===== */
pll_t pllA;   // use with VA
pll_t pllB;   // use with VB
pll_t pllC;   // use with VC

/* ===== TIM1 → TRGO @ 7.68 kHz =====
   Rebuilds TIM1 with ~1 MHz tick (easy ARR) and TRGO=UPDATE */
void TIM1_Config_TRGO_7680Hz(void)
{
  RCC_ClkInitTypeDef clk_cfg; uint32_t flash_lat;
  HAL_RCC_GetClockConfig(&clk_cfg, &flash_lat);

  // TIM1 is on APB2; timer clock doubles when APB2 prescaler != 1
  uint32_t pclk2  = HAL_RCC_GetPCLK2Freq();
  uint32_t timclk = (clk_cfg.APB2CLKDivider == RCC_HCLK_DIV1) ? pclk2 : (pclk2 * 2U);

  const uint32_t target_tick = 1000000UL; // 1 MHz
  uint32_t psc  = (timclk + target_tick - 1U) / target_tick;
  if (psc == 0U) psc = 1U;
  uint32_t tick = timclk / psc;

  const uint32_t FS = 7680U;
  uint32_t arr = (uint32_t)((tick + (FS/2U)) / FS);
  if (arr < 1U) arr = 1U;

  TIM_ClockConfigTypeDef  sClock = {0};
  TIM_MasterConfigTypeDef sM     = {0};

  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = (uint16_t)(psc  - 1U);
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = (uint32_t)(arr  - 1U);
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }

  sClock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClock) != HAL_OK) { Error_Handler(); }

  sM.MasterOutputTrigger  = TIM_TRGO_UPDATE;
  sM.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sM.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sM) != HAL_OK) { Error_Handler(); }
}

static inline void pack12_2in3(uint16_t a, uint16_t b, uint8_t* dst3)
{
  a &= 0x0FFFu; b &= 0x0FFFu;
  dst3[0] = (uint8_t)(a >> 4);
  dst3[1] = (uint8_t)((a & 0x0Fu) << 4) | (uint8_t)(b >> 8);
  dst3[2] = (uint8_t)(b & 0xFFu);
}

static inline uint16_t crc16_ccitt(const uint8_t* data, uint16_t len)
{
  uint16_t crc = 0xFFFFu;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
  }
  return crc;
}

/* ---- Ring buffer TX UART4 DMA ---- */
extern UART_HandleTypeDef huart4;
static inline uint16_t _inc16(uint16_t x){ return (uint16_t)((x+1u) % TXQ_SLOTS); }
static inline uint8_t  _empty(void){ return tx_head==tx_tail; }
static inline uint8_t  _full(void){ return _inc16(tx_head)==tx_tail; }

static void TX_Kick(void)
{
  __disable_irq();
  if (tx_dma_busy || _empty()) {
    __enable_irq();
    return;
  }
  tx_dma_busy = 1;
  __enable_irq();
  (void)HAL_UART_Transmit_DMA(&huart4, txq[tx_tail], FRAME_BYTES);
}
static void TX_Init(void) { tx_head = tx_tail = 0; tx_dma_busy = 0; }
static uint8_t TX_Push(const uint8_t frame[FRAME_BYTES])
{
  uint8_t ret = 0;
  __disable_irq();
  uint16_t nxt = _inc16(tx_head);
  if (nxt != tx_tail) {
    memcpy(txq[tx_head], frame, FRAME_BYTES);
    tx_head = nxt;
    ret = 1;
  }
  __enable_irq();
  TX_Kick();
  return ret;
}
static void TX_Pump(void) { TX_Kick(); }

// Helper function to get the correct header for each channel type
static const uint8_t* get_channel_header(acq_channel_t ch) {
  switch (ch) {
    case CH_VA: return HDR_CYCLE_VA;
    case CH_VB: return HDR_CYCLE_VB;
    case CH_VC: return HDR_CYCLE_VC;
    case CH_IA: return HDR_CYCLE_IA;
    case CH_IB: return HDR_CYCLE_IB;
    case CH_IC: return HDR_CYCLE_IC;
    case CH_IN: return HDR_CYCLE_IN;
    default:    return HDR_DEBUG;
  }
}

// Pack 32-bit value into 4 bytes (little-endian)
static inline void pack32(uint32_t val, uint8_t* dst) {
  dst[0] = (uint8_t)(val & 0xFF);
  dst[1] = (uint8_t)((val >> 8) & 0xFF);
  dst[2] = (uint8_t)((val >> 16) & 0xFF);
  dst[3] = (uint8_t)((val >> 24) & 0xFF);
}

// Pack float as 32-bit IEEE754 (little-endian)
static inline void pack_float(float val, uint8_t* dst) {
  union { float f; uint32_t u; } conv = { .f = val };
  pack32(conv.u, dst);
}

// Send frequency and timing information frame
static void send_freq_info(void) {
  uint8_t frame[FRAME_BYTES];
  uint16_t pos = 0;
  
  // Header
  frame[pos++] = HDR_FREQ_INFO[0]; 
  frame[pos++] = HDR_FREQ_INFO[1];
  
  // Current sample tick (4 bytes) - for synchronization
  pack32(sample_tick, &frame[pos]); pos += 4;
  
  // Overflow status (1 byte)
  frame[pos++] = tick_overflow;
  
  // Frequency measurements from all 3 PLLs (3 × 4 bytes = 12 bytes)
  pack_float(pll_inst_freq_hz(&pllA), &frame[pos]); pos += 4;
  pack_float(pll_inst_freq_hz(&pllB), &frame[pos]); pos += 4;
  pack_float(pll_inst_freq_hz(&pllC), &frame[pos]); pos += 4;
  
  // PLL phase information (3 × 4 bytes = 12 bytes)  
  pack_float(pllA.phase, &frame[pos]); pos += 4;
  pack_float(pllB.phase, &frame[pos]); pos += 4;
  pack_float(pllC.phase, &frame[pos]); pos += 4;
  
  // Timestamp information (2 × 4 bytes = 8 bytes) - simplified
  pack32(sample_tick, &frame[pos]); pos += 4;
  pack32(cycle_start_sample, &frame[pos]); pos += 4;
  pack32(HAL_GetTick(), &frame[pos]); pos += 4; // System tick (ms)
  
  // ADC callback counts for diagnostics (4 × 4 bytes = 16 bytes)
  pack32(adc1_cb_count, &frame[pos]); pos += 4;
  pack32(adc2_cb_count, &frame[pos]); pos += 4;
  pack32(adc3_cb_count, &frame[pos]); pos += 4;
  pack32(adc4_cb_count, &frame[pos]); pos += 4;
  
  // Fill remaining space with system status info
  while (pos < FRAME_BYTES - 6) {
    frame[pos++] = 0x00; // Reserved for future use
  }
  
  // Sequence number
  frame[pos++] = (uint8_t)(freq_info_seq & 0xFF);
  frame[pos++] = (uint8_t)(freq_info_seq >> 8);
  freq_info_seq++;
  
  // Channel ID and reserved
  frame[pos++] = 0xF0; // Special ID for frequency info
  frame[pos++] = 0x00; // Reserved
  
  // CRC
  uint16_t crc = crc16_ccitt(frame, pos);
  frame[pos++] = (uint8_t)(crc >> 8);
  frame[pos++] = (uint8_t)(crc & 0xFF);
  
  if (pos == FRAME_BYTES) (void)TX_Push(frame);
}

/* ---- Aquisição ---- */
extern ADC_HandleTypeDef hadc1, hadc2, hadc3, hadc4;
extern TIM_HandleTypeDef htim1;

static inline void push_sample(acq_channel_t ch, uint16_t s, uint8_t advance_tick)
{
  acq_buf_t* b = &chbuf[ch];
  s &= 0x0FFFu;
  
  // Advance global tick only once per TIM1 trigger (not per channel)
  if (advance_tick) {
    uint32_t prev = sample_tick;
    sample_tick++;
    if (sample_tick == 0 && prev == 0xFFFFFFFFu) tick_overflow = 1;
  }

  // Step the PLL only on voltage channels
  if (ch == CH_VA) {
    float v = adc12_to_norm(s);
    (void)pll_step(&pllA, v, NULL);
  } else if (ch == CH_VB) {
    float v = adc12_to_norm(s);
    (void)pll_step(&pllB, v, NULL);
  } else if (ch == CH_VC) {
    float v = adc12_to_norm(s);
    (void)pll_step(&pllC, v, NULL);
  }

  // Janela de 1 ciclo (128)
  b->cycle[b->i_cycle++] = s;
  if (b->i_cycle >= CYCLE_SAMPLES) {
    b->i_cycle = 0; b->seq_cycle++;
    
    // Record sample number when cycle starts
    cycle_start_sample = sample_tick;

    // Monta frame 250 B deste canal e envia com header específico do canal
    uint8_t frame[FRAME_BYTES]; uint16_t pos=0;
    const uint8_t* hdr = get_channel_header(ch);
    frame[pos++] = hdr[0]; frame[pos++] = hdr[1];
    
    // Pack sample number at cycle start (4 bytes) - primary identifier
    pack32(cycle_start_sample, &frame[pos]); pos += 4;
    
    // Pack overflow flag (1 byte) - critical synchronization info
    frame[pos++] = tick_overflow;
    if (tick_overflow) {
      tick_overflow = 0; // Clear flag after including in frame
    }
    
    // Pack timestamp for synchronization (4 bytes)
    pack32(cycle_start_sample, &frame[pos]); pos += 4;
    
    // Pack all 128 samples (128 samples = 192 bytes when packed 2-in-3)
    for (uint16_t i = 0; i < CYCLE_SAMPLES; i += 2) {
      pack12_2in3(b->cycle[i], b->cycle[i+1], &frame[pos]); pos += 3;
    }
    
    // Get frequency for this channel
    float channel_freq;
    if (ch == CH_VA) {
      channel_freq = pll_inst_freq_hz(&pllA);
    } else if (ch == CH_VB) {
      channel_freq = pll_inst_freq_hz(&pllB);
    } else if (ch == CH_VC) {
      channel_freq = pll_inst_freq_hz(&pllC);
    } else {
      // For current channels, use average frequency
      channel_freq = (pll_inst_freq_hz(&pllA) + pll_inst_freq_hz(&pllB) + pll_inst_freq_hz(&pllC)) / 3.0f;
    }
    pack_float(channel_freq, &frame[pos]); pos += 4;
    
    // Add phase information (4 bytes)
    float channel_phase = 0.0f;
    if (ch == CH_VA) channel_phase = pllA.phase;
    else if (ch == CH_VB) channel_phase = pllB.phase;  
    else if (ch == CH_VC) channel_phase = pllC.phase;
    pack_float(channel_phase, &frame[pos]); pos += 4;
    
    // Fill remaining space before footer
    while (pos < FRAME_BYTES - 6) {
      frame[pos++] = 0x00; // Reserved for future use
    }
    
    uint16_t sseq = seq16[ch]++;
    frame[pos++] = (uint8_t)(sseq & 0xFF);
    frame[pos++] = (uint8_t)(sseq >> 8);
    frame[pos++] = (uint8_t)ch; // Channel ID in reserved byte
    frame[pos++] = 0x00; // reserved
    uint16_t crc = crc16_ccitt(frame, pos);
    frame[pos++] = (uint8_t)(crc >> 8);
    frame[pos++] = (uint8_t)(crc & 0xFF);
    if (pos == FRAME_BYTES) (void)TX_Push(frame);
    
    // Send frequency info every 10 cycles (roughly every 167ms at 60Hz)
    static uint16_t freq_counter = 0;
    if (ch == CH_VA && (++freq_counter >= 10)) {
      freq_counter = 0;
      send_freq_info();
    }
  }

  // Janela IEC (12 ciclos, 1536) — opcional: use no PC
  b->iec12[b->i_iec++] = s;
  if (b->i_iec >= IEC12C_SAMPLES) {
    b->i_iec = 0; b->seq_iec12++;
    // poderia sinalizar aqui se quiser exportar blocos de 200 ms
  }
}

static void demux_block(const uint16_t* blk, uint16_t samples_per_half,
                        const acq_channel_t* map, uint8_t nmap)
{
  for (uint16_t i = 0; i < samples_per_half; i++) {
    // Advance tick with first channel only (once per sampling instant)
    push_sample(map[0], blk[i*nmap + 0], 1);
    // Process remaining channels without advancing tick
    for (uint8_t k = 1; k < nmap; k++) {
      push_sample(map[k], blk[i*nmap + k], 0);
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_CORDIC_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC4_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  // Calibrate all ADCs in a loop
  ADC_HandleTypeDef* adcs[] = {&hadc1, &hadc2, &hadc3, &hadc4};
  for (int i = 0; i < 4; i++) {
    HAL_ADCEx_Calibration_Start(adcs[i], ADC_SINGLE_ENDED);
  }

  // Reconfigure TIM1 to exact 7.68 kHz TRGO (overrides CubeMX defaults)
  TIM1_Config_TRGO_7680Hz();

  // Inicia fila TX
  TX_Init();

  // Send a simple test message first to verify UART is working
  uint8_t test_msg[] = "UART4 Test - STM32 Starting...\r\n";
  HAL_UART_Transmit(&huart4, test_msg, sizeof(test_msg)-1, 1000);

  // Start the trigger timer first
  HAL_TIM_Base_Start(&htim1);

  // Start ADC DMAs with unified configuration
  HAL_StatusTypeDef adc_status[4];
  const uint16_t dma_sizes[] = {2u * 2u * DMA_TRIG_HALF, 2u * 2u * DMA_TRIG_HALF, 
                                2u * 2u * DMA_TRIG_HALF, 2u * 1u * DMA_TRIG_HALF};
  uint32_t* dma_buffers[] = {(uint32_t*)adc1_dma, (uint32_t*)adc2_dma, 
                             (uint32_t*)adc3_dma, (uint32_t*)adc4_dma};
  
  for (int i = 0; i < 4; i++) {
    adc_status[i] = HAL_ADC_Start_DMA(adcs[i], dma_buffers[i], dma_sizes[i]);
  }
  
  // Send unified debug info
  uint8_t debug_msg[120];
  sprintf((char*)debug_msg, "ADC Start Status: 1=%d 2=%d 3=%d 4=%d\r\n", 
          adc_status[0], adc_status[1], adc_status[2], adc_status[3]);
  HAL_UART_Transmit(&huart4, debug_msg, strlen((char*)debug_msg), 1000);
  
  // Also send ADC register status for debugging
  sprintf((char*)debug_msg, "ADC1_CFGR=0x%08lX ADC2_CFGR=0x%08lX\r\n", ADC1->CFGR, ADC2->CFGR);
  HAL_UART_Transmit(&huart4, debug_msg, strlen((char*)debug_msg), 1000);
  sprintf((char*)debug_msg, "ADC3_CFGR=0x%08lX ADC4_CFGR=0x%08lX\r\n", ADC3->CFGR, ADC4->CFGR);
  HAL_UART_Transmit(&huart4, debug_msg, strlen((char*)debug_msg), 1000);

  // Init the three PLLs (visible in debugger; you can tweak kp/ki live)
  // Start conservative; you can increase kp a bit if lock is sluggish
  const float KP = 0.023137f;      // proportional (per-sample)
  const float KI = 0.00026773f;   // integral (per-sample)
  const float BW = 20.0f;        // loop LPF bandwidth [Hz]
  const float FPCT = 0.08f;      // allow ±8% freq excursion

  pll_init(&pllA, FS_F, F0_HZ, KP, KI, BW, FPCT);
  pll_init(&pllB, FS_F, F0_HZ, KP, KI, BW, FPCT);
  pll_init(&pllC, FS_F, F0_HZ, KP, KI, BW, FPCT);

  // Test manual software trigger to see which ADCs respond
  HAL_Delay(1000);  // Wait a bit
  uint8_t test_trigger[] = "Testing software triggers...\r\n";
  HAL_UART_Transmit(&huart4, test_trigger, strlen((char*)test_trigger), 1000);
  
  // Reset counters
  
  // Manual software trigger test (simplified)
  for (int i = 0; i < 4; i++) {
    HAL_ADC_Start(adcs[i]); 
    HAL_ADC_PollForConversion(adcs[i], 100); 
    HAL_ADC_Stop(adcs[i]);
  }
  
  sprintf((char*)debug_msg, "Manual test - ADC values: 1=%d 2=%d 3=%d 4=%d\r\n", 
          (int)HAL_ADC_GetValue(&hadc1), (int)HAL_ADC_GetValue(&hadc2), 
          (int)HAL_ADC_GetValue(&hadc3), (int)HAL_ADC_GetValue(&hadc4));
  HAL_UART_Transmit(&huart4, debug_msg, strlen((char*)debug_msg), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t heartbeat = 0;
  static uint32_t last_counts[4] = {0};
  volatile uint32_t* cb_counts[] = {&adc1_cb_count, &adc2_cb_count, &adc3_cb_count, &adc4_cb_count};
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Monitor ADC activity and restart if stopped
    if (++heartbeat >= 500000) {  // Check every ~0.5 seconds
      heartbeat = 0;
      
      // Check if any ADC counters have stopped incrementing
      uint8_t adc_stuck = 0;
      for (int i = 0; i < 4; i++) {
        if (*cb_counts[i] == last_counts[i]) adc_stuck |= (1 << i);
      }
      
      // Send status message
      uint8_t status_msg[150];
      sprintf((char*)status_msg, "ADC Status: 1=%lu 2=%lu 3=%lu 4=%lu Stuck=0x%02X\r\n", 
              *cb_counts[0], *cb_counts[1], *cb_counts[2], *cb_counts[3], adc_stuck);
      HAL_UART_Transmit(&huart4, status_msg, strlen((char*)status_msg), 1000);
      
      // If any ADC is stuck, try to recover
      if (adc_stuck) {
        uint8_t recovery_msg[] = "ADC Recovery: Restarting stuck ADCs...\r\n";
        HAL_UART_Transmit(&huart4, recovery_msg, sizeof(recovery_msg)-1, 1000);
        
        // Restart stuck ADCs
        for (int i = 0; i < 4; i++) {
          if (adc_stuck & (1 << i)) {
            HAL_ADC_Stop_DMA(adcs[i]);
            HAL_Delay(10);
            HAL_ADC_Start_DMA(adcs[i], dma_buffers[i], dma_sizes[i]);
          }
        }
      }
      
      // Update last counts for next check
      for (int i = 0; i < 4; i++) {
        last_counts[i] = *cb_counts[i];
      }
    }

    // Keep TX running
    TX_Pump();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 38;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.GainCompensation = 0;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 1382400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Debug counters to track ADC callbacks

// Unified ADC callback processing
typedef struct {
  ADC_TypeDef* instance;
  uint16_t* dma_buffer;
  volatile uint32_t* cb_count;
  const acq_channel_t* channel_map;
  uint8_t num_channels;
} adc_config_t;

static const adc_config_t adc_configs[4] = {
  {ADC1, adc1_dma, &adc1_cb_count, (const acq_channel_t[]){CH_VA, CH_IA}, 2},
  {ADC2, adc2_dma, &adc2_cb_count, (const acq_channel_t[]){CH_VB, CH_IB}, 2},
  {ADC3, adc3_dma, &adc3_cb_count, (const acq_channel_t[]){CH_VC, CH_IC}, 2},
  {ADC4, adc4_dma, &adc4_cb_count, (const acq_channel_t[]){CH_IN}, 1}
};

static void process_adc_callback(ADC_HandleTypeDef* hadc, uint16_t buffer_offset) {
  for (int i = 0; i < 4; i++) {
    if (hadc->Instance == adc_configs[i].instance) {
      (*adc_configs[i].cb_count)++;
      demux_block(&adc_configs[i].dma_buffer[buffer_offset], 
                  DMA_TRIG_HALF, adc_configs[i].channel_map, adc_configs[i].num_channels);
      return;
    }
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  process_adc_callback(hadc, 0);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint16_t offset = (hadc->Instance == ADC4) ? 1*DMA_TRIG_HALF : 2*DMA_TRIG_HALF;
  process_adc_callback(hadc, offset);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != UART4) return;
  __disable_irq();
  if (!_empty()) { tx_tail = _inc16(tx_tail); }
  tx_dma_busy = 0;
  __enable_irq();
  TX_Kick();
}

// Unified error handling
typedef struct {
  ADC_TypeDef* instance;
  ADC_HandleTypeDef* handle;
  uint32_t* dma_buffer;
  uint16_t buffer_size;
} adc_restart_config_t;

static const adc_restart_config_t restart_configs[4] = {
  {ADC1, &hadc1, (uint32_t*)adc1_dma, 2u * 2u * DMA_TRIG_HALF},
  {ADC2, &hadc2, (uint32_t*)adc2_dma, 2u * 2u * DMA_TRIG_HALF},
  {ADC3, &hadc3, (uint32_t*)adc3_dma, 2u * 2u * DMA_TRIG_HALF},
  {ADC4, &hadc4, (uint32_t*)adc4_dma, 2u * 1u * DMA_TRIG_HALF}
};

static void send_error_msg(const char* prefix, uint32_t instance, uint32_t error_code) {
  uint8_t error_msg[80];
  sprintf((char*)error_msg, "%s Error: Instance=0x%08lX ErrorCode=0x%08lX\r\n", 
          prefix, instance, error_code);
  HAL_UART_Transmit(&huart4, error_msg, strlen((char*)error_msg), 1000);
}

static void restart_adc_by_instance(ADC_TypeDef* instance) {
  for (int i = 0; i < 4; i++) {
    if (restart_configs[i].instance == instance) {
      HAL_ADC_Stop_DMA(restart_configs[i].handle);
      HAL_Delay(10);
      HAL_ADC_Start_DMA(restart_configs[i].handle, restart_configs[i].dma_buffer, 
                        restart_configs[i].buffer_size);
      break;
    }
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  send_error_msg("ADC", (uint32_t)hadc->Instance, hadc->ErrorCode);
  restart_adc_by_instance(hadc->Instance);
}

void HAL_DMA_ErrorCallback(DMA_HandleTypeDef *hdma)
{
  send_error_msg("DMA", (uint32_t)hdma->Instance, hdma->ErrorCode);
  
  // Map DMA channels to ADC instances
  static const struct { DMA_Channel_TypeDef* dma; ADC_TypeDef* adc; } dma_to_adc[] = {
    {DMA1_Channel1, ADC3}, {DMA1_Channel2, ADC1}, 
    {DMA1_Channel3, ADC2}, {DMA1_Channel4, ADC4}
  };
  
  for (int i = 0; i < 4; i++) {
    if (hdma->Instance == dma_to_adc[i].dma) {
      restart_adc_by_instance(dma_to_adc[i].adc);
      break;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
