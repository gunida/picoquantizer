#include "constants.h"
#include "dac4911.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/time.h"
#include <cmath>
#include <pico/stdio.h>

static float
    VOLTAGES[NUM_PIANO_KEYS]; // Voltages of each actual note starting from 0V
dma_channel_config cfg;
uint dma_chan;
uint8_t cap_buf[NSAMP];
uint16_t defined_scale;
static int NOTE_PINS[12] = {NOTE_PIN_01, NOTE_PIN_02, NOTE_PIN_03, NOTE_PIN_04,
                            NOTE_PIN_05, NOTE_PIN_06, NOTE_PIN_07, NOTE_PIN_08,
                            NOTE_PIN_09, NOTE_PIN_10, NOTE_PIN_11, NOTE_PIN_12};

const float conversion_factor =
    VOLT_MAX / (1 << 8); // 256 bit, for DMA ADC conversion

void generateVoltages(float *voltages) {
  for (int i = 0; i < NUM_PIANO_KEYS; i++) {
    float volt = VOLT_PER_SEMITONE * i;
    voltages[i] = volt;
  }
}

void configure_scale() {
  // Temporary reset
  defined_scale = 0b111111111111;

  // Read all note pins in order and set the scale config
  for (uint i = 0; i < 12; i++) {
    defined_scale = defined_scale ^ ((gpio_get(NOTE_PINS[i]) ? 0 : 1) << i);
  }
}

uint map_note_to_scale(uint note_idx) {
  uint result_idx = note_idx;
  uint scale_note = note_idx % 12;
  uint16_t note_mask = 1 << scale_note;

  // Check if the note is outside of the configured scale
  if ((note_mask & defined_scale) != note_mask) {
    int starting_point = note_idx;

    if (note_idx < 12) {
      // step up chromatically until we hit a note
      for (int i = note_idx; i < starting_point + 12; i++) {
        note_mask =
            1 << (i % 12); // Bit shift to mask the correct note according
                           // to defined_scale, wraps around to 12 at 0
        if ((note_mask & defined_scale) == note_mask) {
          result_idx = i;
          break;
        }
      }
    } else {
      // From the current note, step down chromatically, twelve semitones, and
      // find the next turned on note
      for (int i = starting_point; i >= starting_point - 12; i--) {
        note_mask =
            1 << (i % 12); // Bit shift to mask the correct note according
                           // to defined_scale, wraps around to 12 at 0
        if ((note_mask & defined_scale) == note_mask) {
          result_idx = i;
          break;
        }
      }
    }
  }

  return result_idx;
}

// free-running sample
void sample(uint8_t *capture_buf, uint adc_channel) {
  adc_select_input(adc_channel);

  adc_fifo_drain();
  adc_run(false);

  dma_channel_configure(dma_chan, &cfg,
                        capture_buf,   // dst
                        &adc_hw->fifo, // src
                        NSAMP,         // transfer count
                        true           // start immediately
  );

  gpio_put(LED_PIN, 1);
  adc_run(true);
  dma_channel_wait_for_finish_blocking(dma_chan);

  gpio_put(LED_PIN, 0);
}

// Samples ADC and writes to DAC
void quantizer(spi_inst_t *spi) {
  float adc_voltage; // Average value of samples

  uint cap_channel =
      spi == SPI_A_PORT ? ADC_CAPTURE_CHANNEL_1 : ADC_CAPTURE_CHANNEL_2;

  sleep_ms(10); // sleep a little to let the CV stabilize

  // Capture input voltage
  sample(cap_buf, cap_channel);
  uint64_t sum = 0;
  for (int i = 0; i < NSAMP; i++) {
    sum += cap_buf[i];
  }
  float avg = (float)sum / NSAMP;
  adc_voltage = avg / INPUT_VOLTAGE_DIVISION * conversion_factor;

  // Find nearest output voltage
  uint quantized_idx = floor(adc_voltage / VOLT_PER_SEMITONE);

  // quantize the output voltage
  quantized_idx = map_note_to_scale(quantized_idx);

  uint output_channel = spi == SPI_A_PORT ? 0 : 1;
  DAC_write(output_channel, VOLTAGES[quantized_idx]);
}

void setup_dma() {
  // Set up the DMA to start transferring data as soon as it appears in FIFO
  dma_chan = dma_claim_unused_channel(true);
  cfg = dma_channel_get_default_config(dma_chan);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);
}

void setup_adc() {
  adc_gpio_init(26 + ADC_CAPTURE_CHANNEL_1);
  adc_gpio_init(26 + ADC_CAPTURE_CHANNEL_2);

  adc_init();
  adc_fifo_setup(
      true,  // Write each completed conversion to the sample FIFO
      true,  // Enable DMA data request (DREQ)
      1,     // DREQ (and IRQ) asserted when at least 1 sample present
      false, // We won't see the ERR bit because of 8 bit reads; disable.
      true   // Shift each sample to 8 bits when pushing to FIFO
  );

  // set sample rate
  adc_set_clkdiv(CLOCK_DIV);
}

void setup_gpio() {
  stdio_init_all();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  gpio_init(GATE_PIN_A);
  gpio_set_dir(GATE_PIN_A, GPIO_IN);
  gpio_pull_up(GATE_PIN_A);
  gpio_init(GATE_PIN_B);
  gpio_set_dir(GATE_PIN_B, GPIO_IN);
  gpio_pull_up(GATE_PIN_B);

  for (uint i = 0; i < 12; i++) {
    gpio_init(NOTE_PINS[i]);
    gpio_set_dir(NOTE_PINS[i], GPIO_IN);
    gpio_pull_up(NOTE_PINS[i]);
  }
}

// Triggered on rising edge of GATE_PIN
// Quantizes incoming CV on both inputs
void gpio_callback(uint gpio, uint32_t events) {
  if (gpio == GATE_PIN_A) {
    if (defined_scale != 0)
      quantizer(SPI_A_PORT);
  }
  if (gpio == GATE_PIN_B) {
    if (defined_scale != 0)
      quantizer(SPI_B_PORT);
  }

  if (gpio != GATE_PIN_A && gpio != GATE_PIN_B) {
    // Note config change
    configure_scale();
  }
}

void setup_irq() {
  gpio_set_irq_enabled(GATE_PIN_A, GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(GATE_PIN_B, GPIO_IRQ_EDGE_FALL, true);

  for (uint i = 0; i < 12; i++) {
    gpio_set_irq_enabled(NOTE_PINS[i], GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
                         true);
  }

  gpio_set_irq_callback(&gpio_callback);
  irq_set_enabled(IO_IRQ_BANK0, true);
}

int main() {
  setup_gpio();
  setup_adc();
  sleep_ms(1000); // bit of sleep between ADC and DMA setup
  setup_dma();
  setup_dac(SPI_A_PORT, OUT_A_SCK, OUT_A_SDI, OUT_A_CS, OUT_A_LDAC);
  setup_dac(SPI_B_PORT, OUT_B_SCK, OUT_B_SDI, OUT_B_CS, OUT_B_LDAC);
  sleep_ms(1000);

  generateVoltages(VOLTAGES);
  setup_irq();

  // Startup check of selected scale notes
  configure_scale();

  while (true) {
    sleep_ms(1);
  }
}