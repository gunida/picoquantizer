#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"

// PIN INPUT
#define GATE_PIN_A 20
#define GATE_PIN_B 21

#define NOTE_PIN_01 0  // C
#define NOTE_PIN_02 1  // C#
#define NOTE_PIN_03 2  // D
#define NOTE_PIN_04 3  // D#
#define NOTE_PIN_05 4  // E
#define NOTE_PIN_06 5  // F
#define NOTE_PIN_07 6  // F#
#define NOTE_PIN_08 7  // G
#define NOTE_PIN_09 8  // G#
#define NOTE_PIN_10 9  // A
#define NOTE_PIN_11 10 // A#
#define NOTE_PIN_12 11 // B

#define ADC_CAPTURE_CHANNEL_1 0 // 26 + 0
#define ADC_CAPTURE_CHANNEL_2 1 // 26 + 1

// DAC OUTPUT
#define SPI_VMAX 5.0f
#define SPI_A_PORT spi0
#define OUT_A_LDAC 16
#define OUT_A_CS 17
#define OUT_A_SCK 18
#define OUT_A_SDI 19

#define SPI_B_PORT spi1
#define OUT_B_LDAC 12
#define OUT_B_CS 13
#define OUT_B_SCK 14
#define OUT_B_SDI 15

#define LED_PIN 25

#define INPUT_VOLTAGE_DIVISION (0.333)
#define VOLT_MAX 3.3f // Maximum input voltage

#define VOLT_PER_SEMITONE (1.0 / 12.0)
#define FREQ_0V 16.35 // Frequency at 0V is equal to C0
#define NUM_PIANO_KEYS 6 * 12

// set this to determine sample rate
// 96     = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define FSAMP 5000 // Hz
#define CLOCK_DIV (48000000 / FSAMP)

// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly
#define NSAMP 10

static float FREQUENCIES[NUM_PIANO_KEYS]; // Frequencies of each actual note starting from FREQ_0V
static float VOLTAGES[NUM_PIANO_KEYS];    // Voltages of each actual note starting from 0V
dma_channel_config cfg;
uint dma_chan;
uint8_t cap_buf[NSAMP];
static char event_str[128];
uint16_t defined_scale;

const float conversion_factor = VOLT_MAX / (1 << 8); // 256 bit, for DMA ADC conversion
// const float conversion_factor = VOLT_MAX / (1 << 12); // for ADC_read conversion

void setup();
void sample(uint8_t *capture_buf);
void generateFrequencies();
void generateVoltages();
int quantizeValue(float x, float *values);
void quantizer(spi_inst_t *spi);
void DAC_setup(void);
void DAC_write(spi_inst_t *spi, float volt);
void gpio_event_string(char *buf, uint32_t events);
void gpio_callback(uint gpio, uint32_t events);
void configure_scale();
void print_bits16(uint16_t num);
void print_uint8_array_bits(uint8_t *array, size_t size);

int main()
{
    setup();

    sleep_ms(1000);

    generateFrequencies();
    generateVoltages();

    gpio_set_irq_enabled(GATE_PIN_A, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(GATE_PIN_B, GPIO_IRQ_EDGE_FALL, true);

    gpio_set_irq_enabled(NOTE_PIN_01, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(NOTE_PIN_02, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(NOTE_PIN_03, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(NOTE_PIN_04, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(NOTE_PIN_05, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(NOTE_PIN_06, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(NOTE_PIN_07, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(NOTE_PIN_08, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(NOTE_PIN_09, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(NOTE_PIN_10, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(NOTE_PIN_11, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(NOTE_PIN_12, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_callback(&gpio_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);

    while (true)
    {
        sleep_ms(1);
    }
}

void setup()
{
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(GATE_PIN_A);
    gpio_set_dir(GATE_PIN_A, GPIO_IN);
    gpio_pull_up(GATE_PIN_A);
    gpio_init(GATE_PIN_B);
    gpio_set_dir(GATE_PIN_B, GPIO_IN);
    gpio_pull_up(GATE_PIN_B);

    gpio_init(NOTE_PIN_01);
    gpio_init(NOTE_PIN_02);
    gpio_init(NOTE_PIN_03);
    gpio_init(NOTE_PIN_04);
    gpio_init(NOTE_PIN_05);
    gpio_init(NOTE_PIN_06);
    gpio_init(NOTE_PIN_07);
    gpio_init(NOTE_PIN_08);
    gpio_init(NOTE_PIN_09);
    gpio_init(NOTE_PIN_10);
    gpio_init(NOTE_PIN_11);
    gpio_init(NOTE_PIN_12);
    gpio_set_dir(NOTE_PIN_01, GPIO_IN);
    gpio_set_dir(NOTE_PIN_02, GPIO_IN);
    gpio_set_dir(NOTE_PIN_03, GPIO_IN);
    gpio_set_dir(NOTE_PIN_04, GPIO_IN);
    gpio_set_dir(NOTE_PIN_05, GPIO_IN);
    gpio_set_dir(NOTE_PIN_06, GPIO_IN);
    gpio_set_dir(NOTE_PIN_07, GPIO_IN);
    gpio_set_dir(NOTE_PIN_08, GPIO_IN);
    gpio_set_dir(NOTE_PIN_09, GPIO_IN);
    gpio_set_dir(NOTE_PIN_10, GPIO_IN);
    gpio_set_dir(NOTE_PIN_11, GPIO_IN);
    gpio_set_dir(NOTE_PIN_12, GPIO_IN);
    gpio_pull_up(NOTE_PIN_01);
    gpio_pull_up(NOTE_PIN_02);
    gpio_pull_up(NOTE_PIN_03);
    gpio_pull_up(NOTE_PIN_04);
    gpio_pull_up(NOTE_PIN_05);
    gpio_pull_up(NOTE_PIN_06);
    gpio_pull_up(NOTE_PIN_07);
    gpio_pull_up(NOTE_PIN_08);
    gpio_pull_up(NOTE_PIN_09);
    gpio_pull_up(NOTE_PIN_10);
    gpio_pull_up(NOTE_PIN_11);
    gpio_pull_up(NOTE_PIN_12);

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

    sleep_ms(1000);
    // Set up the DMA to start transferring data as soon as it appears in FIFO
    uint dma_chan = dma_claim_unused_channel(true);
    cfg = dma_channel_get_default_config(dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);
    // END ADC SETUP

    // DAC chip setup
    DAC_setup();

    // Startup check of selected scale notes
    configure_scale();
}

// Triggered on rising edge of GATE_PIN
// Quantizes incoming CV on both inputs
void gpio_callback(uint gpio, uint32_t events)
{
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    // gpio_event_string(event_str, events);
    // printf("GPIO %d %s\n", gpio, event_str);

    if (gpio == GATE_PIN_A)
    {
        if (defined_scale != 0)
            quantizer(SPI_A_PORT);
    }
    if (gpio == GATE_PIN_B)
    {
        if (defined_scale != 0)
            quantizer(SPI_B_PORT);
    }

    if (gpio != GATE_PIN_A && gpio != GATE_PIN_B)
    {
        // Note config change
        configure_scale();
    }
}

void configure_scale()
{
    // Temporary reset
    defined_scale = 0b111111111111;
    // Read all note pins in order and set the scale config
    defined_scale = defined_scale ^ ((gpio_get(NOTE_PIN_01) ? 0 : 1) << 0);
    defined_scale = defined_scale ^ ((gpio_get(NOTE_PIN_02) ? 0 : 1) << 1);
    defined_scale = defined_scale ^ ((gpio_get(NOTE_PIN_03) ? 0 : 1) << 2);
    defined_scale = defined_scale ^ ((gpio_get(NOTE_PIN_04) ? 0 : 1) << 3);
    defined_scale = defined_scale ^ ((gpio_get(NOTE_PIN_05) ? 0 : 1) << 4);
    defined_scale = defined_scale ^ ((gpio_get(NOTE_PIN_06) ? 0 : 1) << 5);
    defined_scale = defined_scale ^ ((gpio_get(NOTE_PIN_07) ? 0 : 1) << 6);
    defined_scale = defined_scale ^ ((gpio_get(NOTE_PIN_08) ? 0 : 1) << 7);
    defined_scale = defined_scale ^ ((gpio_get(NOTE_PIN_09) ? 0 : 1) << 8);
    defined_scale = defined_scale ^ ((gpio_get(NOTE_PIN_10) ? 0 : 1) << 9);
    defined_scale = defined_scale ^ ((gpio_get(NOTE_PIN_11) ? 0 : 1) << 10);
    defined_scale = defined_scale ^ ((gpio_get(NOTE_PIN_12) ? 0 : 1) << 11);

    printf("Configured scale: ");
    print_bits16(defined_scale);
    printf("\n");
}

// free-running sample
void sample(uint8_t *capture_buf, int adc_channel)
{
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

// Sample using adc_read
float sample_single(int adc_channel)
{
    adc_select_input(adc_channel);
    gpio_put(LED_PIN, 1);

    uint64_t sum = 0;

    for (int i = 0; i < NSAMP; i++)
    {
        uint16_t result = adc_read();
        printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
        sum += result;
    }
    float avg = (float)sum / NSAMP;
    float adc_voltage = avg / INPUT_VOLTAGE_DIVISION * conversion_factor;

    // adc_read();
    // uint16_t result = adc_read();
    gpio_put(LED_PIN, 0);
    // printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
    return adc_voltage;
}

void generateFrequencies()
{
    float freq = (float)FREQ_0V;

    for (int i = 0; i < NUM_PIANO_KEYS; i++)
    {
        freq = FREQ_0V * pow(pow(2, VOLT_PER_SEMITONE), i);
        FREQUENCIES[i] = roundf(freq * 1000) / 1000;
    }
}

void generateVoltages()
{
    int minVolt = 0;
    int maxVolt = 10;

    for (int i = 0; i < NUM_PIANO_KEYS; i++)
    {
        float volt = VOLT_PER_SEMITONE * i;
        VOLTAGES[i] = volt;
    }
}

// Returns the index of the value closest to x in values ("rounded" down)
int quantizeValue(float x, float *values)
{
    int n = NUM_PIANO_KEYS;
    int l = 0;     // lower limit
    int u = n - 1; // upper limit

    if (x <= values[0])
        return values[0];
    else if (x >= values[u])
        return values[u];

    while (u - l > 1)
    {
        int midPoint = (u + l) >> 1;
        if (x == values[midPoint])
            return values[midPoint];
        else if (x > values[midPoint])
            l = midPoint;
        else
            u = midPoint;
    }

    return l;
}

// Samples ADC and writes to DAC
void quantizer(spi_inst_t *spi)
{
    float adc_voltage = 0.0f;     // Average value of samples
    float desired_voltage = 0.0f; // Quantized voltage

    int cap_channel = spi == SPI_A_PORT ? ADC_CAPTURE_CHANNEL_1 : ADC_CAPTURE_CHANNEL_2;

    // adc_voltage = sample_single(cap_channel);
    sleep_ms(10); // sleep a little to let the CV stabilize
    sample(cap_buf, cap_channel);
    uint64_t sum = 0;
    for (int i = 0; i < NSAMP; i++)
    {
        sum += cap_buf[i];
    }
    float avg = (float)sum / NSAMP;
    adc_voltage = avg / INPUT_VOLTAGE_DIVISION * conversion_factor;
    int quantized_idx = quantizeValue(adc_voltage, VOLTAGES);
    int scale_note = quantized_idx % 12;
    uint16_t note_mask = 1 << scale_note;

    if ((note_mask & defined_scale) != note_mask && quantized_idx != 0)
    {
        size_t starting_point = quantized_idx;
        quantized_idx = -1;
        // From the current note, step down chromatically, up to an octave, and find the next turned on note
        for (size_t i = starting_point; i >= starting_point - 12; i--)
        {
            note_mask = 1 << (i % 12); // Bit shift to mask the correct note according to defined_scale, wraps around to 12 at 0
            if ((note_mask & defined_scale) == note_mask)
            {
                quantized_idx = i;
                printf("Floored index %0u to %0u\n", starting_point, quantized_idx);
                break;
            }
        }
    }

    if (quantized_idx < 0)
        // Trying to output a voltage below 0, which means we've reached the end of the range, and should keep outputting the previous voltage
        return;

    // printf("Scale note check %u ", scale_note);
    // print_bits16(1 << scale_note);
    // printf(" ");
    // print_bits16((1 << scale_note) & defined_scale);
    // printf("\n");

    desired_voltage = MIN(SPI_VMAX, VOLTAGES[quantized_idx]);
    printf("Sampled voltage (avg): %0.4fV Quantized => %0.4fV, %0.1fHz, idx %0u \n", adc_voltage, desired_voltage, FREQUENCIES[quantized_idx], quantized_idx);
    DAC_write(spi, desired_voltage);
}

// Initializes 4911 DAC
void DAC_setup(void)
{
    int spi_speed = 500000; // 500kHz

    // SPI A
    spi_init(SPI_A_PORT, spi_speed);
    gpio_set_function(OUT_A_SCK, GPIO_FUNC_SPI);
    gpio_set_function(OUT_A_SDI, GPIO_FUNC_SPI);

    gpio_init(OUT_A_CS);
    gpio_set_dir(OUT_A_CS, GPIO_OUT);
    gpio_put(OUT_A_CS, 1);

    gpio_init(OUT_A_LDAC);
    gpio_set_dir(OUT_A_LDAC, GPIO_OUT);
    gpio_put(OUT_A_LDAC, 1);

    // SPI B
    spi_init(SPI_B_PORT, spi_speed);
    gpio_set_function(OUT_B_SCK, GPIO_FUNC_SPI);
    gpio_set_function(OUT_B_SDI, GPIO_FUNC_SPI);

    gpio_init(OUT_B_CS);
    gpio_set_dir(OUT_B_CS, GPIO_OUT);
    gpio_put(OUT_B_CS, 1);

    gpio_init(OUT_B_LDAC);
    gpio_set_dir(OUT_B_LDAC, GPIO_OUT);
    gpio_put(OUT_B_LDAC, 1);
}

void DAC_write(spi_inst_t *spi, float volt)
{
    float _volt = MIN(volt, SPI_VMAX);
    uint8_t data[2];
    float volt_per_bit = SPI_VMAX / 1023.0;
    uint16_t value = (int)floor(_volt / volt_per_bit);
    data[0] = (0b0111'0000 & 0xF0) | ((value >> 6) & 0x0F);
    data[1] = (uint8_t)((value & 0xFF) << 2);

    printf("Writing %0.u to SPI%0u: ", value, spi == SPI_A_PORT ? 0 : 1);
    print_uint8_array_bits(data, 2);

    if (spi == SPI_A_PORT)
    {
        gpio_put(OUT_A_CS, 0);
        spi_write_blocking(SPI_A_PORT, data, 2);
        gpio_put(OUT_A_CS, 1);
        gpio_put(OUT_A_LDAC, 0);
        gpio_put(OUT_A_LDAC, 1);
    }
    else if (spi == SPI_B_PORT)
    {
        gpio_put(OUT_B_CS, 0);
        spi_write_blocking(SPI_B_PORT, data, 2);
        gpio_put(OUT_B_CS, 1);
        gpio_put(OUT_B_LDAC, 0);
        gpio_put(OUT_B_LDAC, 1);
    }
}

static const char *gpio_irq_str[] = {
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events)
{
    for (uint i = 0; i < 4; i++)
    {
        uint mask = (1 << i);
        if (events & mask)
        {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0')
            {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events)
            {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

void print_bits(uint8_t num)
{
    for (int i = 7; i >= 0; i--)
    { // 8 bits in a uint8_t
        printf("%d", (num >> i) & 1);
    }
}

void print_bits16(uint16_t num)
{
    for (int i = 15; i >= 0; i--)
    { // 8 bits in a uint8_t
        printf("%d", (num >> i) & 1);
    }
}

void print_uint8_array_bits(uint8_t *array, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        print_bits(array[i]);
        printf(" "); // Optional: space between elements
    }
    printf("\n");
}
