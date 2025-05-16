#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "kiss_fftr.h"

#define CAPTURE_CHANNEL 1
#define LED_PIN 25
#define CV_OUT_PIN 13

#define VOLT_PER_SEMITONE (1.0 / 12.0)
#define FREQ_0V 27.5 // TODO: set FREQ_0V to be whatever frequency 0V is equal to
#define NUM_PIANO_KEYS 120

// set this to determine sample rate
// 96     = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define FSAMP 8000 // Hz
#define CLOCK_DIV (48000000 / FSAMP)

// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly
#define NSAMP 2048

#define LED_DELAY_MS 1000

static float FREQUENCIES[NUM_PIANO_KEYS]; // Frequencies of each actual note starting from FREQ_0V
static float VOLTAGES[NUM_PIANO_KEYS];    // Voltages of each actual note starting from 0V
dma_channel_config cfg;
uint dma_chan;
float freqs[NSAMP];

// Which scale should we quantize to
// 0 = Chromatic
// 1 = Major
// 2 = Dorian
// 3 = Phrygian
// 4 = Lydian
// 5 = Mixolydian
// 6 = Aeolian
// 7 = Locrian?
static int chosenScale = 0;

// 0-8 C D E F G A B C
static int rootNote = 0;

// 0 = neutral
// 1 = Flat
// 2 = Sharp
static short sign = 1;

void setup();
void sample(uint8_t *capture_buf);
void generateFrequencies();
void generateVoltages();
float quantizeValue(float x, float *values);
void quantizeValue_should_find_nearest_lower();

void sample(uint8_t *capture_buf)
{
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

void setup()
{
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    adc_gpio_init(26 + CAPTURE_CHANNEL);

    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
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

    float f_max = FSAMP;
    float f_res = f_max / NSAMP;
    for (int i = 0; i < NSAMP; i++)
    {
        freqs[i] = f_res * i;
    }
    // END ADC SETUP

    // Set up output
    gpio_set_function(CV_OUT_PIN, GPIO_FUNC_PWM);
}

int main()
{
    FILE *fptr;

    uint8_t cap_buf[NSAMP];
    kiss_fft_scalar fft_in[NSAMP]; // kiss_fft_scalar is a float
    kiss_fft_cpx fft_out[NSAMP];
    kiss_fftr_cfg cfg = kiss_fftr_alloc(NSAMP, false, 0, 0);

    setup();
    sleep_ms(5000);

    generateFrequencies();
    generateVoltages();

    quantizeValue_should_find_nearest_lower();

    // startBlinking();
    float f_max = FSAMP;
    float f_res = f_max / NSAMP;
    while (1)
    {
        // printf("f_res: %f, FSAMP: %i, NSAMP: %i, CLKDIV: %i\n", f_res, FSAMP, NSAMP, CLOCK_DIV);
        // get NSAMP samples at FSAMP
        sample(cap_buf);

        // fill fourier transform input while subtracting DC component
        uint64_t sum = 0;
        for (int i = 0; i < NSAMP; i++)
        {
            // printf("%u\n", cap_buf[i]);
            sum += cap_buf[i];
        }
        float avg = (float)sum / NSAMP;
        for (int i = 0; i < NSAMP; i++)
        {
            fft_in[i] = (float)cap_buf[i] - avg;
        }

        // compute fft
        kiss_fftr(cfg, fft_in, fft_out);

        // compute power and calculate max freq component
        float max_power = 0;
        int max_idx = 0;
        // any frequency bin over NSAMP/2 is aliased (nyquist sampling theorum)
        for (int i = 0; i < NSAMP / 2; i++)
        {
            float power = fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i;
            if (power > max_power)
            {
                max_power = power;
                max_idx = i;
            }
        }

        float max_freq = freqs[max_idx];
        printf("Greatest Frequency Components: %0.1f Hz", max_freq);
        float quantized = quantizeValue(max_freq, FREQUENCIES);
        printf(", Quantized => %0.1f\n", quantized);
    }

    // should never get here
    kiss_fft_free(cfg);
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
        float volt = VOLT_PER_SEMITONE * (i + 1);
        VOLTAGES[i] = volt;
    }
}

// Finds the value closest to x in values ("rounded" down)
float quantizeValue(float x, float *values)

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

    return values[l];
}

// steps through a few frequencies to check what they get quantized to
void quantizeValue_should_find_nearest_lower()
{
    float baseFreq = 110;
    for (int i = 0; i < 20; i++)
    {
        float freq = baseFreq + i;
        float result = quantizeValue(freq, FREQUENCIES);
        printf("%f => %f\n", freq, result);
    }
}

void no_duplicate_frequency_bins()
{
}