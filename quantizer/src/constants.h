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
#define SPI_SPEED 500000 // 500kHz
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

#define INPUT_VOLTAGE_DIVISION                                                 \
  (0.1)               // The incoming voltage will have been divided by 10
#define VOLT_MAX 3.3f // Maximum input voltage

#define VOLT_PER_SEMITONE (1.0 / 12.0)
#define NUM_PIANO_KEYS 6 * 12

// set this to determine sample rate
// 96     = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define FSAMP 5000 // Hz
#define CLOCK_DIV (48000000.0 / FSAMP)

// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly
#define NSAMP 10