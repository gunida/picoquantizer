#include "dac4911.h"
#include "constants.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include <math.h>

DAC_CHANNEL channels[2];
int num_channels = 0;

/// Set up one output
void setup_dac(spi_inst_t *spi_port, uint out_sck, uint out_sdi, uint out_cs,
               uint out_ldac) {
  DAC_CHANNEL channel = {spi_port, out_sck, out_sdi, out_cs, out_ldac};

  spi_init(spi_port, SPI_SPEED);
  gpio_set_function(out_sck, GPIO_FUNC_SPI);
  gpio_set_function(out_sdi, GPIO_FUNC_SPI);

  gpio_init(out_cs);
  gpio_set_dir(out_cs, GPIO_OUT);
  gpio_put(out_cs, 1);

  gpio_init(out_ldac);
  gpio_set_dir(out_ldac, GPIO_OUT);
  gpio_put(out_ldac, 1);

  channels[num_channels] = channel;
  num_channels += 1;
}

void DAC_write(uint channel, float volt) {
  float _volt = MIN(volt, SPI_VMAX);
  uint8_t data[2];
  float volt_per_bit = SPI_VMAX / 1023.0;
  uint16_t value = (int)floor(_volt / volt_per_bit);
  data[0] = (0b0111'0000 & 0xF0) | ((value >> 6) & 0x0F);
  data[1] = (uint8_t)((value & 0xFF) << 2);

  gpio_put(channels[channel].cs, 0);
  spi_write_blocking(channels[channel].spi_port, data, 2);
  gpio_put(channels[channel].cs, 1);
  gpio_put(channels[channel].ldac, 0);
  gpio_put(channels[channel].ldac, 1);
}