#include "hardware/spi.h"

struct DAC_CHANNEL {
  spi_inst_t *spi_port;
  uint sck;
  uint sdi;
  uint cs;
  uint ldac;
};

void setup_dac(spi_inst_t *spi_port, uint out_sck, uint out_sdi, uint out_cs,
               uint out_ldac);

void DAC_write(uint spi, float volt);