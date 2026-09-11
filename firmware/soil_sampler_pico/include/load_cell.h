#ifndef SOIL_SAMPLER_LOAD_CELL_H
#define SOIL_SAMPLER_LOAD_CELL_H

#include <stdbool.h>
#include <stdint.h>

bool load_cell_init(uint8_t buffer_size);
bool load_cell_available(void);
bool load_cell_update(void);
int32_t load_cell_read_average(void);
float load_cell_read_newton(void);
uint8_t load_cell_get_sample_count(void);
void load_cell_calibrate_bias(uint8_t num_samples);

extern int32_t LOAD_CELL_BIAS;
extern const float LOAD_CELL_CONVERSION_FACTOR;

#endif