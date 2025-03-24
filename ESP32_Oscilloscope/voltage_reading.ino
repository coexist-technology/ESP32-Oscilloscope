#include <Arduino.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <soc/syscon_reg.h>
#include "esp_adc_cal.h"

#define ADC_CHANNEL   ADC1_CHANNEL_5  // GPIO33
#define I2S_NUM       (0)
#define NUM_SAMPLES   1000            // Number of samples
#define BUFF_SIZE     NUM_SAMPLES

uint16_t i2s_buff[BUFF_SIZE];
esp_adc_cal_characteristics_t adc_chars;

void configure_i2s(int rate) {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 2,
    .dma_buf_len = NUM_SAMPLES,
    .use_apll = false,
  };

  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
  adc1_config_width(ADC_WIDTH_BIT_12);
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);
  i2s_adc_enable(I2S_NUM);
}

void characterize_adc() {
  esp_adc_cal_characterize(
    ADC_UNIT_1,
    ADC_ATTEN_DB_11,
    ADC_WIDTH_BIT_12,
    1100,
    &adc_chars
  );
}

float to_voltage(uint16_t reading) {
  return esp_adc_cal_raw_to_voltage(reading, &adc_chars) / 1000.0; // Convert mV to V
}

void setup() {
  Serial.begin(115200);
  configure_i2s(1000000); // 1 MSPS
  characterize_adc();
}

void loop() {
  size_t bytes_read;
  esp_err_t result = i2s_read(I2S_NUM, (void*)i2s_buff, NUM_SAMPLES * sizeof(uint16_t), &bytes_read, portMAX_DELAY);
  
  if (result != ESP_OK) {
    Serial.println("Error with i2s_read");
  }

  for (int i = 0; i < NUM_SAMPLES; i++) {
    float voltage = to_voltage(i2s_buff[i]);
    Serial.println(voltage);
  }

  delay(100); // Adjust delay as needed
}
