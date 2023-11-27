#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "driver/adc.h"

#define MLX90614_ADDR   0x5A

// Definições do pino do sensor de umidade
#define SENSOR_PIN GPIO_NUM_32
/**
 * @brief Parâmetros de inicialização da conexão I2C
 * @param conf Ponteiro com os parâmetros
 */
void i2c_conf ()
{
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = 21,
		.scl_io_num = 22,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 50000,
	};
	i2c_param_config(I2C_NUM_0, &conf);

	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}


/**
 * @brief Leitura da Temperatura de um objeto
 * @param MLX90614_ADDR Endereço do dispositivo
 * @return Temperatura em graus celsius
 */
float mlx90614_read_temp(i2c_port_t i2c_num)
{
    uint8_t data[3];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MLX90614_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x07, true);     // Leitura da temperatura do objeto no endereço 0x07
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MLX90614_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[2], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    float temp = (data[1] << 8) | data[0];
    temp *= 0.02;
    temp -= 273.15;
    return temp;
}


void app_main() {

    i2c_conf();

    // Configurar a estrutura de configuração ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_0);

	while (1) {
        float temp = mlx90614_read_temp(I2C_NUM_0);
        printf("Temperature: %.2f°C\n", temp);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Ler o valor analógico do sensor
        int valor_leitura = adc1_get_raw(ADC1_CHANNEL_4);

        // Fazer algo com o valor lido (por exemplo, imprimir)
        printf("Valor de umidade: %d\n", valor_leitura);

        // Aguardar um tempo antes da próxima leitura
        vTaskDelay(pdMS_TO_TICKS(1000)); // Aguarda 1 segundo
	}
}
