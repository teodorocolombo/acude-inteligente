#include "sct013.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SCT013_ADC_UNIT           ADC_UNIT_1
#define SCT013_ADC_CHANNEL        ADC_CHANNEL_6
#define SCT013_ADC_ATTEN          ADC_ATTEN_DB_12

#define SCT013_SAMPLES_TO_READ    200
#define SCT013_SAMPLE_DELAY_MS    1

#define SCT013_SENSOR_SENSITIVITY 50.0

#define SCT013_NOISE_THRESHOLD_MV 600.0

static const char *TAG = "SCT013_SENSOR";
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool adc_initialized = false;
static bool do_calibration = false;


static bool adc_calibration_init(adc_unit_t unit_id, adc_channel_t channel, adc_atten_t atten,
                                 adc_cali_handle_t *cali_handle) {
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;


#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Attempting curve fitting calibration...");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit_id,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, cali_handle);
        if (ret == ESP_OK) {
            calibrated = true;
        } else if (ret == ESP_ERR_NOT_SUPPORTED || ret == ESP_ERR_INVALID_ARG) {
            ESP_LOGW(TAG, "Curve fitting calibration not supported or invalid arg for this ADC, ret=0x%x", ret);
        } else {
            ESP_LOGE(TAG, "Failed to create curve fitting calibration scheme, ret=0x%x", ret);
        }
    }
#endif


#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Attempting line fitting calibration...");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit_id,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, cali_handle);
        if (ret == ESP_OK) {
            calibrated = true;
        } else if (ret == ESP_ERR_NOT_SUPPORTED || ret == ESP_ERR_INVALID_ARG) {
            ESP_LOGW(TAG, "Line fitting calibration not supported or invalid arg for this ADC, ret=0x%x", ret);
        } else {
            ESP_LOGE(TAG, "Failed to create line fitting calibration scheme, ret=0x%x", ret);
        }
    }
#endif

    if (calibrated) {
        ESP_LOGI(TAG, "ADC calibration scheme created successfully.");
    }
    return calibrated;
}


esp_err_t sct013_init(void) {
    if (adc_initialized) {
        ESP_LOGW(TAG, "SCT-013 sensor interface already initialized.");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing SCT-013 current sensor interface...");


    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = SCT013_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC1 unit, error code: 0x%x", ret);
        return ret;
    }


    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = SCT013_ADC_ATTEN,
    };
    ret = adc_oneshot_config_channel(adc1_handle, SCT013_ADC_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC1 channel %d (GPIO-34), error code: 0x%x", SCT013_ADC_CHANNEL, ret);

        adc_oneshot_del_unit(adc1_handle);
        return ret;
    }


    do_calibration = adc_calibration_init(SCT013_ADC_UNIT, SCT013_ADC_CHANNEL, SCT013_ADC_ATTEN, &adc_cali_handle);
    if (!do_calibration) {
        ESP_LOGW(
            TAG, "ADC calibration not enabled or failed. Raw ADC readings will be used, which might be less accurate.");
    }

    adc_initialized = true;
    ESP_LOGI(TAG, "SCT-013 sensor interface initialized on GPIO-34 (ADC1_CHANNEL%d).", SCT013_ADC_CHANNEL);
    return ESP_OK;
}


esp_err_t sct013_read_current(double *current_rms) {
    if (!adc_initialized) {
        ESP_LOGE(TAG, "ADC not initialized. Call sct013_init() first.");
        *current_rms = 0.0;
        return ESP_FAIL;
    }

    if (current_rms == NULL) {
        ESP_LOGE(TAG, "current_rms pointer is NULL. Cannot store result.");
        return ESP_ERR_INVALID_ARG;
    }

    int raw_adc_val = 0;

    int min_raw_adc = 4096;

    int max_raw_adc = 0;


    for (int i = 0; i < SCT013_SAMPLES_TO_READ; i++) {
        esp_err_t ret = adc_oneshot_read(adc1_handle, SCT013_ADC_CHANNEL, &raw_adc_val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read ADC, error code: 0x%x", ret);
            *current_rms = 0.0;
            return ret;
        }


        if (raw_adc_val < min_raw_adc) {
            min_raw_adc = raw_adc_val;
        }
        if (raw_adc_val > max_raw_adc) {
            max_raw_adc = raw_adc_val;
        }


        vTaskDelay(pdMS_TO_TICKS(SCT013_SAMPLE_DELAY_MS));
    }


    int min_voltage_mv = 0;
    int max_voltage_mv = 0;

    if (do_calibration && adc_cali_handle != NULL) {
        esp_err_t ret_min = adc_cali_raw_to_voltage(adc_cali_handle, min_raw_adc, &min_voltage_mv);
        esp_err_t ret_max = adc_cali_raw_to_voltage(adc_cali_handle, max_raw_adc, &max_voltage_mv);

        if (ret_min != ESP_OK || ret_max != ESP_OK) {
            ESP_LOGE(TAG, "Failed to convert raw ADC to voltage using calibration (min err: 0x%x, max err: 0x%x).",
                     ret_min, ret_max);
            *current_rms = 0.0;
            return ESP_FAIL;
        }
    } else {
        float voltage_per_step = 3300.0 / 4095.0;
        min_voltage_mv = (int) (min_raw_adc * voltage_per_step);
        max_voltage_mv = (int) (max_raw_adc * voltage_per_step);
        ESP_LOGW(TAG, "Using uncalibrated voltage conversion. Accuracy may vary.");
    }

    double v_peak_to_peak_mv = (double) (max_voltage_mv - min_voltage_mv);
    double v_peak_mv = v_peak_to_peak_mv / 2.0;
    double v_rms_mv = v_peak_mv / sqrt(2.0);
    double v_rms_volts = v_rms_mv / 1000.0;


    *current_rms = v_rms_volts * SCT013_SENSOR_SENSITIVITY;


    ESP_LOGI(TAG, "Raw ADC: Min=%d, Max=%d", min_raw_adc, max_raw_adc);
    ESP_LOGI(TAG, "Voltage (mV): Min=%d, Max=%d, Pk-Pk=%.2f, RMS=%.2f", min_voltage_mv, max_voltage_mv,
             v_peak_to_peak_mv, v_rms_mv);
    ESP_LOGI(TAG, "Calculated Current RMS: %.3f Amps", *current_rms);

    return ESP_OK;
}
