#ifndef __SERVO_CONTROLLER_H__
#define __SERVO_CONTROLLER_H__

#include "mcp_server.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>

#define TAG_SERVO "ServoController"

// SG90-style servo driven by ESP32-S3 LEDC PWM (50 Hz).
// Exposed through MCP so XiaoZhi can drive it by voice, e.g.
//   "把舵机转到 90 度" -> self.servo.set(angle=90)
class ServoController {
private:
    gpio_num_t gpio_num_;
    ledc_channel_t channel_ = LEDC_CHANNEL_1;
    bool attached_ = false;

    void Attach() {
        // TIMER_2 / CHANNEL_1: the LCD backlight already owns TIMER_0 / CHANNEL_0.
        ledc_timer_config_t timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_13_BIT,
            .timer_num = LEDC_TIMER_2,
            .freq_hz = 50,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&timer));

        ledc_channel_config_t channel = {
            .gpio_num = gpio_num_,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = channel_,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_2,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
        attached_ = true;
        ESP_LOGI(TAG_SERVO, "attached on GPIO%d, LEDC timer2/ch1, 50Hz 13bit", (int)gpio_num_);
    }

    // 50Hz period = 20000us, 13-bit resolution = 8191.
    // Standard servo: 0deg -> 0.5ms pulse, 180deg -> 2.5ms pulse.
    uint32_t AngleToDuty(int angle) {
        double pulse_us = 500.0 + (angle / 180.0) * 2000.0;
        return (uint32_t)(pulse_us / 20000.0 * 8191.0);
    }

    volatile bool sweeping_ = false;
    volatile int sweep_cycles_ = 1;

    // Sweep task: runs off the main event loop so we never block it.
    // Keeps swinging 0 <-> 180 so the wiring can be wiggled hands-free.
    static void SweepTask(void* arg) {
        auto* self = static_cast<ServoController*>(arg);
        int cycles = self->sweep_cycles_;
        ESP_LOGI(TAG_SERVO, "sweep START, %d cycles (~%d s) -- wiggle the wires now",
                 cycles, cycles * 2);
        for (int i = 0; i < cycles; i++) {
            self->SetAngle(0);
            vTaskDelay(pdMS_TO_TICKS(1000));
            self->SetAngle(180);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        self->SetAngle(90);
        ESP_LOGI(TAG_SERVO, "sweep DONE, back to center");
        self->sweeping_ = false;
        vTaskDelete(nullptr);
    }

public:
    // Re-bind LEDC to the pin after something else (e.g. a pin loopback test)
    // has taken the GPIO over.
    void Reattach() {
        if (gpio_num_ == GPIO_NUM_NC) return;
        attached_ = false;
        Attach();
        SetAngle(90);
        ESP_LOGI(TAG_SERVO, "reattached after external pin use");
    }

    void SetAngle(int angle) {
        if (!attached_) {
            ESP_LOGW(TAG_SERVO, "SetAngle(%d) ignored: not attached", angle);
            return;
        }
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;
        uint32_t duty = AngleToDuty(angle);
        esp_err_t e1 = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_, duty);
        esp_err_t e2 = ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_);
        ESP_LOGI(TAG_SERVO, "SetAngle(%d) -> duty=%u (%.0fus) set=%s update=%s",
                 angle, (unsigned)duty, 500.0 + (angle / 180.0) * 2000.0,
                 esp_err_to_name(e1), esp_err_to_name(e2));
    }

    // Fire the sweep without any voice/network involvement.
    void StartSweep(int cycles) {
        if (sweeping_) return;
        sweeping_ = true;
        sweep_cycles_ = cycles;
        if (xTaskCreate(SweepTask, "servo_sweep", 3072, this, 3, nullptr) != pdPASS) {
            sweeping_ = false;
            ESP_LOGE(TAG_SERVO, "failed to create sweep task");
        }
    }

    ServoController(gpio_num_t gpio_num, int boot_selftest_cycles = 0)
        : gpio_num_(gpio_num) {
        if (gpio_num_ == GPIO_NUM_NC) return;

        Attach();

        // Power-on self test: swings immediately after boot. No Wi-Fi, no MCP,
        // no wake word -- if the hardware works, it moves. This isolates the
        // servo from every software layer above it.
        if (boot_selftest_cycles > 0) {
            ESP_LOGI(TAG_SERVO, "boot self-test scheduled: %d cycles", boot_selftest_cycles);
            StartSweep(boot_selftest_cycles);
        }

        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.servo.set", "Set the servo angle in degrees (0-180)",
            PropertyList({ Property("angle", kPropertyTypeInteger, 90, 0, 180) }),
            [this](const PropertyList& properties) -> ReturnValue {
                int angle = properties["angle"].value<int>();
                ESP_LOGI(TAG_SERVO, ">>> tool self.servo.set called, angle=%d", angle);
                SetAngle(angle);
                return "{\"ok\":true,\"angle\":" + std::to_string(angle) + "}";
            });

        mcp_server.AddTool("self.servo.center", "Move the servo to center (90 degrees)",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_SERVO, ">>> tool self.servo.center called");
                SetAngle(90);
                return "{\"ok\":true,\"angle\":90}";
            });

        mcp_server.AddTool("self.servo.sweep",
            "Self-test: keep swinging the servo between 0 and 180 degrees so the user "
            "can check the wiring. Each cycle takes about 2 seconds.",
            PropertyList({ Property("cycles", kPropertyTypeInteger, 15, 1, 60) }),
            [this](const PropertyList& properties) -> ReturnValue {
                int cycles = properties["cycles"].value<int>();
                ESP_LOGI(TAG_SERVO, ">>> tool self.servo.sweep called, cycles=%d", cycles);
                if (sweeping_) {
                    ESP_LOGW(TAG_SERVO, "sweep already running, ignored");
                    return "{\"ok\":false,\"reason\":\"already sweeping\"}";
                }
                sweeping_ = true;
                sweep_cycles_ = cycles;
                if (xTaskCreate(SweepTask, "servo_sweep", 3072, this, 3, nullptr) != pdPASS) {
                    sweeping_ = false;
                    ESP_LOGE(TAG_SERVO, "failed to create sweep task");
                    return "{\"ok\":false,\"reason\":\"task create failed\"}";
                }
                return "{\"ok\":true,\"action\":\"sweep\",\"cycles\":" +
                       std::to_string(cycles) + "}";
            });
    }
};

#endif  // __SERVO_CONTROLLER_H__
