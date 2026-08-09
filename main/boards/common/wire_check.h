#ifndef __WIRE_CHECK_H__
#define __WIRE_CHECK_H__

#include "mcp_server.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <functional>
#include <string>

#define TAG_WIRECHECK "WireCheck"

// Turns the board into a poor-man's test instrument so wiring can be verified
// without a multimeter. Three tools:
//
//   self.wirecheck.run    - continuity of a wire placed between DRIVE and SENSE
//   self.wirecheck.pin    - loopback test of an arbitrary output pin -> SENSE
//   self.wirecheck.probe  - tri-state probe on SENSE: GND / HIGH / floating
//
// SAFETY: never feed 5V into any of these pins. ESP32-S3 GPIOs are NOT 5V
// tolerant. Only 0V (GND) and 3.3V are safe to probe.
class WireCheck {
private:
    gpio_num_t drive_;
    gpio_num_t sense_;
    gpio_num_t restore_pin_;
    std::function<void()> restore_fn_;
    bool ready_ = false;

    void ConfigureSense(bool pull_up) {
        gpio_config_t in_cfg = {
            .pin_bit_mask = 1ULL << sense_,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pull_down_en = pull_up ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&in_cfg));
    }

    void ConfigureDrive(gpio_num_t pin) {
        gpio_config_t out_cfg = {
            .pin_bit_mask = 1ULL << pin,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&out_cfg));
        gpio_set_level(pin, 0);
    }

    // Reject pins that would brick the board or kill the serial console.
    static bool PinAllowed(int pin, std::string& why) {
        if (pin < 0 || pin > 48) { why = "out of range"; return false; }
        if (pin >= 22 && pin <= 25) { why = "does not exist on ESP32-S3"; return false; }
        if (pin >= 26 && pin <= 37) { why = "reserved for flash/PSRAM"; return false; }
        if (pin == 43 || pin == 44) { why = "UART0, would kill the serial log"; return false; }
        if (pin == 0 || pin == 45 || pin == 46) { why = "strapping pin"; return false; }
        return true;
    }

    // Drive a pin high/low several times and watch SENSE follow.
    int RunLoopback(gpio_num_t pin, int& high_pass, int& low_pass) {
        const int rounds = 6;
        high_pass = 0;
        low_pass = 0;
        ConfigureSense(false);
        ConfigureDrive(pin);
        for (int i = 0; i < rounds; i++) {
            gpio_set_level(pin, 1);
            vTaskDelay(pdMS_TO_TICKS(3));
            int hi = gpio_get_level(sense_);
            gpio_set_level(pin, 0);
            vTaskDelay(pdMS_TO_TICKS(3));
            int lo = gpio_get_level(sense_);
            if (hi == 1) high_pass++;
            if (lo == 0) low_pass++;
            ESP_LOGI(TAG_WIRECHECK, "round %d: drive=1 sense=%d | drive=0 sense=%d",
                     i + 1, hi, lo);
        }
        gpio_set_level(pin, 0);
        return rounds;
    }

    static std::string Verdict(int high_pass, int rounds) {
        if (high_pass == rounds) return "connected";
        if (high_pass == 0) return "open circuit";
        return "intermittent contact";
    }

public:
    WireCheck(gpio_num_t drive, gpio_num_t sense,
              gpio_num_t restore_pin = GPIO_NUM_NC,
              std::function<void()> restore_fn = nullptr)
        : drive_(drive), sense_(sense),
          restore_pin_(restore_pin), restore_fn_(std::move(restore_fn)) {
        if (drive_ == GPIO_NUM_NC || sense_ == GPIO_NUM_NC) return;

        ConfigureDrive(drive_);
        ConfigureSense(false);
        ready_ = true;
        ESP_LOGI(TAG_WIRECHECK, "ready: drive=GPIO%d sense=GPIO%d",
                 (int)drive_, (int)sense_);

        auto& mcp_server = McpServer::GetInstance();

        mcp_server.AddTool("self.wirecheck.run",
            "Continuity test for a single wire. The user puts the wire under test "
            "between the drive pin and the sense pin, then calls this.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_WIRECHECK, ">>> tool self.wirecheck.run called");
                if (!ready_) return "{\"ok\":false,\"reason\":\"not configured\"}";
                int high_pass = 0, low_pass = 0;
                int rounds = RunLoopback(drive_, high_pass, low_pass);
                std::string verdict = Verdict(high_pass, rounds);
                ESP_LOGI(TAG_WIRECHECK, "RESULT: %s (high %d/%d)",
                         verdict.c_str(), high_pass, rounds);
                return "{\"ok\":true,\"verdict\":\"" + verdict +
                       "\",\"high_pass\":" + std::to_string(high_pass) +
                       ",\"rounds\":" + std::to_string(rounds) + "}";
            });

        mcp_server.AddTool("self.wirecheck.pin",
            "Loopback test of one output pin. The user wires that pin to the sense "
            "pin. Proves the pin itself can drive a signal through the wire. "
            "Useful to prove the servo signal pin really works.",
            PropertyList({ Property("pin", kPropertyTypeInteger, 14, 1, 48) }),
            [this](const PropertyList& properties) -> ReturnValue {
                int pin = properties["pin"].value<int>();
                ESP_LOGI(TAG_WIRECHECK, ">>> tool self.wirecheck.pin called, pin=%d", pin);
                std::string why;
                if (!PinAllowed(pin, why)) {
                    ESP_LOGW(TAG_WIRECHECK, "pin %d refused: %s", pin, why.c_str());
                    return "{\"ok\":false,\"reason\":\"GPIO" + std::to_string(pin) +
                           " refused: " + why + "\"}";
                }
                if (pin == (int)sense_) {
                    return "{\"ok\":false,\"reason\":\"that is the sense pin\"}";
                }

                int high_pass = 0, low_pass = 0;
                int rounds = RunLoopback((gpio_num_t)pin, high_pass, low_pass);
                std::string verdict = Verdict(high_pass, rounds);
                ESP_LOGI(TAG_WIRECHECK, "RESULT GPIO%d: %s (high %d/%d)",
                         pin, verdict.c_str(), high_pass, rounds);

                if (restore_fn_ && pin == (int)restore_pin_) {
                    ESP_LOGI(TAG_WIRECHECK, "restoring GPIO%d to its owner", pin);
                    restore_fn_();
                }
                return "{\"ok\":true,\"pin\":" + std::to_string(pin) +
                       ",\"verdict\":\"" + verdict +
                       "\",\"high_pass\":" + std::to_string(high_pass) +
                       ",\"rounds\":" + std::to_string(rounds) + "}";
            });

        mcp_server.AddTool("self.wirecheck.probe",
            "Probe whatever is wired to the sense pin and report if it is ground, "
            "a 3.3V high rail, or floating. Safe for GND and 3V3 only. "
            "Never connect 5V to it.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGI(TAG_WIRECHECK, ">>> tool self.wirecheck.probe called");
                if (!ready_) return "{\"ok\":false,\"reason\":\"not configured\"}";

                // Park the drive pin as input so it cannot influence the reading.
                gpio_config_t idle = {
                    .pin_bit_mask = 1ULL << drive_,
                    .mode = GPIO_MODE_INPUT,
                    .pull_up_en = GPIO_PULLUP_DISABLE,
                    .pull_down_en = GPIO_PULLDOWN_DISABLE,
                    .intr_type = GPIO_INTR_DISABLE,
                };
                ESP_ERROR_CHECK(gpio_config(&idle));

                ConfigureSense(true);
                vTaskDelay(pdMS_TO_TICKS(5));
                int with_pullup = gpio_get_level(sense_);
                ConfigureSense(false);
                vTaskDelay(pdMS_TO_TICKS(5));
                int with_pulldown = gpio_get_level(sense_);

                ESP_LOGI(TAG_WIRECHECK, "probe: pullup reads %d, pulldown reads %d",
                         with_pullup, with_pulldown);

                std::string state;
                if (with_pullup == 0 && with_pulldown == 0) {
                    state = "ground";
                } else if (with_pullup == 1 && with_pulldown == 1) {
                    state = "high rail 3V3";
                } else {
                    state = "floating, nothing connected";
                }
                ESP_LOGI(TAG_WIRECHECK, "PROBE RESULT: %s", state.c_str());

                ConfigureDrive(drive_);
                ConfigureSense(false);
                return "{\"ok\":true,\"state\":\"" + state + "\"}";
            });
    }
};

#endif  // __WIRE_CHECK_H__
