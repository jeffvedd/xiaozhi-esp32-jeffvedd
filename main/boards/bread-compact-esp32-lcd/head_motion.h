#pragma once
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_random.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Pinos dos servos (livres na bread-compact-esp32-lcd)
#ifndef HEAD_SERVO_PAN_GPIO
#define HEAD_SERVO_PAN_GPIO  GPIO_NUM_13   // esquerda/direita
#endif
#ifndef HEAD_SERVO_TILT_GPIO
#define HEAD_SERVO_TILT_GPIO GPIO_NUM_16   // cima/baixo
#endif

class HeadMotion {
public:
    enum Mode { kIdle, kListening, kSpeaking };

    static HeadMotion& Instance() {
        static HeadMotion h;
        return h;
    }

    void Init() {
        if (started_) return;
        started_ = true;

        ledc_timer_config_t t = {};
        t.speed_mode = LEDC_LOW_SPEED_MODE;
        t.duty_resolution = LEDC_TIMER_14_BIT;
        t.timer_num = LEDC_TIMER_3;
        t.freq_hz = 50;
        t.clk_cfg = LEDC_AUTO_CLK;
        ledc_timer_config(&t);

        Attach(HEAD_SERVO_PAN_GPIO, LEDC_CHANNEL_6);
        Attach(HEAD_SERVO_TILT_GPIO, LEDC_CHANNEL_7);
        Write(LEDC_CHANNEL_6, pan_);
        Write(LEDC_CHANNEL_7, tilt_);

        xTaskCreate(Task, "head_motion", 4096, this, 3, nullptr);
    }

    void SetMode(Mode m) { mode_ = m; }

private:
    bool started_ = false;
    volatile Mode mode_ = kIdle;
    float pan_ = 90, tilt_ = 90, pan_t_ = 90, tilt_t_ = 90;

    // Limites de segurança para não forçar o suporte
    static constexpr float kPanMin = 55, kPanMax = 125;
    static constexpr float kTiltMin = 65, kTiltMax = 115;

    static float Clamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    static float Rand(float range) {  // -range .. +range
        return ((int)(esp_random() % 2001) - 1000) / 1000.0f * range;
    }

    void Attach(gpio_num_t pin, ledc_channel_t ch) {
        ledc_channel_config_t c = {};
        c.gpio_num = pin;
        c.speed_mode = LEDC_LOW_SPEED_MODE;
        c.channel = ch;
        c.timer_sel = LEDC_TIMER_3;
        c.duty = 0;
        c.hpoint = 0;
        ledc_channel_config(&c);
    }

    void Write(ledc_channel_t ch, float deg) {
        float us = 500.0f + (deg / 180.0f) * 1900.0f;      // 500..2400 us
        uint32_t duty = (uint32_t)(us / 20000.0f * 16383.0f);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    }

    static void Task(void* arg) {
        auto* s = static_cast<HeadMotion*>(arg);
        TickType_t next_change = 0;
        Mode last_mode = kIdle;

        while (true) {
            TickType_t now = xTaskGetTickCount();
            Mode m = s->mode_;

            if (m != last_mode || now >= next_change) {
                last_mode = m;
                switch (m) {
                case kSpeaking:   // pequenos balanços enquanto fala
                    s->pan_t_ = 90 + Rand(8);
                    s->tilt_t_ = 90 + Rand(5);
                    next_change = now + pdMS_TO_TICKS(400 + esp_random() % 500);
                    break;
                case kListening:  // cabeça levemente inclinada, atenta
                    s->pan_t_ = 90 + Rand(3);
                    s->tilt_t_ = 82;
                    next_change = now + pdMS_TO_TICKS(1200);
                    break;
                default:          // ocioso: olha em volta de vez em quando
                    s->pan_t_ = 90 + Rand(15);
                    s->tilt_t_ = 90 + Rand(6);
                    next_change = now + pdMS_TO_TICKS(5000 + esp_random() % 5000);
                    break;
                }
                s->pan_t_ = Clamp(s->pan_t_, kPanMin, kPanMax);
                s->tilt_t_ = Clamp(s->tilt_t_, kTiltMin, kTiltMax);
            }

            // Movimento suave até o alvo
            float dp = s->pan_t_ - s->pan_;
            float dt = s->tilt_t_ - s->tilt_;
            if (dp > 0.1f || dp < -0.1f) {
                s->pan_ += dp * 0.12f;
                s->Write(LEDC_CHANNEL_6, s->pan_);
            }
            if (dt > 0.1f || dt < -0.1f) {
                s->tilt_ += dt * 0.12f;
                s->Write(LEDC_CHANNEL_7, s->tilt_);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
};
