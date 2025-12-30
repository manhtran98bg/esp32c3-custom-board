#include "wifi_board.h"
#include "codecs/es8311_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "led/single_led.h"
#include "assets/lang_config.h"
#include <wifi_station.h>
#include <esp_log.h>
#include <esp_efuse_table.h>
#include <driver/i2c_master.h>

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_gc9a01.h>
#include "system_reset.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <esp_timer.h>
#include "i2c_device.h"
#include <esp_lcd_panel_vendor.h>
#include <driver/spi_common.h>
#include "power_save_timer.h"
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "power_manager.h"

#define TAG "CustomC3Board"

LV_FONT_DECLARE(font_puhui_16_4);
LV_FONT_DECLARE(font_awesome_16_4);


class Cst816d : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };
    Cst816d(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        read_buffer_ = new uint8_t[6];
    }

    ~Cst816d() {
        if (read_buffer_) {
            delete[] read_buffer_;
            read_buffer_ = nullptr;
        }
    }

    void UpdateTouchPoint() {
        if (!read_buffer_) return;
        ReadRegs(0x02, read_buffer_, 6);
        if (read_buffer_[0] == 0xFF) {
            read_buffer_[0] = 0x00;
        }
        tp_.num = read_buffer_[0] & 0x01;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    const TouchPoint_t& GetTouchPoint() const {
        return tp_;
    }

    static bool Probe(i2c_master_bus_handle_t i2c_bus, uint8_t addr, uint8_t& chip_id) {
        if (!i2c_bus) return false;
        i2c_master_dev_handle_t dev = nullptr;
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 400 * 1000,
            .scl_wait_us = 0,
            .flags = {
                .disable_ack_check = 0,
            },
        };
        esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &cfg, &dev);
        if (ret != ESP_OK || dev == nullptr) {
            return false;
        }
        uint8_t reg = 0xA3;
        uint8_t id = 0;
        ret = i2c_master_transmit_receive(dev, &reg, 1, &id, 1, 100);
        i2c_master_bus_rm_device(dev);
        if (ret == ESP_OK) { 
            chip_id = id;
            return true;
        }
        return false;
    }

private:
    uint8_t* read_buffer_ = nullptr;
    TouchPoint_t tp_;
    uint8_t last_chip_id_ = 0;
};


class CustomLcdDisplay : public SpiLcdDisplay {
public:
    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle,
                    esp_lcd_panel_handle_t panel_handle,
                    int width,
                    int height,
                    int offset_x,
                    int offset_y,
                    bool mirror_x,
                    bool mirror_y,
                    bool swap_xy)
        : SpiLcdDisplay(io_handle, panel_handle, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
        DisplayLockGuard lock(this);
        // Because the screen is round, the status bar needs to have increased left and right inner margins.
        lv_obj_set_style_pad_left(status_bar_, LV_HOR_RES * 0.33, 0);
        lv_obj_set_style_pad_right(status_bar_, LV_HOR_RES * 0.33, 0);
    }
};


class CustomC3Board : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    Button boot_button_;
    Display* display_ = nullptr;
    Cst816d* cst816d_ = nullptr;
    Backlight *backlight_ = nullptr;
    AudioCodec *audio_codec_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    esp_timer_handle_t touchpad_timer_ = nullptr;
    PowerSaveTimer* power_save_timer_ = nullptr;
    

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(160, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
        });
        power_save_timer_->SetEnabled(true);
    }


    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
        if (i2c_master_probe(i2c_bus_, ES8311_CODEC_DEFAULT_ADDR >> 1, 1000) != ESP_OK) {
            ESP_LOGE(TAG, "Failed probe Codec ES8311, check hardware first.");
            while (true) {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            
        }
        ESP_LOGI(TAG, "I2C bus initialized, found codec ES8311 device");
        audio_codec_ = new Es8311AudioCodec(i2c_bus_, I2C_BUS, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
    }



    static void touchpad_timer_callback(void* arg) {
        auto* board = static_cast<CustomC3Board*>(arg);
        if (!board || !board->cst816d_) return;
        static bool was_touched = false;
        static int64_t touch_start_time = 0;
        const int64_t TOUCH_THRESHOLD_MS = 500;  // Touch duration threshold: touch duration exceeding 500ms is considered a long press.

        board->cst816d_->UpdateTouchPoint();
        auto touch_point = board->cst816d_->GetTouchPoint();

        // Touch detection begins
        if (touch_point.num > 0 && !was_touched) {
            was_touched = true;
            touch_start_time = esp_timer_get_time() / 1000; // 转换为毫秒
        }
        // Touch release
        else if (touch_point.num == 0 && was_touched) {
            was_touched = false;
            int64_t touch_duration = (esp_timer_get_time() / 1000) - touch_start_time;

            // short touch
            if (touch_duration < TOUCH_THRESHOLD_MS) {
                auto& app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateStarting) {
                    board->EnterWifiConfigMode();
                    return;
                }
                app.ToggleChatState();
            }
        }
    }

    void InitializeCst816DTouchPad() {
        // RST io initialize
        if (TP_PIN_NUM_TP_RST != GPIO_NUM_NC) 
        {
            gpio_config_t rst_io_conf = {};
            rst_io_conf.mode = GPIO_MODE_OUTPUT;
            rst_io_conf.pin_bit_mask = (1ULL << TP_PIN_NUM_TP_RST);
            gpio_config(&rst_io_conf);
        }
        gpio_set_level(TP_PIN_NUM_TP_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(TP_PIN_NUM_TP_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(50));

        // Detecting the presence of a touch chip
        uint8_t chip_id = 0;
        if (!i2c_bus_) {
            ESP_LOGW(TAG, "Touch I2C bus not initialized, skip touch");
            return;
        }
        bool touch_available = Cst816d::Probe(i2c_bus_, TP_CTS816D_ADDR, chip_id);
        if (!touch_available) {
            ESP_LOGW(TAG, "CST816D not found, running in non-touch mode");
            return;
        }

        cst816d_ = new Cst816d(i2c_bus_, TP_CTS816D_ADDR);

        // Create a timer with a 10ms interval (Polling mode, no isr)
        esp_timer_create_args_t timer_args = {
            .callback = touchpad_timer_callback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "touchpad_timer",
            .skip_unhandled_events = true,
        };

        if (esp_timer_create(&timer_args, &touchpad_timer_) == ESP_OK) {
            esp_timer_start_periodic(touchpad_timer_, 10 * 1000); // 10ms = 10000us
        }
    }

    // SPI Initialization
    void InitializeSpi() {
        ESP_LOGI(TAG, "Initialize SPI bus");
        spi_bus_config_t buscfg = GC9A01_PANEL_BUS_SPI_CONFIG(DISPLAY_SPI_SCLK_PIN, DISPLAY_SPI_MOSI_PIN,
                                    DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    // GC9A01 initialization
    void InitializeGc9a01Display() {
        ESP_LOGI(TAG, "Init GC9A01 display");
        ESP_LOGI(TAG, "Install panel IO");
        esp_lcd_panel_io_handle_t io_handle = NULL;
        esp_lcd_panel_io_spi_config_t io_config = GC9A01_PANEL_IO_SPI_CONFIG(DISPLAY_SPI_CS_PIN, DISPLAY_SPI_DC_PIN, 0, NULL);
        io_config.pclk_hz = DISPLAY_SPI_SCLK_HZ;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &io_handle));

        ESP_LOGI(TAG, "Install GC9A01 panel driver");
        esp_lcd_panel_handle_t panel_handle = NULL;
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_SPI_RESET_PIN;    // Set to -1 if not use
        panel_config.rgb_endian = LCD_RGB_ENDIAN_BGR;           //LCD_RGB_ENDIAN_RGB;
        panel_config.bits_per_pixel = 16;

        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
        panel_ = panel_handle;
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

        uint8_t data_0x62[] = { 0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70 };
        esp_lcd_panel_io_tx_param(io_handle, 0x62, data_0x62, sizeof(data_0x62));

        uint8_t data_0x63[] = { 0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70 };
        esp_lcd_panel_io_tx_param(io_handle, 0x63, data_0x63, sizeof(data_0x63));

        uint8_t data_0x36[] = { 0x48};
        esp_lcd_panel_io_tx_param(io_handle, 0x36, data_0x36, sizeof(data_0x36));

        uint8_t data_0xC3[] = { 0x1F};
        esp_lcd_panel_io_tx_param(io_handle, 0xC3, data_0xC3, sizeof(data_0xC3));

        uint8_t data_0xC4[] = { 0x1F};
        esp_lcd_panel_io_tx_param(io_handle, 0xC4, data_0xC4, sizeof(data_0xC4));

        display_ = new CustomLcdDisplay(io_handle, panel_handle,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        // backlight
        backlight_ = new PwmBacklight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        backlight_->RestoreBrightness();

    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

public:
    CustomC3Board() : boot_button_(BOOT_BUTTON_GPIO) {
        // Initialize I2C bus (Codec and Touch on same bus)
        InitializeI2c();
        InitializeCst816DTouchPad();
        // Initialize LCD Display
        InitializeSpi();
        InitializeGc9a01Display();
        InitializeButtons();
        InitializePowerSaveTimer();
    }

    ~CustomC3Board() {
        if (touchpad_timer_) {
            esp_timer_stop(touchpad_timer_);
            esp_timer_delete(touchpad_timer_);
            touchpad_timer_ = nullptr;
        }
        if (cst816d_) {
            delete cst816d_;
            cst816d_ = nullptr;
        }
        if (power_save_timer_) {
            delete power_save_timer_;
            power_save_timer_ = nullptr;
        }
        if (display_) {
            delete display_;
            display_ = nullptr;
        }
        if (backlight_) {
            delete backlight_;
            backlight_ = nullptr;
        }
        if (audio_codec_) {
            delete audio_codec_;
            audio_codec_ = nullptr;
        }
        if (i2c_bus_) {
            i2c_del_master_bus(i2c_bus_);
            i2c_bus_ = nullptr;
        }
    }


    virtual Led* GetLed() override {
        return nullptr;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        return backlight_;
    }

    virtual AudioCodec* GetAudioCodec() override {
        return audio_codec_;
    }

    Cst816d* GetTouchpad() {
        return cst816d_;
    }

};

DECLARE_BOARD(CustomC3Board);
