    /*
    * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
    *
    * SPDX-License-Identifier: Apache-2.0
    */

    #include "esp_err.h"
    #include "esp_log.h"
    #include "esp_check.h"
    #include "driver/i2c.h"
    #include "driver/gpio.h"
    #include "driver/spi_master.h"
    #include "esp_lcd_panel_io.h"
    #include "esp_lcd_panel_vendor.h"
        #include "esp_lcd_panel_ops.h"
    #include "esp_lvgl_port.h"

    #include "esp_lcd_touch_ft5x06.h"

    /* LCD size */
    #define LCD_H_RES   (240)
    #define LCD_V_RES   (320)

    /* LCD settings */
    #define LCD_SPI_NUM         (SPI3_HOST)
    #define LCD_PIXEL_CLK_HZ    (40 * 1000 * 1000)
    #define LCD_CMD_BITS        (8)
    #define LCD_PARAM_BITS      (8)
    #define LCD_COLOR_SPACE     (ESP_LCD_COLOR_SPACE_RGB)
    #define LCD_BITS_PER_PIXEL  (16)
    #define LCD_DRAW_BUFF_DOUBLE (1)
    #define LCD_DRAW_BUFF_HEIGHT (50)
    #define LCD_BL_ON_LEVEL     (1)

    /* LCD pins */
    #define LCD_GPIO_SCLK (GPIO_NUM_18)
    #define LCD_GPIO_MOSI (GPIO_NUM_23)
    #define LCD_GPIO_RST (GPIO_NUM_NC)
    #define LCD_GPIO_DC (GPIO_NUM_4)
    #define LCD_GPIO_CS (GPIO_NUM_5)
    #define LCD_GPIO_BL (GPIO_NUM_17) //(GPIO_NUM_27)


    /* Touch settings */
    #define TOUCH_I2C_NUM       (0)
    #define TOUCH_I2C_CLK_HZ    (400000)

    /* LCD touch pins */
    #define TOUCH_I2C_SCL       (GPIO_NUM_22)
    #define TOUCH_I2C_SDA       (GPIO_NUM_21)
    #define TOUCH_GPIO_INT      (GPIO_NUM_NC)

    static const char *TAG = "EXAMPLE";
    static lv_group_t *grp;

    // LVGL image declare
    LV_IMG_DECLARE(esp_logo)

    /* LCD IO and panel */
    static esp_lcd_panel_io_handle_t lcd_io = NULL;
    static esp_lcd_panel_handle_t lcd_panel = NULL;
    static esp_lcd_touch_handle_t touch_handle = NULL;

    /* LVGL display and touch */
    static lv_display_t *lvgl_disp = NULL;
    static lv_indev_t *lvgl_touch_indev = NULL;
    static lv_indev_t *encoder = NULL;
    static button_handle_t btn_hdl = NULL;
    static esp_err_t app_lcd_init(void)
    {
        esp_err_t ret = ESP_OK;

        /* LCD backlight */
        gpio_config_t bk_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << LCD_GPIO_BL
        };
        ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

        /* LCD initialization */
        ESP_LOGD(TAG, "Initialize SPI bus");
        const spi_bus_config_t buscfg = {
            .sclk_io_num = LCD_GPIO_SCLK,
            .mosi_io_num = LCD_GPIO_MOSI,
            .miso_io_num = GPIO_NUM_NC,
            .quadwp_io_num = GPIO_NUM_NC,
            .quadhd_io_num = GPIO_NUM_NC,
            .max_transfer_sz = LCD_H_RES * LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
        };
        ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

        ESP_LOGD(TAG, "Install panel IO");
        const esp_lcd_panel_io_spi_config_t io_config = {
            .dc_gpio_num = LCD_GPIO_DC,
            .cs_gpio_num = LCD_GPIO_CS,
            .pclk_hz = LCD_PIXEL_CLK_HZ,
            .lcd_cmd_bits = LCD_CMD_BITS,
            .lcd_param_bits = LCD_PARAM_BITS,
            .spi_mode = 0,
            .trans_queue_depth = 10,
        };
        ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

        ESP_LOGD(TAG, "Install LCD driver");
        const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = LCD_GPIO_RST,
            .color_space = LCD_COLOR_SPACE,
            .bits_per_pixel = LCD_BITS_PER_PIXEL,
        };
        ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

        esp_lcd_panel_reset(lcd_panel);
        esp_lcd_panel_init(lcd_panel);
        esp_lcd_panel_mirror(lcd_panel, true, true);
        esp_lcd_panel_disp_on_off(lcd_panel, true);

        /* LCD backlight on */
        ESP_ERROR_CHECK(gpio_set_level(LCD_GPIO_BL, LCD_BL_ON_LEVEL));

        return ret;

    err:
        if (lcd_panel) {
            esp_lcd_panel_del(lcd_panel);
        }
        if (lcd_io) {
            esp_lcd_panel_io_del(lcd_io);
        }
        spi_bus_free(LCD_SPI_NUM);
        return ret;
    }

    static esp_err_t app_touch_init(void)
    {
        /* Initilize I2C */
        const i2c_config_t i2c_conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = TOUCH_I2C_SDA,
            .sda_pullup_en = GPIO_PULLUP_DISABLE,
            .scl_io_num = TOUCH_I2C_SCL,
            .scl_pullup_en = GPIO_PULLUP_DISABLE,
            .master.clk_speed = TOUCH_I2C_CLK_HZ
        };
        ESP_RETURN_ON_ERROR(i2c_param_config(TOUCH_I2C_NUM, &i2c_conf), TAG, "I2C configuration failed");
        ESP_RETURN_ON_ERROR(i2c_driver_install(TOUCH_I2C_NUM, i2c_conf.mode, 0, 0, 0), TAG, "I2C initialization failed");

        /* Initialize touch HW */
        const esp_lcd_touch_config_t tp_cfg = {
            .x_max = LCD_H_RES,
            .y_max = LCD_V_RES,
            .rst_gpio_num = GPIO_NUM_NC, // Shared with LCD reset
            .int_gpio_num = TOUCH_GPIO_INT,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 0,
                .mirror_x = 1,
                .mirror_y = 1,
            },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TOUCH_I2C_NUM, &tp_io_config, &tp_io_handle), TAG, "");
        return esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &touch_handle);
    }

    static esp_err_t app_lvgl_init(void)
    {
        /* Initialize LVGL */
        const lvgl_port_cfg_t lvgl_cfg = {
            .task_priority = 4,         /* LVGL task priority */
            .task_stack = 4096,         /* LVGL task stack size */
            .task_affinity = -1,        /* LVGL task pinned to core (-1 is no affinity) */
            .task_max_sleep_ms = 500,   /* Maximum sleep in LVGL task */
            .timer_period_ms = 5        /* LVGL timer tick period in ms */
        };
        ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

        /* Add LCD screen */
        ESP_LOGD(TAG, "Add LCD screen");
        const lvgl_port_display_cfg_t disp_cfg = {
            .io_handle = lcd_io,
            .panel_handle = lcd_panel,
            .buffer_size = LCD_H_RES * LCD_DRAW_BUFF_HEIGHT,
            .double_buffer = LCD_DRAW_BUFF_DOUBLE,
            .hres = LCD_H_RES,
            .vres = LCD_V_RES,
            .monochrome = false,
            .color_format = LV_COLOR_FORMAT_RGB565,
            .rotation = {
                .swap_xy = false,
                .mirror_x = false,
                .mirror_y = true,
            },
            .flags = {
                .buff_dma = true,
                .swap_bytes = true,
            }
        };
        lvgl_disp = lvgl_port_add_disp(&disp_cfg);

        /* Add touch input (for selected screen) */
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lvgl_disp,
            .handle = touch_handle,
        };
        lvgl_touch_indev = lvgl_port_add_touch(&touch_cfg);

        const button_config_t btn_cfg = {
            .type = BUTTON_TYPE_GPIO,
            .gpio_button_config = {
                .gpio_num = GPIO_NUM_0,
                .active_level = 0,
            }};
        const knob_config_t knob_cfg = {
            .default_direction = 0,
            .gpio_encoder_a = GPIO_NUM_32,
            .gpio_encoder_b = GPIO_NUM_27,
        };

        const lvgl_port_encoder_cfg_t enc_cfg = {
            .disp = lvgl_disp,
            .encoder_a_b = &knob_cfg,
            .encoder_enter = &btn_cfg,
        };
        encoder = lvgl_port_add_encoder(&enc_cfg);
        grp = lv_group_create();
        lv_indev_set_group(encoder, grp);
        lv_group_set_wrap(grp, false);
        lv_group_set_default(grp);

        if (encoder != NULL)
            ESP_LOGI(TAG, "ENCODER ADDED");

        return ESP_OK;
    }

    static void _app_button_cb(lv_event_t *e)
    {
        lv_disp_rotation_t rotation = lv_disp_get_rotation(lvgl_disp);
        rotation++;
        if (rotation > LV_DISPLAY_ROTATION_270) {
            rotation = LV_DISPLAY_ROTATION_0;
        }

        /* LCD HW rotation */
        lv_disp_set_rotation(lvgl_disp, rotation);
    }

    static void app_main_display(void)
    {
        lv_obj_t *scr = lv_scr_act();

        /* Task lock */
        lvgl_port_lock(0);

        /* Your LVGL objects code here .... */

        /* Create image */
        lv_obj_t *img_logo = lv_img_create(scr);
        lv_img_set_src(img_logo, &esp_logo);
        lv_obj_align(img_logo, LV_ALIGN_TOP_MID, 0, 20);

        /* Label */
        lv_obj_t *label = lv_label_create(scr);
        lv_obj_set_width(label, LCD_H_RES);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

        lv_label_set_text(label, LV_SYMBOL_BELL" Hello world Espressif and LVGL "LV_SYMBOL_BELL"\n "LV_SYMBOL_WARNING" For simplier initialization, use BSP "LV_SYMBOL_WARNING);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 20);

        /* Button */
        lv_obj_t *btn = lv_btn_create(scr);
        label = lv_label_create(btn);
        lv_label_set_text_static(label, "Rotate screen");
        lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -30);
        lv_obj_add_event_cb(btn, _app_button_cb, LV_EVENT_CLICKED, NULL);

        /* Task unlock */
        lvgl_port_unlock();
    }

    void app_main(void)
    {
        /* LCD HW initialization */
        ESP_ERROR_CHECK(app_lcd_init());

        /* Touch initialization */
        ESP_ERROR_CHECK(app_touch_init());

        /* LVGL initialization */
        ESP_ERROR_CHECK(app_lvgl_init());

        /* Show LVGL objects */
        app_main_display();
    }
