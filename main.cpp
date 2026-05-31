#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h" 
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"

static const char *TAG = "STUDYMETER";

// --- PİN TANIMLAMALARI ---
#define I2C_MASTER_SDA_IO           GPIO_NUM_1    
#define I2C_MASTER_SCL_IO           GPIO_NUM_2    
#define HALL_SENSOR_PIN             GPIO_NUM_4
#define BUZZER_PIN                  GPIO_NUM_5

#define PIN_NUM_CLK                 GPIO_NUM_12   
#define PIN_NUM_MOSI                GPIO_NUM_11   
#define PIN_NUM_RST                 GPIO_NUM_13   
#define PIN_NUM_DC                  GPIO_NUM_9    
#define PIN_NUM_CS                  GPIO_NUM_10   
#define PIN_NUM_BCKL                GPIO_NUM_21   

// --- DONANIM AYARLARI ---
#define I2C_MASTER_NUM              I2C_NUM_0     
#define I2C_MASTER_FREQ_HZ          400000        
#define MPU6050_ADDR                0x68          
#define MPU6050_PWR_MGMT_1          0x6B          
#define MPU6050_ACCEL_XOUT_H        0x3B          

#define BUZZER_LEDC_MODE            LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_CHANNEL         LEDC_CHANNEL_0
#define BUZZER_LEDC_TIMER           LEDC_TIMER_0
#define BUZZER_LEDC_RESOLUTION      LEDC_TIMER_13_BIT
#define BUZZER_FREQ_HZ              2300          

#define LCD_HOST                    SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ          (20 * 1000 * 1000)
#define LCD_H_RES                   160 // Yatay Mod İçin 160
#define LCD_V_RES                   80  // Yatay Mod İçin 80

// Renk Kodları
#define COLOR_BLACK                 0x0000
#define COLOR_WHITE                 0xFFFF
#define COLOR_RED                   0xF800
#define COLOR_GREEN                 0x07E0
#define COLOR_BLUE                  0x001F
#define COLOR_YELLOW                0xFFE0
#define COLOR_MAGENTA               0xF81F
#define COLOR_CYAN                  0x07FF

esp_lcd_panel_handle_t panel_handle = NULL;

typedef enum {
    POS_UNKNOWN = 0, POS_1_X_POZITIF, POS_2_X_NEGATIF,
    POS_3_Y_POZITIF, POS_4_Y_NEGATIF, POS_5_Z_POZITIF, POS_6_Z_NEGATIF
} CubePosition;

// --- DAHİLİ SÜRÜCÜ BAŞLATMALARI ---
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t mpu6050_wake(void) {
    uint8_t write_buf[2] = {MPU6050_PWR_MGMT_1, 0x00}; 
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, write_buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t mpu6050_sleep(void) {
    uint8_t write_buf[2] = {MPU6050_PWR_MGMT_1, 0x40}; 
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, write_buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t mpu6050_read_raw(uint8_t *data) {
    uint8_t reg_addr = MPU6050_ACCEL_XOUT_H;
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg_addr, 1, data, 14, pdMS_TO_TICKS(100));
}

static void hall_sensor_init(void) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << HALL_SENSOR_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}

static void buzzer_init(void) {
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = BUZZER_LEDC_MODE;
    ledc_timer.timer_num        = BUZZER_LEDC_TIMER;
    ledc_timer.duty_resolution  = BUZZER_LEDC_RESOLUTION;
    ledc_timer.freq_hz          = BUZZER_FREQ_HZ; 
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.speed_mode     = BUZZER_LEDC_MODE;
    ledc_channel.channel        = BUZZER_LEDC_CHANNEL;
    ledc_channel.timer_sel      = BUZZER_LEDC_TIMER;
    ledc_channel.gpio_num       = BUZZER_PIN;
    ledc_channel.duty           = 0; 
    ledc_channel.hpoint         = 0;
    ledc_channel_config(&ledc_channel);
}

static void buzzer_beep(uint32_t duration_ms) {
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 4096);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

// --- TFT BAŞLATMA (ARKADAŞININ V6 KODUYLA) ---
static void tft_init(void) {
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_BCKL, 1);

    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.miso_io_num = GPIO_NUM_NC;
    buscfg.quadwp_io_num = GPIO_NUM_NC;
    buscfg.quadhd_io_num = GPIO_NUM_NC;
    buscfg.max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.dc_gpio_num = PIN_NUM_DC;
    io_config.cs_gpio_num = PIN_NUM_CS;
    io_config.pclk_hz = LCD_PIXEL_CLOCK_HZ;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = PIN_NUM_RST;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR; 
    panel_config.bits_per_pixel = 16;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    
    // YATAY KÜP İÇİN DÖNDÜRME VE ZIT OFFSET (Arkadaşında 26,1 idi, bizde 1,26)
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    // !!! YENİ EKLENEN SATIR !!!
    // IPS ekranda yatay modda yazıların aynalı çıkmaması için dikey eksende aynalama yapıyoruz
    esp_lcd_panel_mirror(panel_handle, false, true); // (X_mirror, Y_mirror) 
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 1, 26)); 
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false)); 
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
}

// Tüm ekranı boyayan fonksiyon (Arkadaşının DMA mantığıyla)
void tft_fill_color(uint16_t color) {
    if (panel_handle == NULL) return;
    
    size_t pixel_count = LCD_H_RES * LCD_V_RES;
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(pixel_count * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!buffer) return;

    uint16_t swapped_color = (color >> 8) | (color << 8);
    for (size_t i = 0; i < pixel_count; i++) {
        buffer[i] = swapped_color;
    }

    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, buffer);
    free(buffer);
}

// --- MİNİMALİST 8x8 FONT (Sadece Rakamlar ve Gerekli Karakterler) ---
// Sırayla: ' ', '0'-'9', ':' 
const uint8_t font8x8[13][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Bosluk (Index 0)
    {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x00, 0x00}, // 0 (Index 1)
    {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, 0x00, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46, 0x00, 0x00, 0x00}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31, 0x00, 0x00, 0x00}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, 0x00, 0x00}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39, 0x00, 0x00, 0x00}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00, 0x00, 0x00}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03, 0x00, 0x00, 0x00}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00, 0x00}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00, 0x00}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00},  // : (Index 11)
    {0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C}  // Gülücük (Index 12)
};

// --- KOCAMAN VE 90 DERECE DÖNÜK GÜLÜCÜK MATRİSİ ---
// Saat yönünde 90 derece yatırılmış 8x8 yüz (Satır bazlı)
const uint16_t icon_smiley_16[16] = {
    0x07E0, // 0000 0111 1110 0000 (Kafa üst)
    0x1818, // 0001 1000 0001 1000
    0x2004, // 0010 0000 0000 0100
    0x4002, // 0100 0000 0000 0010
    0x4662, // 0100 0110 0110 0010 (Gözler)
    0x8661, // 1000 0110 0110 0001 (Gözler)
    0x8001, // 1000 0000 0000 0001
    0x8001, // 1000 0000 0000 0001
    0x8001, // 1000 0000 0000 0001
    0x8811, // 1000 1000 0001 0001 (Ağız kenarları)
    0x4422, // 0100 0100 0010 0010 (Ağız kavis)
    0x43C2, // 0100 0011 1100 0010 (Ağız alt orta)
    0x2004, // 0010 0000 0000 0100
    0x1818, // 0001 1000 0001 1000
    0x07E0, // 0000 0111 1110 0000 (Kafa alt)
    0x0000  // 0000 0000 0000 0000
};

// 16-Bit Matrisleri okuyabilen YENİ Çizim Fonksiyonu
void tft_draw_big_icon(int start_x, int start_y, const uint16_t icon[16], int scale, uint16_t fg_color, uint16_t bg_color) {
    if (panel_handle == NULL) return;
    
    int size = 16 * scale; // 16x16 matrisi scale ile çarpıyoruz
    size_t pixel_count = size * size;
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(pixel_count * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!buffer) return;

    uint16_t sw_fg = (fg_color >> 8) | (fg_color << 8);
    uint16_t sw_bg = (bg_color >> 8) | (bg_color << 8);

    for (int row = 0; row < size; row++) {
        for (int col = 0; col < size; col++) {
            uint16_t icon_row_data = icon[row / scale];
            // 16 bitlik veriyi okumak için kaydırma işlemi: 15 - (col / scale)
            bool pixel_on = (icon_row_data & (1 << (15 - (col / scale)))) != 0;
            
            buffer[row * size + col] = pixel_on ? sw_fg : sw_bg;
        }
    }
    esp_lcd_panel_draw_bitmap(panel_handle, start_x, start_y, start_x + size, start_y + size, buffer);
    free(buffer);
}

// Ekrana 16x16 boyutunda (2x ölçekli) karakter basan fonksiyon
void tft_draw_char(int x, int y, char c, uint16_t fg_color, uint16_t bg_color) {
    if (panel_handle == NULL) return;
    
    int font_idx = 0;
    if (c == ' ') font_idx = 0;
    else if (c >= '0' && c <= '9') font_idx = c - '0' + 1;
    else if (c == ':') font_idx = 11;
    else if (c == '*') font_idx = 12; // Yıldız karakteri gelirse gülücük çiz
    else return; // Desteklenmeyen karakter
    
    // 8x8 fontu 2x ölçekleyerek 16x16 yapıyoruz (Ekranda net okunsun diye)
    size_t pixel_count = 16 * 16;
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(pixel_count * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!buffer) return;

    uint16_t sw_fg = (fg_color >> 8) | (fg_color << 8);
    uint16_t sw_bg = (bg_color >> 8) | (bg_color << 8);

    for (int row = 0; row < 16; row++) {
        for (int col = 0; col < 16; col++) {
            // 2x ölçekleme mantığı
            uint8_t font_row = font8x8[font_idx][col / 2];
            bool pixel_on = (font_row & (1 << (row / 2))) != 0;
            buffer[row * 16 + col] = pixel_on ? sw_fg : sw_bg;
        }
    }
    esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + 16, y + 16, buffer);
    free(buffer);
}

// Yan yana metin (Süre) yazdırma fonksiyonu
void tft_print_time(int start_x, int start_y, int mins, int secs, uint16_t fg_color, uint16_t bg_color) {
    char time_str[6];
    sprintf(time_str, "%02d:%02d", mins, secs); // "25:00" formatına çevir
    
    int current_x = start_x;
    for (int i = 0; i < 5; i++) {
        tft_draw_char(current_x, start_y, time_str[i], fg_color, bg_color);
        current_x += 16; // Her harften sonra 16 piksel sağa kay
    }
}

// --- POZİSYON ALGILAMA ---
CubePosition detect_position(float ax, float ay, float az) {
    float threshold = 12000.0;
    if (ax > threshold) return POS_1_X_POZITIF;
    if (ax < -threshold) return POS_2_X_NEGATIF;
    if (ay > threshold) return POS_3_Y_POZITIF;
    if (ay < -threshold) return POS_4_Y_NEGATIF;
    if (az > threshold) return POS_5_Z_POZITIF;
    if (az < -threshold) return POS_6_Z_NEGATIF;
    return POS_UNKNOWN; 
}

// --- ANA UYGULAMA ---
extern "C" void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    hall_sensor_init();
    buzzer_init();
    tft_init(); 
    
    mpu6050_sleep();
    gpio_set_level(PIN_NUM_BCKL, 0); 
    ESP_LOGW(TAG, "Sistem UYKUDA. Baslamak icin miknatis okutun...");

    uint8_t raw_data[14];
    float ax_filtered = 0, ay_filtered = 0, az_filtered = 0;
    float alpha = 0.1; 

    bool system_active = false;      
    int last_hall_state = 1;         
    CubePosition last_logged_pos = POS_UNKNOWN;
    
    uint32_t mode_ticks = 0;          
    uint32_t current_mode_time_s = 0; 
    uint32_t target_time_s = 0;       
    bool is_countdown = false;        
    uint32_t total_session_time_s = 0; // Yeni genel sayaç

    while (1) {
        int current_hall_state = gpio_get_level(HALL_SENSOR_PIN);

        // --- KART OKUTMA (TOGGLE) ---
        if (last_hall_state == 1 && current_hall_state == 0) {
            system_active = !system_active; 
            if (system_active) {
                mpu6050_wake(); 
                gpio_set_level(PIN_NUM_BCKL, 1);

                // --- DEVASA AÇILIŞ GÜLÜCÜĞÜ ---
                tft_fill_color(COLOR_YELLOW); // Sapsarı ekran
                
                // 16x16 ikonu 3 kat büyütüyoruz (16 * 3 = 48x48 piksel boyutunda olacak)
                tft_draw_big_icon(56, 16, icon_smiley_16, 3, COLOR_BLACK, COLOR_YELLOW);

                ESP_LOGI(TAG, ">>> STUDYMETER AKTIF <<<");
                buzzer_beep(100); vTaskDelay(pdMS_TO_TICKS(50)); buzzer_beep(100); 
                // Level Up Melodisi (Farklı bekleme süreleriyle ritmik)
                //buzzer_beep(100); vTaskDelay(pdMS_TO_TICKS(50)); 
                //buzzer_beep(100); vTaskDelay(pdMS_TO_TICKS(50)); 
                //buzzer_beep(300);
                
                vTaskDelay(pdMS_TO_TICKS(2000)); // Gülücüğü 2 saniye ekranda tut
            } else {
                // --- KAPANIŞ EKRANI VE ÖZETİ ---
                tft_fill_color(COLOR_BLUE);
                int t_mins = total_session_time_s / 60;
                int t_secs = total_session_time_s % 60;
                
                // Toplam süreyi mavi ekrana beyaz yazıyla bas
                tft_print_time(40, 32, t_mins, t_secs, COLOR_WHITE, COLOR_BLUE);
                ESP_LOGW(TAG, "Oturum Kapandi. Toplam Calisma: %02d:%02d", t_mins, t_secs);
                
                buzzer_beep(100); vTaskDelay(pdMS_TO_TICKS(100)); buzzer_beep(300); // Kapanış melodisi
                
                vTaskDelay(pdMS_TO_TICKS(3000)); // 3 saniye ekranda tut (Gurur tablosu)

                mpu6050_sleep(); 
                tft_fill_color(COLOR_BLACK);     
                gpio_set_level(PIN_NUM_BCKL, 0); 
                
                total_session_time_s = 0; // Yeni oturum için sıfırla 
            }
            vTaskDelay(pdMS_TO_TICKS(500)); 
        }
        last_hall_state = current_hall_state;

        // --- AKTİF MOD İŞLEMLERİ ---
        if (system_active) {
            if (mpu6050_read_raw(raw_data) == ESP_OK) {
                int16_t ax = (raw_data[0] << 8) | raw_data[1];
                int16_t ay = (raw_data[2] << 8) | raw_data[3];
                int16_t az = (raw_data[4] << 8) | raw_data[5];
                
                ax_filtered = (alpha * ax) + ((1.0 - alpha) * ax_filtered);
                ay_filtered = (alpha * ay) + ((1.0 - alpha) * ay_filtered);
                az_filtered = (alpha * az) + ((1.0 - alpha) * az_filtered);

                CubePosition current_pos = detect_position(ax_filtered, ay_filtered, az_filtered);

                // KÜPÜN YÜZÜ DEĞİŞTİ (YENİ MOD)
                if (current_pos != last_logged_pos && current_pos != POS_UNKNOWN) {
                    last_logged_pos = current_pos;
                    mode_ticks = 0;          
                    current_mode_time_s = 0; 
                    buzzer_beep(50);         

                    switch (current_pos) {
                        case POS_1_X_POZITIF:
                            ESP_LOGI(TAG, "MOD: Pomodoro Basladi! (25 Dk)");
                            tft_fill_color(COLOR_RED);
                            target_time_s = 25 * 60; 
                            is_countdown = true;
                            break;
                        case POS_2_X_NEGATIF:
                            ESP_LOGI(TAG, "MOD: Kisa Mola Basladi! (5 Dk)");
                            tft_fill_color(COLOR_GREEN);
                            target_time_s = 0.1 * 60;  
                            is_countdown = true;
                            break;
                        case POS_3_Y_POZITIF:
                            ESP_LOGI(TAG, "MOD: Uzun Mola Basladi! (15 Dk)");
                            tft_fill_color(COLOR_CYAN);
                            target_time_s = 15 * 60; 
                            is_countdown = true;
                            break;
                        case POS_4_Y_NEGATIF:
                            ESP_LOGI(TAG, "MOD: Proje Calisma Kronometresi Aktif.");
                            tft_fill_color(COLOR_BLUE);
                            is_countdown = false; 
                            break;
                        case POS_5_Z_POZITIF:
                            ESP_LOGI(TAG, "MOD: Serbest Calisma Kronometresi Aktif.");
                            tft_fill_color(COLOR_YELLOW);
                            is_countdown = false; 
                            break;
                        case POS_6_Z_NEGATIF:
                            ESP_LOGW(TAG, "MOD: Zaman Durduruldu!");
                            tft_fill_color(COLOR_MAGENTA);
                            break;
                        default: break;
                    }
                }

                // --- ZAMAN YÖNETİMİ (1 Saniyelik Tikler) ---
                if (current_pos != POS_6_Z_NEGATIF && current_pos != POS_UNKNOWN) {
                    mode_ticks++;
                    if (mode_ticks >= 10) { // 1 saniye doldu
                        mode_ticks = 0;
                        current_mode_time_s++;
                        total_session_time_s++; // Oturum boyunca sürekli artar

                        // O anki modun arka plan rengini belirle
                        uint16_t current_bg_color = COLOR_BLACK;
                        if (current_pos == POS_1_X_POZITIF) current_bg_color = COLOR_RED;
                        else if (current_pos == POS_2_X_NEGATIF) current_bg_color = COLOR_GREEN;
                        else if (current_pos == POS_3_Y_POZITIF) current_bg_color = COLOR_CYAN;
                        else if (current_pos == POS_4_Y_NEGATIF) current_bg_color = COLOR_BLUE;
                        else if (current_pos == POS_5_Z_POZITIF) current_bg_color = COLOR_YELLOW;

                        if (is_countdown) {
                            uint32_t remaining = target_time_s - current_mode_time_s;
                            int mins = remaining / 60;
                            int secs = remaining % 60;
                            
                            // Ekrana tam ortaya (X:40, Y:32 koordinatlarına) beyaz yazıyla süreyi bas
                            tft_print_time(40, 32, mins, secs, COLOR_BLACK, current_bg_color);
                            
                            if (current_mode_time_s % 10 == 0) {
                                ESP_LOGI(TAG, "Kalan Sure: %02d:%02d", mins, secs);
                            }

                            if (current_mode_time_s >= target_time_s) {
                                ESP_LOGW(TAG, "!!! SURE BITTI !!! Lutfen kutuyu cevirin.");
                                buzzer_beep(200); vTaskDelay(pdMS_TO_TICKS(100)); buzzer_beep(200);
                                current_mode_time_s = 0; 
                            }
                        } else {
                            // Kronometre Modu (İleri Sayım)
                            int mins = current_mode_time_s / 60;
                            int secs = current_mode_time_s % 60;
                            
                            tft_print_time(40, 32, mins, secs, COLOR_BLACK, current_bg_color); // Sarı/Mavi üstüne siyah daha iyi okunur
                            
                            if (current_mode_time_s % 10 == 0) {
                                ESP_LOGI(TAG, "Calisma Suresi: %02d:%02d", mins, secs);
                            }
                        }
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}