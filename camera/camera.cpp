#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"

const int SD_PIN_CS = 21;
unsigned long frameInterval = 500; // Capture every 500ms
unsigned long lastCaptureTime = 0;
int imageCount = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000); // Allow power stabilization

    Serial.println("Initializing...");

    // Initialize Camera
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    if (esp_camera_init(&config) != ESP_OK)
    {
        Serial.println("Camera init failed!");
        while (1)
            ;
    }

    // Initialize SD card
    if (!SD.begin(SD_PIN_CS))
    {
        Serial.println("SD card initialization failed!");
        while (1)
            ;
    }

    Serial.println("Camera and SD initialized. Starting continuous capture...");
}

void loop()
{
    unsigned long now = millis();

    if (now - lastCaptureTime >= frameInterval)
    {
        lastCaptureTime = now;
        Serial.println("Capturing image...");

        // Capture Image
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
        {
            Serial.println("Camera capture failed!");
            return;
        }

        // Create filename
        char filename[32];
        sprintf(filename, "/image_%05d.jpg", imageCount++);

        // Save to SD card
        File file = SD.open(filename, FILE_WRITE);
        if (!file)
        {
            Serial.println("Failed to open file for writing.");
        }
        else
        {
            file.write(fb->buf, fb->len);
            Serial.printf("Saved: %s (%d bytes)\n", filename, fb->len);
            file.close();
        }

        esp_camera_fb_return(fb);
    }
}