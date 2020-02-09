#include <Adafruit_GFX.h>    // Core graphics library
#include <Arduino_ST7789_Comp.h> // Hardware-specific library for ST7789
#include "esp_camera.h"
#include <SPI.h>             // Arduino SPI library
 
// ST7789 TFT module connections
#define TFT_RST         2 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC          4
#define TFT_MOSI        14  // Data out
#define TFT_SCLK        15  // Clock out

//// Initialize Adafruit ST7789 TFT library
Arduino_ST7789 tft = Arduino_ST7789(TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK);

#define SCREEN_WIDTH  240 
#define SCREEN_HEIGHT 240 

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

static camera_config_t camera_config = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM, 
  .pin_sscb_sda = SIOD_GPIO_NUM,  
  .pin_sscb_scl = SIOC_GPIO_NUM,
     
  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM, 
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,  
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,

  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  
  .pixel_format = PIXFORMAT_RGB565,
  .frame_size = FRAMESIZE_HQVGA,
  .jpeg_quality = 12,
  .fb_count = 1,
};

byte x,y;
uint8_t oldpixel, newpixel;
int quanterror;

esp_err_t camera_init(){
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.print("Camera Init Failed");
        return err;
    }
    sensor_t * s = esp_camera_sensor_get();
    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV2640_PID) {
        s->set_vflip(s, 1);//flip it back
        s->set_brightness(s, 1);//up the blightness just a bit
        s->set_contrast(s, 1);
    }
  
    Serial.print("Camera Init OK");
    return ESP_OK;
}

void showImage(void)
{
    camera_fb_t *fb = esp_camera_fb_get();

    tft.drawEsp32camImage(0, 0, 240, 480, (uint8_t*)fb->buf);

    esp_camera_fb_return(fb);
}

 
void setup(void) {
    Serial.begin(9600);
    
    // if the display has CS pin try with SPI_MODE0
    tft.init(240, 240);    // Init ST7789 display 240x240 pixel
    
    // if the screen is flipped, remove this command
    tft.setRotation(2);
    
    Serial.println(F("Initialized"));
    
    uint16_t time = millis();
    tft.fillScreen( WHITE);
    time = millis() - time;
    
    camera_init();
    
    
    Serial.println("done");
    delay(1000);
}
 
void loop() {
    showImage();

    //delay(10);
}
 