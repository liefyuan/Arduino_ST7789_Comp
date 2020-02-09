# Arduino_ST7789_Comp
Comp SPI library for the ST7789 240x240 IPS display

## Configuration
Custom SPI pins

## Extra Features
- drawImage() from RAM
- drawImageF() from flash (PROGMEM)
- drawEsp32camImage from esp32cam image

## Connections:

All SPI pins support customization

### use defalut spi pins or use custom spi pins

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


### show esp32cam image in 240*240 screen

	void showImage(void)
	{
	    camera_fb_t *fb = esp_camera_fb_get();
	    tft.drawEsp32camImage(0, 0, 240, 480, (uint8_t*)fb->buf);
	    esp_camera_fb_return(fb);
	}
