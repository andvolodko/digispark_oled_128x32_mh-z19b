

#include "EEPROM.h"
#include "font6x8.h"
#include "font16x24.h"
#include "lines.h"
#include "logo.h"

#include <SoftSerial.h>
#include <TinyPinChange.h>

SoftSerial mySerial(4, 1); // RX, TX

uint8_t cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
unsigned char co2SensorResponse[9];

uint8_t graphData[128];
uint8_t graphPosition = 0;

unsigned int oldPPM = 0;

// ----------------------------------------------------------------------------
void ssd1306_init(void);
void ssd1306_xfer_start(void);
void ssd1306_xfer_stop(void);
void ssd1306_send_byte(uint8_t byte);
void ssd1306_send_command(uint8_t command);
void ssd1306_send_data_start(void);
void ssd1306_send_data_stop(void);
void ssd1306_setpos(uint8_t x, uint8_t y);
void ssd1306_fillscreen(uint8_t fill_Data);
void ssd1306_char_f6x8(uint8_t x, uint8_t y, const char ch[]);

unsigned long frames = 0;

void setup() {

  showLogo();

  ssd1306_char_f16x24(40, 0, "///");

  mySerial.begin(9600);
}

void showLogo() {
  ssd1306_init();
  ssd1306_fillscreen(0x00);
  //ssd1306_char_f6x8(8, 2, "000");
  draw_bitmap(0, 0, 128, 4, ssd1306xled_logo);
  delay(3000);
  ssd1306_fillscreen(0x00);
}

void loop() {

  mySerial.txMode();

  for (byte i = 0; i < 9; i++)
  {
    mySerial.write(cmd[i]);
  }

  mySerial.rxMode();

  bool received = false;
  int ls = 0;
  int hs = 0;
  int i = 0;
  int statusByte = 231;
  int temperature = 0;

  while (mySerial.available() > 0) {

    received = true;

    int val = mySerial.read();

    co2SensorResponse[i] = val;

    switch (i) {
      case 2:
        ls = val;
        break;
      case 3:
        hs = val;
        break;
      case 4:
        temperature = val - 40;
        break;
      case 5:
        statusByte = val;
        break;
    }
    i++;

  }

  if (received) {

    if (validateCo2Response()) {

      unsigned int ppm = (256 * ls) + hs;

      //New text at center position
      if (ppm >= 0 && ppm < 10000 && oldPPM != ppm) {
        char temp[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        itoa(ppm, temp, 10);

        if (ppm > 999) {
          ssd1306_char_f16x24(32, 0, temp);
        } else {
          if (oldPPM > 999) {
            //Clear previous text
            ssd1306_char_f16x24(32, 0, "    ");
          }
          ssd1306_char_f16x24(40, 0, temp);
        }

        oldPPM = ppm;
      }
    }

    /* itoa(temperature, temp, 10);
      ssd1306_char_f6x8(30, 1, "   ");
      ssd1306_char_f6x8(30, 1, temp); */
  }

  //Show captions
  ssd1306_char_f6x8(0, 0, "CO2");
  ssd1306_char_f6x8(109, 0, "PPM");

  //Save data for graph
  graphData[graphPosition] = 0;
  if (oldPPM > 400) {
    graphData[graphPosition] = 1;
  }
  if (oldPPM > 700) {
    graphData[graphPosition] = 2;
  }
  if (oldPPM > 1000) {
    graphData[graphPosition] = 3;
  }
  if (oldPPM > 1500) {
    graphData[graphPosition] = 4;
  }
  if (oldPPM > 2000) {
    graphData[graphPosition] = 5;
  }
  if (oldPPM > 3000) {
    graphData[graphPosition] = 6;
  }
  if (oldPPM > 4000) {
    graphData[graphPosition] = 7;
  }

  if (oldPPM > 0) {
    graphPosition++;
    if (graphPosition > 126) {
      graphPosition = 126;
      //Array shift to left for 1 new item
      memcpy(graphData, &graphData[1], sizeof(graphData) - sizeof(uint8_t));
    }
  }

  drawGraph();

  //Show detector 3 min preheating time indicator
  unsigned long runTime = millis();
  if (oldPPM > 0 && runTime < 180000) {
    if (frames % 2 == 0) {
      ssd1306_char_f6x8(0, 2, "+");
    } else {
      ssd1306_char_f6x8(0, 2, "-");
    }
  } else {
    ssd1306_char_f6x8(0, 2, " ");
  }

  frames++;

  delay(10000);
}

bool validateCo2Response() {
  byte crc = 0;
  for (int i = 1; i < 8; i++) {
    crc += co2SensorResponse[i];
  }
  crc = 256 - crc;
  //crc++;
  bool valid = co2SensorResponse[0] == 0xFF && co2SensorResponse[1] == 0x86 && co2SensorResponse[8] == crc;
  /* if (!valid) {
    Serial.println("CRC error: " + String(crc) + "/" + String(co2SensorResponse[8]));
    } */
  return valid;
}

void drawGraph() {
  ssd1306_setpos(0, 3);
  ssd1306_send_data_start();
  for (uint8_t i = 0; i < 127; i++)
  {
    uint8_t lineType = graphData[i];
    ssd1306_send_byte(pgm_read_byte(&ssd1306xled_lines[lineType]));
  }
  ssd1306_send_data_stop();
}

#define DIGITAL_WRITE_HIGH(PORT) PORTB |= (1 << PORT)
#define DIGITAL_WRITE_LOW(PORT) PORTB &= ~(1 << PORT)

// Some code based on "IIC_wtihout_ACK" by http://www.14blog.com/archives/1358
#ifndef SSD1306XLED_H
#define SSD1306XLED_H
// ---------------------	// Vcc,	Pin 1 on SSD1306 Board
// ---------------------	// GND,	Pin 2 on SSD1306 Board
#ifndef SSD1306_SCL
#define SSD1306_SCL		PB2	// SCL,	Pin 3 on SSD1306 Board
#endif
#ifndef SSD1306_SDA
#define SSD1306_SDA		PB0	// SDA,	Pin 4 on SSD1306 Board
#endif
#ifndef SSD1306_SA
#define SSD1306_SA		0x78	// Slave address
#endif

// ----------------------------------------------------------------------------
#endif
void ssd1306_init(void) {


  DDRB |= (1 << SSD1306_SDA); // Set port as output
  DDRB |= (1 << SSD1306_SCL); // Set port as output

  ssd1306_send_command(0xAE); // display off
  ssd1306_send_command(0x00); // Set Memory Addressing Mode
  ssd1306_send_command(0x10); // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
  ssd1306_send_command(0x40); // Set Page Start Address for Page Addressing Mode,0-7
  ssd1306_send_command(0x81); // Set COM Output Scan Direction
  ssd1306_send_command(0xCF); // ---set low column address
  ssd1306_send_command(0xA1); // ---set high column address
  ssd1306_send_command(0xC0); // --set start line address. Flip vertical C0 or C8
  ssd1306_send_command(0xA6); // --set contrast control register
  ssd1306_send_command(0xA0); // Flip horizontal A0 or A8
  ssd1306_send_command(0x1F); // --set segment re-map 0 to 127; 3F - 64; 1F - 32
  ssd1306_send_command(0xD3); // --set normal display
  ssd1306_send_command(0x00); // --set multiplex ratio(1 to 64)
  ssd1306_send_command(0xD5); //
  ssd1306_send_command(0x80); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
  ssd1306_send_command(0xD9); // -set display offset
  ssd1306_send_command(0xF1); // -not offset
  ssd1306_send_command(0xDA); // --set display clock divide ratio/oscillator frequency
  ssd1306_send_command(0x02); // --set divide ratio
  ssd1306_send_command(0xDB); // --set pre-charge period
  ssd1306_send_command(0x40); //
  ssd1306_send_command(0x20); // --set com pins hardware configuration
  ssd1306_send_command(0x02);
  ssd1306_send_command(0x8D); // --set vcomh
  ssd1306_send_command(0x14); // 0x20,0.77xVcc
  ssd1306_send_command(0xA4); // --set DC-DC enable
  ssd1306_send_command(0xA6); // Set display not inverted
  ssd1306_send_command(0xAF); // --turn on oled panel

/*
  DDRB |= (1 << SSD1306_SDA);  // Set port as output
  DDRB |= (1 << SSD1306_SCL); // Set port as output

  ssd1306_send_command(0xAE); // display off
  ssd1306_send_command(0x00); // Set Memory Addressing Mode
  ssd1306_send_command(0x10); // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
  ssd1306_send_command(0x40); // Set Page Start Address for Page Addressing Mode,0-7
  ssd1306_send_command(0x81); // Set COM Output Scan Direction
  ssd1306_send_command(0xCF); // ---set low column address
  ssd1306_send_command(0xA1); // ---set high column address
  ssd1306_send_command(0xC0); // --set start line address
  ssd1306_send_command(0xA6); // --set contrast control register
  ssd1306_send_command(0xA0);
  ssd1306_send_command(0x3F); // --set segment re-map 0 to 127
  ssd1306_send_command(0xD3); // --set normal display
  ssd1306_send_command(0x00); // --set multiplex ratio(1 to 64)
  ssd1306_send_command(0xD5); // 
  ssd1306_send_command(0x80); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
  ssd1306_send_command(0xD9); // -set display offset
  ssd1306_send_command(0xF1); // -not offset
  ssd1306_send_command(0xDA); // --set display clock divide ratio/oscillator frequency
  ssd1306_send_command(0x12); // --set divide ratio
  ssd1306_send_command(0xDB); // --set pre-charge period
  ssd1306_send_command(0x40); // 
  ssd1306_send_command(0x20); // --set com pins hardware configuration
  ssd1306_send_command(0x02);
  ssd1306_send_command(0x8D); // --set vcomh
  ssd1306_send_command(0x14); // 0x20,0.77xVcc
  ssd1306_send_command(0xA4); // --set DC-DC enable
  ssd1306_send_command(0xA6); // 
  ssd1306_send_command(0xAF); // --turn on oled panel 
  */
}

void ssd1306_xfer_start(void) {
  DIGITAL_WRITE_HIGH(SSD1306_SCL);	// Set to HIGH
  DIGITAL_WRITE_HIGH(SSD1306_SDA);	// Set to HIGH
  DIGITAL_WRITE_LOW(SSD1306_SDA);		// Set to LOW
  DIGITAL_WRITE_LOW(SSD1306_SCL);		// Set to LOW
}

void ssd1306_xfer_stop(void) {
  DIGITAL_WRITE_LOW(SSD1306_SCL);		// Set to LOW
  DIGITAL_WRITE_LOW(SSD1306_SDA);		// Set to LOW
  DIGITAL_WRITE_HIGH(SSD1306_SCL);	// Set to HIGH
  DIGITAL_WRITE_HIGH(SSD1306_SDA);	// Set to HIGH
}

void ssd1306_send_byte(uint8_t byte) {
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    if ((byte << i) & 0x80)
      DIGITAL_WRITE_HIGH(SSD1306_SDA);
    else
      DIGITAL_WRITE_LOW(SSD1306_SDA);

    DIGITAL_WRITE_HIGH(SSD1306_SCL);
    DIGITAL_WRITE_LOW(SSD1306_SCL);
  }
  DIGITAL_WRITE_HIGH(SSD1306_SDA);
  DIGITAL_WRITE_HIGH(SSD1306_SCL);
  DIGITAL_WRITE_LOW(SSD1306_SCL);
}

void ssd1306_send_command(uint8_t command) {
  ssd1306_xfer_start();
  ssd1306_send_byte(SSD1306_SA);  // Slave address, SA0=0
  ssd1306_send_byte(0x00);	// write command
  ssd1306_send_byte(command);
  ssd1306_xfer_stop();
}

void ssd1306_send_data_start(void) {
  ssd1306_xfer_start();
  ssd1306_send_byte(SSD1306_SA);
  ssd1306_send_byte(0x40);	//write data
}

void ssd1306_send_data_stop(void) {
  ssd1306_xfer_stop();
}

void ssd1306_setpos(uint8_t x, uint8_t y)
{
  ssd1306_xfer_start();
  ssd1306_send_byte(SSD1306_SA);  //Slave address,SA0=0
  ssd1306_send_byte(0x00);	//write command

  ssd1306_send_byte(0xb0 + y);
  ssd1306_send_byte(((x & 0xf0) >> 4) | 0x10); // |0x10
  ssd1306_send_byte((x & 0x0f) | 0x01); // |0x01

  ssd1306_xfer_stop();
}

void ssd1306_fillscreen(uint8_t fill_Data) {
  uint8_t m, n;
  for (m = 0; m < 4; m++)
  {
    ssd1306_send_command(0xb0 + m);	//page0-page1
    ssd1306_send_command(0x00);		//low column start address
    ssd1306_send_command(0x10);		//high column start address
    ssd1306_send_data_start();
    for (n = 0; n < 128; n++)
    {
      ssd1306_send_byte(fill_Data);
    }
    ssd1306_send_data_stop();
  }
}


void ssd1306_char_f6x8(uint8_t x, uint8_t y, const char ch[]) {
  uint8_t c, i, j = 0;
  while (ch[j] != '\0')
  {
    c = ch[j] - 32;
    if (x > 126)
    {
      x = 0;
      y++;
    }
    ssd1306_setpos(x, y);
    ssd1306_send_data_start();
    for (i = 0; i < 6; i++)
    {
      ssd1306_send_byte(pgm_read_byte(&ssd1306xled_font6x8[c * 6 + i]));
    }
    ssd1306_send_data_stop();
    x += 6;
    j++;
  }
}

void ssd1306_char_f16x24(uint8_t tx, uint8_t ty, const char ch[]) {
  int c, i, j = 0;
  while (ch[j] != '\0')
  {
    c = ch[j] - 47;

    uint16_t jj = c * 48;
    uint16_t y, x;
    uint16_t y1 = ty + 3;
    uint16_t x1 = tx + 16;

    for (y = ty; y < y1; y++)
    {
      ssd1306_setpos(tx, y);
      ssd1306_send_data_start();
      for (x = tx; x < x1; x++)
      {
        if (c >= 0) {
          ssd1306_send_byte(pgm_read_byte(&ssd1306xled_font16x24[jj++]));
        } else {
          ssd1306_send_byte(0x00);
        }
      }
      ssd1306_send_data_stop();
    }

    tx += 16;
    j++;
  }
}

void draw_bitmap(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t bitmap[])
{
  uint16_t j = 0;
  uint8_t y, x;
  // if (y1 % 8 == 0) y = y1 / 8;   // else y = y1 / 8 + 1;   // tBUG :: this does nothing as y is initialized below
  //  THIS PARAM rule on y makes any adjustment here WRONG   //usage oled.bitmap(START X IN PIXELS, START Y IN ROWS OF 8 PIXELS, END X IN PIXELS, END Y IN ROWS OF 8 PIXELS, IMAGE ARRAY);
  for (y = y0; y < y1; y++)
  {
    ssd1306_setpos(x0, y);
    ssd1306_send_data_start();
    for (x = x0; x < x1; x++)
    {
      ssd1306_send_byte(pgm_read_byte(&bitmap[j++]));
    }
    ssd1306_send_data_stop();
  }
}
