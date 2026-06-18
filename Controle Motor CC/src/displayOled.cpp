#include "display.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

#define FILTRO_VEL_DISPLAY 0.5f

float spAbs;
float vel_filt = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const unsigned char dir_h [] PROGMEM = {
  0b00000000, 0b00000000, 0b00011100, 
  0b00010000, 0b00011000, 0b00011100,
  0b00111000, 0b00111100, 0b00011100, 
  0b01111100, 0b01100110, 0b01111111, 
  0b11111110, 0b01100110, 0b00111110, 
  0b00111000, 0b00111100, 0b00011100, 
  0b00111000, 0b00011000, 0b00001000,
  0b00111000, 0b00000000, 0b00000000
};



const unsigned char dir_ah [] PROGMEM = {
  0b00111000, 0b00000000, 0b00000000, 
  0b00111000, 0b00011000, 0b000011000,
  0b00111000, 0b00111100, 0b00011100, 
  0b11111110, 0b01100110, 0b00111110, 
  0b01111100, 0b01100110, 0b01111111, 
  0b00111000, 0b00111100, 0b00011100, 
  0b00010000, 0b00011000, 0b00011100,
  0b00000000, 0b00000000, 0b00011100
};

void initDisplay() {
    Wire.begin(21, 22); // SDA=21, SCL=22
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for(;;); // trava se não encontrar
    }
    display.clearDisplay();
    display.setTextSize(1);             
    display.setTextColor(SSD1306_WHITE);
    display.display();
}

void showMessage(const char* msg) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(msg);
    display.display();
}

void updateDisplay(float vel, float corrente, float tensao, float setpoint) {

    vel_filt = vel*(1-FILTRO_VEL_DISPLAY) + vel_filt*FILTRO_VEL_DISPLAY;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);

    display.print("   CONTROLE DIGITAL   ");
    display.print("     UFC SOBRAL     ");

    display.setCursor(0,18);
    display.print("Velocidade: ");
    display.print(vel_filt,0);
    display.println(" RPM");

    display.setCursor(0,27);
    display.print("  Corrente: ");
    display.print(corrente,2);
    display.println(" A");

    display.setCursor(0,36);
    display.print("    Tensao: ");
    display.print(tensao,2);
    display.println(" V");

    display.setCursor(0,45);
    display.print("      Pot.: ");
    display.print(tensao*corrente,2);
    display.println(" W");

    display.setCursor(0,57);
    display.print("Set: ");
    spAbs = (setpoint >= 0.0f) ? setpoint : -setpoint;
    display.print(spAbs,0);
    display.print(" RPM ");

    int posX = 90; 
    int posY = 56; 
    if (setpoint == 0.0f) {
    } else if (setpoint < 0.0f) {
        // sentido horário
        display.drawBitmap(posX, posY, dir_h, 24, 8, SSD1306_WHITE);
    } else {
        // sentido anti-horário
        display.drawBitmap(posX, posY, dir_ah, 24, 8, SSD1306_WHITE);
    }

    display.display();
}
