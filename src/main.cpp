#include "display/ssd1306.h"
#include "sprite/dict8.h"
#include "capsens/circular_cap.h"
#include "capsens/display_helper.h"
#include "pico/stdlib.h"
#include "pico/time.h"

SSD1306_128x64 display;
Dict8 dict8(&display);
CircCapSens capsens({15, 14, 13, 12, 11, 10, 25, 0, 1, 2, 7, 6}, 5000, 10);

int main(){
    display.init(5, 4, 0);
    capsens.init();
    while(1){
        uint64_t begin, end;
        display.clear();
        begin=time_us_64();
        capsens.read();
        end=time_us_64();
        std::vector<int> valuesi = capsens.rawVal();
        for(int i=0; i<valuesi.size(); i++){
            dict8.print((i%4)*32, (i/4)*10+20, "%d", valuesi[i]);
        }
        std::vector<bool> values = capsens.pressed();
        for(int i=0; i<values.size(); i++){
            dict8.print(i*6, 0, "%c", values[i]?'X':'.');
        }
        dict8.print(10, 10, "%c", capsens.isPressed()?'X':'.');
        dict8.print(20, 10, "%c", capsens.isHeld()?'X':'.');
        dict8.print(30, 10, "%c", capsens.isReleased()?'X':'.');
        dict8.print(40, 10, "%c", capsens.wasClicked()?'X':'.');
        dict8.print(50, 10, "%c", capsens.isInvalid()?'X':'.');
        dict8.print(70, 10, "%d", capsens.lastDelta());
        dict8.print(90, 10, "%d", capsens.lastRead());

        dict8.print(50, 56, "%dus", (int)(end-begin));

        displayInfoRing(capsens, display, 254);

        display.display();
        sleep_ms(10);
    }
}