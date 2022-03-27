#include "display/ssd1306.h"
#include "sprite/dict8.h"
#include "capsens/circular_cap.h"
#include "pico/stdlib.h"

SSD1306_128x64 display;
Dict8 dict8(&display);
CircCapSens capsens({6, 7, 2, 1, 0, 25, 10, 11, 12, 13, 14, 15}, 1000, 6);

int main(){
    display.init(5, 4, 0);
    capsens.init();
    while(1){
        capsens.read();
        display.clear();
        // std::vector<int> values = capsens.rawVal();
        // for(int i=0; i<values.size(); i++){
        //     dict8.print((i%4)*32, (i/4)*10, "%d", values[i]);
        // }
        std::vector<bool> values = capsens.pressed();
        for(int i=0; i<values.size(); i++){
            dict8.print(i*6, 10, "%c", values[i]?'X':'.');
        }
        dict8.print(10, 20, "%c", capsens.isPressed()?'X':'.');
        dict8.print(30, 20, "%c", capsens.isHeld()?'X':'.');
        dict8.print(50, 20, "%c", capsens.isReleased()?'X':'.');

        display.display();
        sleep_ms(10);
    }
}