#include "display_helper.h"

void displayInfoRing(CircCapSens& sensor, Sprite& sprite, int startOff){
    bool invalid = sensor.isInvalid();
    std::vector<bool> pressed = sensor.pressed();
    const int accPer[4] = {
        sprite.width()-1,
        sprite.width()+sprite.height()-2,
        (2*sprite.width())+(sprite.height())-3,
        (2*sprite.width())+(2*sprite.height())-4
    };
    const int acc = 1+accPer[3]/pressed.size();
    for(int i=0; i<pressed.size(); i++){
        Sprite::Color c = pressed[i]?Sprite::WHITE:Sprite::BLACK;
        int offset = (startOff+(i*accPer[3])/pressed.size())%accPer[3];
        int end = offset+acc;
        if(offset<accPer[0]){
            if(end<accPer[0]){
                sprite.horzLine(offset, end, 0, c);
                continue;
            } else {
                sprite.horzLine(offset, sprite.width()-1, 0, c);
                offset = accPer[0];
            }
        }
        if(offset<accPer[1]){
            if(end<accPer[1]){
                sprite.vertLine(sprite.width()-1, offset-accPer[0], end-accPer[0], c);
                continue;
            } else {
                sprite.vertLine(sprite.width()-1, offset-accPer[0], sprite.height()-1, c);
                offset = accPer[1];
            }
        }
        if(offset<accPer[2]){
            if(end<accPer[2]){
                sprite.horzLine(sprite.width()-(end+1-accPer[1]), sprite.width()-(offset+1-accPer[1]), sprite.height()-1, c);
                continue;
            } else {
                sprite.horzLine(0, sprite.width()-(offset+1-accPer[1]), sprite.height()-1, c);
                offset = accPer[2];
            }
        }
        if(end<accPer[3]){
            sprite.vertLine(0, sprite.height()-(end+1-accPer[2]), sprite.height()-(offset+1-accPer[2]), c);
        } else {
            sprite.vertLine(0, 0, sprite.height()-(offset+1-accPer[2]), c);
            end -= accPer[3];
            sprite.horzLine(0, end, 0, c);
        }
    }
}