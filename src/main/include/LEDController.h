#pragma once
#include <frc/AddressableLED.h>
#include <frc/Timer.h>
#include "enums.h"
BETTER_ENUM(LED_MODE, int, Red, Rainbow, SolidBlue, SolidYellow, Dot, Ratpack, None)
class LEDController {
    public:
        LED_MODE oldMode = LED_MODE::None;
        int firstPixelHue = 0;
        bool modeChange = false;
        frc::Timer timer;
        frc::AddressableLED ledInterface;
        std::vector<frc::AddressableLED::LEDData> ledStrip;
        int length;
        LEDController(int len, int pin) : ledInterface(pin), ledStrip(len), length(len) {
            ledInterface.SetLength(len);
            ledInterface.SetData(ledStrip);
            ledInterface.Start();
            timer.Start();
        }
        //Affected leds
    void Set(int index) {
        // https://stackoverflow.com/questions/11452920/how-to-cast-int-to-enum-in-c
        if (index < 0) { 
            std::cout << "WARNING: Set index < 0; using 0.\n";
            index = 0;
        }
        _set(LED_MODE::_from_integral(index));
    }
    int NumModes() {
        return LED_MODE::_size();
    }
    void Set(LED_MODE mode) {
        _set(mode);
    }
    std::string Get() {
        // https://stackoverflow.com/questions/1195675/convert-a-char-to-stdstring
        const char *s = oldMode._to_string();
        return std::string(s);
    }
    void _set(LED_MODE mode) {
        modeChange = (mode != oldMode);
        oldMode = mode;
        switch (mode) {
            case LED_MODE::Red:
                _set_all(255, 0, 0);
                break;
            case LED_MODE::Rainbow:
                rainbow();
                break;
            case LED_MODE::SolidBlue:
                ratblue();
                break;
            case LED_MODE::SolidYellow:
                ratyellow();
                break;
            case LED_MODE::Ratpack:
                ratpack(.5);
                break;
            case LED_MODE::Dot:
                dot(1.0);
                break;
            case LED_MODE::None:
                _set_all(0,0,0);
                break;
            default:
                return;
        }
        ledInterface.SetData(ledStrip);
    } 
    void rainbow() {
        // For every pixel
        auto pixelHue = (firstPixelHue + (0 * 180 / length)) % 180;
        for (int i = 0; i < length; i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to process
            pixelHue = (firstPixelHue + (i * 180 / length)) % 180;
            // Set the value
            ledStrip[i].SetHSV(pixelHue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        firstPixelHue += 3;
        // Check bounds
        firstPixelHue %= 180;
    }
    double getTime() {
        return timer.Get();
    }
    void ratpack(double interval) {

        if (std::fmod(timer.Get(), interval * 2.0) < interval) {
            ratblue();
        } else {
            ratyellow();
        }
        // (std::fmod(timer.Get(), interval * 2.0) < interval) ? ratblue() : ratyellow();
    }
    void ratblue() {
        _set_all(58,55,171);
    }
    void ratyellow() {
        _set_all(251,215,4);
    }
    void dot(double animationTime) {
        if (modeChange) { // clear all
            _set_all(0,0,0);
        }
        double timerMod = std::fmod(timer.Get(), animationTime);
        // we want to convert time [0,.5] into nth pixel [0,40]
        // 2 * [0, .5] * length
        int i = (int) (1.0 / animationTime * timerMod * length);
        ledStrip[i].SetRGB(255, 0, 0);
        // set previous pixel back
        if (i >= 1) {
            ledStrip[i - 1].SetRGB(0,0,0);
        } else if (i == 0) {
            ledStrip[length - 1].SetRGB(0,0,0);
        }
    }
    void _set_all(int r, int g, int b) {
        for (int i = 0; i< length; i++) {
            ledStrip[i].SetRGB(r,g,b);
        }
    }
};