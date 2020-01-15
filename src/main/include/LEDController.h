#include <frc/AddressableLED.h>
#include <frc/Timer.h>
enum LEDModes{Red, Green, Rainbow, SolidBlue, SolidYellow, Dot, Ratpack, None, COUNT};
class LEDController {
    public:
        int length;
        frc::AddressableLED ledInterface;
        //Affected leds
        std::vector<frc::AddressableLED::LEDData> ledStrip;
        frc::Timer timer;
        int firstPixelHue = 0;
        bool modeChange = false;
        LEDModes oldMode = LEDModes::None;
        LEDController(int len, int pin) : ledInterface(pin), ledStrip(len), length(len) {
            ledInterface.SetLength(len);
            ledInterface.SetData(ledStrip);
            ledInterface.Start();
            timer.Start();
        }
    void Set(int index) {
        // https://stackoverflow.com/questions/11452920/how-to-cast-int-to-enum-in-c
        _set(static_cast<LEDModes>(index));
    }
    void Set(LEDModes mode) {
        _set(mode);
    }
    void _set(LEDModes mode) {
        modeChange = (mode != oldMode);
        oldMode = mode;
        switch (mode) {
            case Red:
                _set_all(255, 0, 0);
                break;
            case Green:
                _set_all(0, 255, 0);
                break;
            case Rainbow:
                rainbow();
                break;
            case SolidBlue:
                ratblue();
                break;
            case SolidYellow:
                ratyellow();
                break;
            case Ratpack:
                ratpack(.2);
                break;
            case Dot:
                dot(1.0);
                break;
            case None:
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
    void ratpack(double interval) {
        (std::fmod(timer.Get(), interval * 2.0) < interval) ? ratblue() : ratyellow();
    }
    void red(){
    }
    void green() {
    }
    void ratblue() {
        _set_all(58,55,171);
    }
    void ratyellow() {
        _set_all(251,215,4);
    }
    void dot(double animationTime) {
        if (modeChange) {
            _set_all(0,0,0);
        }
        double timerMod = std::fmod(timer.Get(), animationTime);
        // we want to convert time [0,.5] into nth pixel [0,40]. multiply by pixels e.g. [0, 40], cast to int
        int i = (int) (1.0 / timerMod) * length;
        ledStrip[i].SetRGB(255, 0, 0);
        // set previous pixel back
        if (i >= 1) {
            ledStrip[i - 1].SetRGB(0,0,0);
        }
    }
    void _set_all(int r, int g, int b) {
        for (int i = 0; i< length; i++) {
            ledStrip[i].SetRGB(r,g,b);
        }
    }
};