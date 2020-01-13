#include <frc/AddressableLED.h>
enum LEDModes{Red, Rainbow, COUNT};
class LEDController {
    public:
        int length;
        frc::AddressableLED ledInterface;
        //Affected leds
        std::vector<frc::AddressableLED::LEDData> ledStrip;
    LEDController(int len, int pin): ledInterface(pin), ledStrip(len), length(len) {
        ledInterface.SetLength(len);
        ledInterface.SetData(ledStrip);
        ledInterface.Start();
    }
    void Set(int index) {
        // https://stackoverflow.com/questions/11452920/how-to-cast-int-to-enum-in-c
        _set(static_cast<LEDModes>(index));
    }
    void Set(LEDModes mode) {
        _set(mode);
    }
    void _set(LEDModes mode) {
        switch (mode) {

            case Red:
                red();
                break;
            case Rainbow:
                rainbow();
                break;
            default:
                return;
        }
        ledInterface.SetData(ledStrip);
    }
    void rainbow() {
        // For every pixel
        static int firstPixelHue = 0;
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
    void red(){
        for (int i = 0; i < length; i++){
            ledStrip[i].SetRGB(255,100,100);
        }
    }
};