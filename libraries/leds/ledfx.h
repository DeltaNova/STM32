// ledfx.h - WS2812 Effects Library 


void RGBLoop(uint8_t (&array)[NUM_LEDS][3]);
void Sparkle(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint8_t SpeedDelay);
void RunningLights(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3],  uint16_t WaveDelay);

void SnowSparkle(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint16_t SparkleDelay, uint16_t SpeedDelay);   // Updated
void colorWipe(uint8_t R, uint8_t G, uint8_t B, uint8_t (&array)[NUM_LEDS][3], uint16_t SpeedDelay);                            // Updated

void CylonBounce(uint8_t R, uint8_t G, uint8_t B, int EyeSize, int SpeedDelay, int ReturnDelay);
void Strobe(uint8_t R, uint8_t G, uint8_t B, uint8_t StrobeCount, uint16_t FlashDelay, uint16_t EndPause);
void Twinkle(uint8_t R, uint8_t G, uint8_t B, uint8_t Count, uint8_t SpeedDelay, bool OnlyOne);
void TwinkleRandom(uint8_t Count, uint8_t SpeedDelay, bool OnlyOne);
void theaterChase(uint8_t R, uint8_t G, uint8_t B, uint8_t SpeedDelay);

void meteorRain(uint8_t R, uint8_t G, uint8_t B, uint8_t meteorSize, uint8_t meteorTrailDecay, bool meteorRandomDecay, int SpeedDelay);
void fadeToBlack(int ledNo, uint8_t fadeValue);

void theaterChaseRainbow(int SpeedDelay);
uint8_t* Wheel(uint8_t WheelPos);

void setPixelHeatColor (uint8_t Pixel, uint8_t temperature);
void Fire(int Cooling, int Sparking, int SpeedDelay);
