# ESP32 Anime Eyes Project

An interactive anime-style eyes display project for the **ESP32-C6-LCD-1.47** development board using LVGL graphics library.

## Features

- **Animated Anime Eyes**: Realistic eye animations with blinking, pupil movement, and emotional expressions
- **Multiple Emotions**: 6 different eye emotions (Normal, Happy, Surprised, Sleepy, Angry, Sad)
- **Smooth Animations**: Fluid eye movement and blinking with realistic timing
- **Touch Interaction**: Touch-responsive eyes that follow your finger (for touch-enabled boards)
- **Auto Behavior**: Automatic random eye movements and periodic blinking
- **Brightness Control**: Adjustable display brightness

## Hardware Requirements

- **ESP32-C6-LCD-1.47** development board from Waveshare
  - 1.47" IPS LCD display (172×320 resolution)
  - ST7789 display controller
  - Optional: Capacitive touch panel (AXS5106L controller)

## Pin Configuration

### Display (ST7789)
- MOSI: GPIO 7
- SCLK: GPIO 6
- CS: GPIO 5
- DC: GPIO 4
- RST: GPIO 8
- Backlight: GPIO 9

### Touch (Optional - AXS5106L)
- SDA: GPIO 11
- SCL: GPIO 10
- I2C Address: 0x3C

## Project Structure

```
├── components/
│   └── lvgl/                 # LVGL graphics library
├── main/
│   ├── include/
│   │   ├── anime_eyes.h      # Eye animation definitions
│   │   ├── st7789_display.h  # Display driver header
│   │   └── touch_input.h     # Touch input header (optional)
│   ├── anime_eyes.c          # Eye animation implementation
│   ├── st7789_display.c      # ST7789 display driver
│   ├── touch_input.c         # Touch input handling (optional)
│   ├── main.c                # Main application
│   └── CMakeLists.txt
├── CMakeLists.txt
└── README.md
```

## Building and Flashing

1. **Setup ESP-IDF environment** (v5.0 or later)
2. **Clone and build**:
   ```bash
   git clone <repository-url>
   cd ESP32-anime-eyes
   idf.py build
   ```
3. **Flash to device**:
   ```bash
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

## Eye Behaviors

### Automatic Behaviors
- **Periodic Blinking**: Eyes blink naturally every ~3 seconds
- **Random Movement**: Eyes look around randomly when idle
- **Emotion Cycling**: Automatically cycles through different emotions every 5 seconds

### Touch Interactions (Touch Version)
- **Eye Following**: Eyes track finger movement on screen
- **Emotion Zones**:
  - Top area: Surprised expression
  - Bottom area: Sleepy expression  
  - Left area: Happy expression
  - Right area: Angry expression
  - Center area: Normal expression
- **Touch Blink**: Eyes blink when first touched

### Available Emotions
1. **Normal**: Standard blue eyes with neutral expression
2. **Happy**: Slight smile-shaped eyes with bright blue iris
3. **Surprised**: Wide open eyes with green iris
4. **Sleepy**: Half-closed eyes with standard iris
5. **Angry**: Normal eyes with red iris
6. **Sad**: Standard eyes with dark blue iris

## Customization

### Display Settings
- Adjust brightness: `display_set_brightness(0-255)`
- Modify display refresh rate in `lv_conf.h`
- Change color depth and memory allocation in LVGL config

### Eye Parameters
- Eye size: Modify `EYE_WIDTH` and `EYE_HEIGHT` in `anime_eyes.h`
- Pupil/iris size: Adjust `PUPIL_RADIUS` and `IRIS_RADIUS`
- Movement range: Change `LOOK_RANGE` for eye movement limits
- Animation speed: Modify `MOVEMENT_SMOOTH_FACTOR`

### Timing Settings
- Blink frequency: Adjust `BLINK_INTERVAL_MS`
- Blink duration: Modify `BLINK_DURATION_MS`
- Random movement interval: Change timeout in `anime_eyes_update()`

## API Reference

### Display Functions
```c
void st7789_init(void);                                    // Initialize display
void display_set_brightness(uint8_t brightness);          // Set backlight brightness
```

### Eye Animation Functions
```c
void anime_eyes_init(void);                               // Initialize eye system
void anime_eyes_update(void);                             // Update eye animations
void anime_eyes_set_emotion(eye_emotion_t emotion);       // Set eye emotion
void anime_eyes_look_at(float x, float y);               // Look at position (-1.0 to 1.0)
void anime_eyes_start_blink(void);                        // Trigger blink
void anime_eyes_set_sleepy_mode(bool enable);             // Enable/disable sleepy mode
```

### Touch Functions (Optional)
```c
void touch_init(void);                                    // Initialize touch controller
bool touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data); // Read touch data
```

## Technical Details

- **Graphics Library**: LVGL v8.3.11
- **Display Driver**: Custom ST7789 SPI driver
- **Memory Usage**: ~32KB LVGL buffer + application memory
- **Performance**: 30 FPS eye updates, 33 FPS display refresh
- **Color Depth**: 16-bit RGB565

## Troubleshooting

### Display Issues
- Check SPI connections and pin definitions
- Verify display initialization sequence
- Ensure adequate power supply

### Performance Issues
- Reduce LVGL buffer size if memory is limited
- Lower display refresh rate
- Optimize drawing operations

### Touch Issues (Touch Version)
- Verify I2C connections and pull-up resistors
- Check touch controller I2C address
- Calibrate touch coordinates if needed

## License

This project is open source. See LICENSE file for details.