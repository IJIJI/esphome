#pragma once

#include "esphome/core/component.h"
// #include "esphome/components/spi/spi.h"
#include "esphome/components/display/display_buffer.h"

namespace esphome {
namespace hub75 {

typedef uint32_t PortType; // Formerly 'RwReg' but interfered w/CMCIS header

class HUB75 : public display::DisplayBuffer{
 public:
   /*!
    @brief  Constructor for 32x32 or 32x64 panel.
    @param  a        Address/row-select A pin number.
    @param  b        Address/row-select B pin number.
    @param  c        Address/row-select C pin number.
    @param  d        Address/row-select D pin number.
    @param  clk      RGB clock pin number.
    @param  lat      RGB latch pin number.
    @param  oe       Output enable pin number.
    @param  width    Specify 32 or 64 for the two supported matrix widths
                     (default is 32).
    @param  pinlist  uint8_t array of 6 pin numbers corresponding
                     to upper R, G, B and lower R, G, B pins.
    @param  dbuf     If true, display is double-buffered, allowing for
                     smoother animation (requires 2X RAM).
  */
  HUB75(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t clk,
                uint8_t lat, uint8_t oe, uint8_t width = 64,
                uint8_t *pinlist = NULL, boolean dbuf = false
  );

  /*!
    @brief  Start RGB matrix. Initializes timers and interrupts.
  */
  void begin() override;

  /*!
    @brief  Set the brightness of the display.
    @param  brightness Brightness level, from 0 (off) to 255 (max brightness).
  */
  void set_brightness(uint8_t brightness) { 
    this->brightness_ = brightness; 
    this->update();
  }

  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void update() override { this->updateDisplay();};

  void dump_config() override;


  void fill(Color color) override;

  /*!
    @brief   Decimate 8-bits R,G,B (used in a lot of existing graphics code
             in other projects and languages) to the '565' color format used
             in Adafruit_GFX.
    @param   r  Red value, 0-255.
    @param   g  Green value, 0-255.
    @param   b  Blue value, 0-255.
    @return  16-bit '565' color as used by Adafruit_GFX, can then be passed
             to drawing functions. Actual colors issued to matrix will be
             further decimated from this, since it uses fewer bitplanes.
  */
  uint16_t Color888(uint8_t r, uint8_t g, uint8_t b);

  display::DisplayType get_display_type() override { return display::DisplayType::DISPLAY_TYPE_BINARY; }

 protected:
  IRAM_ATTR void updateDisplay();
  
  void draw_absolute_pixel_internal(int x, int y, Color color) override;

  size_t get_buffer_length_();

  // void start_command_();
  // void end_command_();
  // void start_data_();
  // void end_data_();

  int get_width_internal() override;
  int get_height_internal() override;

  uint8_t brightness_;
  uint8_t width_;

  // TODO USE DEAFULT SPI 
  GPIOPin *clock_pin_;
  GPIOPin *latch_pin_;
  GPIOPin *oe_pin_;
  
  // TODO SUPPORT HIGHER RES SCREENS
  // GPIOPin *address_pins_[4];
  // GPIOPin *rgb_pins_[6];

  GPIOPin *address_a_pin_;
  GPIOPin *address_b_pin_;
  GPIOPin *address_c_pin_;
  GPIOPin *address_d_pin_;

  GPIOPin *rgb_r1_pin_;
  GPIOPin *rgb_g1_pin_;
  GPIOPin *rgb_b1_pin_;
  GPIOPin *rgb_r2_pin_;
  GPIOPin *rgb_g2_pin_;
  GPIOPin *rgb_b2_pin_;

  private:
    uint8_t *matrixbuff[2];     ///< Buffer pointers for double-buffering
    uint8_t nRows;              ///< Number of rows (derived from A/B/C/D pins)
    volatile uint8_t backindex; ///< Index (0-1) of back buffer
    volatile boolean swapflag;  ///< if true, swap on next vsync

    // Init/alloc code common to both constructors:
    void init(uint8_t rows, uint8_t a, uint8_t b, uint8_t c, uint8_t clk,
              uint8_t lat, uint8_t oe, boolean dbuf, uint8_t width,
              uint8_t *rgbpins
    );

    RAM_ATTR void IRQ_HANDLER(void *arg);

    uint8_t _clk;       ///< RGB clock pin number
    uint8_t _lat;       ///< RGB latch pin number
    uint8_t _oe;        ///< Output enable pin number
    uint8_t _a;         ///< Address/row-select A pin number
    uint8_t _b;         ///< Address/row-select B pin number
    uint8_t _c;         ///< Address/row-select C pin number
    uint8_t _d;         ///< Address/row-select D pin number
    PortType clkmask;   ///< RGB clock pin bitmask
    PortType latmask;   ///< RGB latch pin bitmask
    PortType oemask;    ///< Output enable pin bitmask
    PortType addramask; ///< Address/row-select A pin bitmask
    PortType addrbmask; ///< Address/row-select B pin bitmask
    PortType addrcmask; ///< Address/row-select C pin bitmask
    PortType addrdmask; ///< Address/row-select D pin bitmask
    // PORT register pointers (CLKPORT is hardcoded on AVR)
    volatile PortType *latport;   ///< RGB latch PORT register
    volatile PortType *oeport;    ///< Output enable PORT register
    volatile PortType *addraport; ///< Address/row-select A PORT register
    volatile PortType *addrbport; ///< Address/row-select B PORT register
    volatile PortType *addrcport; ///< Address/row-select C PORT register
    volatile PortType *addrdport; ///< Address/row-select D PORT register

    // CHECK IF PINS ARE DOUBLED OR NOT
    uint8_t rgbpins[6];           ///< Pin numbers for 2x R,G,B bits
    volatile PortType *outsetreg; ///< RGB PORT bit set register
    volatile PortType *outclrreg; ///< RGB PORT bit clear register
    PortType rgbclkmask;          ///< Mask of all RGB bits + CLK
    PortType expand[256];         ///< 6-to-32 bit converter table

    volatile uint8_t row;      ///< Row counter for interrupt handler
    volatile uint8_t plane;    ///< Bitplane counter for interrupt handler
    volatile uint8_t *buffptr; ///< Current RGB pointer for interrupt handler
};

}  // namespace pcd8544
}  // namespace esphome
