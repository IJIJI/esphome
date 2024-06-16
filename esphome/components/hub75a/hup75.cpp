#include "hub75.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace hub75 {

static const char *const TAG = "pcd_8544";

// Constructor for 32 high matrix
HUB75::HUB75(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t clk,
                uint8_t lat, uint8_t oe, uint8_t width = 64,
                uint8_t *pinlist = NULL, boolean dbuf = false){

  init(16, a, b, c, clk, lat, oe, dbuf, width,
       pinlist
  );
}

IRAM_ATTR void IRQ_HANDLER(void *);

void HUB75::begin() {
  backindex = 0;                       // Back buffer
  buffptr = matrixbuff[1 - backindex]; // -> front buffer
  activePanel = this;                  // For interrupt hander

  // Enable all comm & address pins as outputs, set default states:
  pinMode(_clk, OUTPUT);
  digitalWrite(_clk, LOW); // Low
  pinMode(_lat, OUTPUT);
  *latport &= ~latmask; // Low
  pinMode(_oe, OUTPUT);
  *oeport |= oemask; // High (disable output)
  pinMode(_a, OUTPUT);
  *addraport &= ~addramask; // Low
  pinMode(_b, OUTPUT);
  *addrbport &= ~addrbmask; // Low
  pinMode(_c, OUTPUT);
  *addrcport &= ~addrcmask; // Low
  if (nRows > 8) {
    pinMode(_d, OUTPUT);
    *addrdport &= ~addrdmask; // Low
  }

  // Semi-configurable RGB bits; must be on same PORT as CLK
  if (_clk < 32) {
    outsetreg = &GPIO.out_w1ts;
    outclrreg = &GPIO.out_w1tc;
  } else {
    outsetreg = (volatile PortType *)&(GPIO.out1_w1ts);
    outclrreg = (volatile PortType *)&(GPIO.out1_w1tc);
  }

  PortType rgbmask[6];
  clkmask = rgbclkmask = digitalPinToBitMask(_clk);
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(rgbpins[i], OUTPUT);
    rgbmask[i] = digitalPinToBitMask(rgbpins[i]); // Pin bit mask
    rgbclkmask |= rgbmask[i];                     // Add to RGB+CLK bit mask
  }
  for (int i = 0; i < 256; i++) {
    expand[i] = 0;
    if (i & 0x04)
      expand[i] |= rgbmask[0];
    if (i & 0x08)
      expand[i] |= rgbmask[1];
    if (i & 0x10)
      expand[i] |= rgbmask[2];
    if (i & 0x20)
      expand[i] |= rgbmask[3];
    if (i & 0x40)
      expand[i] |= rgbmask[4];
    if (i & 0x80)
      expand[i] |= rgbmask[5];
  }

  timer_config_t tim_config;
  tim_config.divider = 2; // Run Timer at 40 MHz
  tim_config.counter_dir = TIMER_COUNT_UP;
  tim_config.counter_en = TIMER_PAUSE;
  tim_config.alarm_en = TIMER_ALARM_EN;
  tim_config.auto_reload = TIMER_AUTORELOAD_EN;
  tim_config.intr_type = TIMER_INTR_LEVEL;

  timer_init(TIMER_GROUP_1, TIMER_0, &tim_config);
  /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on
     alarm */
  timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0x00000000ULL);
  /* Configure the alarm value and the interrupt on alarm. */
  timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 10000);
  timer_enable_intr(TIMER_GROUP_1, TIMER_0);
  timer_isr_register(TIMER_GROUP_1, TIMER_0, IRQ_HANDLER, (void *)TIMER_0,
                     ESP_INTR_FLAG_IRAM, NULL);

  timer_start(TIMER_GROUP_1, TIMER_0);
}



void HUB75::updateDisplay() {

  uint8_t i, tick, tock, *ptr;
  uint16_t t, duration;

  *oeport |= oemask;   // Disable LED output during row/plane switchover
  *latport |= latmask; // Latch data loaded during *prior* interrupt

  // Calculate time to next interrupt BEFORE incrementing plane #.
  // This is because duration is the display time for the data loaded
  // on the PRIOR interrupt.  CALLOVERHEAD is subtracted from the
  // result because that time is implicit between the timer overflow
  // (interrupt triggered) and the initial LEDs-off line at the start
  // of this method.
  t = (nRows > 8) ? LOOPTIME : (LOOPTIME * 2);
  duration = ((t + CALLOVERHEAD * 2) << plane) - CALLOVERHEAD;

  // Borrowing a technique here from Ray's Logic:
  // www.rayslogic.com/propeller/Programming/AdafruitRGB/AdafruitRGB.htm
  // This code cycles through all four planes for each scanline before
  // advancing to the next line.  While it might seem beneficial to
  // advance lines every time and interleave the planes to reduce
  // vertical scanning artifacts, in practice with this panel it causes
  // a green 'ghosting' effect on black pixels, a much worse artifact.

  if (++plane >= nPlanes) {   // Advance plane counter.  Maxed out?
    plane = 0;                // Yes, reset to plane 0, and
    if (++row >= nRows) {     // advance row counter.  Maxed out?
      row = 0;                // Yes, reset row counter, then...
      if (swapflag == true) { // Swap front/back buffers if requested
        backindex = 1 - backindex;
        swapflag = false;
      }
      buffptr = matrixbuff[1 - backindex]; // Reset into front buffer
    }
  } else if (plane == 1) {
    // Plane 0 was loaded on prior interrupt invocation and is about to
    // latch now, so update the row address lines before we do that:
    if (row & 0x1)
      *addraport |= addramask;
    else
      *addraport &= ~addramask;
    // MYSTERY: certain matrices REQUIRE these delays ???
    delayMicroseconds(10);
    if (row & 0x2)
      *addrbport |= addrbmask;
    else
      *addrbport &= ~addrbmask;
    delayMicroseconds(10);
    if (row & 0x4)
      *addrcport |= addrcmask;
    else
      *addrcport &= ~addrcmask;
    delayMicroseconds(10);
    if (nRows > 8) {
      if (row & 0x8)
        *addrdport |= addrdmask;
      else
        *addrdport &= ~addrdmask;
      delayMicroseconds(10);
    }
  }

  // buffptr, being 'volatile' type, doesn't take well to optimization.
  // A local register copy can speed some things up:
  ptr = (uint8_t *)buffptr;


static timg_dev_t *TG[2] = {&TIMERG0, &TIMERG1};
static portMUX_TYPE timer_spinlock[TIMER_GROUP_MAX] = {
    portMUX_INITIALIZER_UNLOCKED, portMUX_INITIALIZER_UNLOCKED};
portENTER_CRITICAL(&timer_spinlock[TIMER_GROUP_1]);
TG[TIMER_GROUP_1]->hw_timer[TIMER_0].alarm_high = 0;
TG[TIMER_GROUP_1]->hw_timer[TIMER_0].alarm_low = (uint32_t)duration;
portEXIT_CRITICAL(&timer_spinlock[TIMER_GROUP_1]);
  *oeport &= ~oemask;   // Re-enable output
  *latport &= ~latmask; // Latch down

  // Record current state of CLKPORT register, as well as a second
  // copy with the clock bit set.  This makes the innnermost data-
  // pushing loops faster, as they can just set the PORT state and
  // not have to load/modify/store bits every single time.  It's a
  // somewhat rude trick that ONLY works because the interrupt
  // handler is set ISR_BLOCK, halting any other interrupts that
  // might otherwise also be twiddling the port at the same time
  // (else this would clobber them). only needed for AVR's where you
  // cannot set one bit in a single instruction

  if (plane > 0) { // 188 ticks from TCNT1=0 (above) to end of function

    // Planes 1-3 copy bytes directly from RAM to PORT without unpacking.
    // The least 2 bits (used for plane 0 data) are presumed masked out
    // by the port direction bits.


#define pew                                                                    \
  *outclrreg = rgbclkmask;                                                     \
  *outsetreg = expand[*ptr++];                                                 \
  *outsetreg = clkmask;

    // Loop is unrolled for speed:
    pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew
        pew pew pew pew pew pew pew pew pew pew pew pew pew

        if (WIDTH == 64){
            pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew
                pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew}

            *outclrreg = clkmask; // Set clock low

    buffptr = ptr; //+= 32;

  } else { // 920 ticks from TCNT1=0 (above) to end of function

    for (int i = 0; i < WIDTH; i++) {
      byte b = (ptr[i] << 6) | ((ptr[i + WIDTH] << 4) & 0x30) |
               ((ptr[i + WIDTH * 2] << 2) & 0x0C);

      *outclrreg = rgbclkmask; // Clear all data and clock bits together
      *outsetreg = expand[b];  // Set new data bits
      *outsetreg = clkmask;    // Set clock high
    }
    *outclrreg = clkmask; // Set clock low
  }
}

void HUB75::dump_config() {
  LOG_DISPLAY("", "HUB&%", this);
  LOG_PIN("  Clock Pin: ", this->clock_pin_);
  LOG_PIN("  Latch Pin: ", this->latch_pin_);
  LOG_PIN("  OE Pin: ", this->oe_pin_);
  LOG_PIN("  Address A Pin: ", this->address_a_pin_);
  LOG_PIN("  Address B Pin: ", this->address_b_pin_);
  LOG_PIN("  Address C Pin: ", this->address_c_pin_);
  LOG_PIN("  Address D Pin: ", this->address_d_pin_);
  LOG_PIN("  R1 Pin: ", this->rgb_r1_pin_);
  LOG_PIN("  G1 Pin: ", this->rgb_g1_pin_);
  LOG_PIN("  B1 Pin: ", this->rgb_b1_pin_);
  LOG_PIN("  R2 Pin: ", this->rgb_r2_pin_);
  LOG_PIN("  G2 Pin: ", this->rgb_g2_pin_);
  LOG_PIN("  B2 Pin: ", this->rgb_b2_pin_);
  LOG_NUMBER("  Width: ", this->width_);
  LOG_NUMBER("  Brightness: ", this->brightness_);
  LOG_UPDATE_INTERVAL(this);
}



void HUB75::fill(Color color) {
  uint16_t c = color.is_on() ? 0xFFF : 0x000;
  // For black or white, all bits in frame buffer will be identically
  // set or unset (regardless of weird bit packing), so it's OK to just
  // quickly memset the whole thing:
  memset(matrixbuff[backindex], c, WIDTH * nRows * 3);
}

void HOT HUB75::display() {
  // uint8_t col, maxcol, p;

  // for (p = 0; p < 6; p++) {
  //   this->command(this->PCD8544_SETYADDR | p);

  //   // start at the beginning of the row
  //   col = 0;
  //   maxcol = this->get_width_internal() - 1;

  //   this->command(this->PCD8544_SETXADDR | col);

  //   this->start_data_();
  //   for (; col <= maxcol; col++) {
  //     this->write_byte(this->buffer_[(this->get_width_internal() * p) + col]);
  //   }
  //   this->end_data_();
  // }

  // this->command(this->PCD8544_SETYADDR);
}

void HOT HUB75::draw_absolute_pixel_internal(int x, int y, Color color) {

  c = display::ColorUtil::color_to_565(color);;

  uint8_t r, g, b, bit, limit, *ptr;

  if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))
    return;



  // Adafruit_GFX uses 16-bit color in 5/6/5 format, while matrix needs
  // 4/4/4.  Pluck out relevant bits while separating into R,G,B:
  r = c >> 12;        // RRRRrggggggbbbbb
  g = (c >> 7) & 0xF; // rrrrrGGGGggbbbbb
  b = (c >> 1) & 0xF; // rrrrrggggggBBBBb

  // Loop counter stuff
  bit = 2;
  limit = 1 << nPlanes;

  if (y < nRows) {
    // Data for the upper half of the display is stored in the lower
    // bits of each byte.
    ptr = &matrixbuff[backindex][y * WIDTH * (nPlanes - 1) + x]; // Base addr
    // Plane 0 is a tricky case -- its data is spread about,
    // stored in least two bits not used by the other planes.
    ptr[WIDTH * 2] &= ~B00000011; // Plane 0 R,G mask out in one op
    if (r & 1)
      ptr[WIDTH * 2] |= B00000001; // Plane 0 R: 64 bytes ahead, bit 0
    if (g & 1)
      ptr[WIDTH * 2] |= B00000010; // Plane 0 G: 64 bytes ahead, bit 1
    if (b & 1)
      ptr[WIDTH] |= B00000001; // Plane 0 B: 32 bytes ahead, bit 0
    else
      ptr[WIDTH] &= ~B00000001; // Plane 0 B unset; mask out
    // The remaining three image planes are more normal-ish.
    // Data is stored in the high 6 bits so it can be quickly
    // copied to the DATAPORT register w/6 output lines.
    for (; bit < limit; bit <<= 1) {
      *ptr &= ~B00011100; // Mask out R,G,B in one op
      if (r & bit)
        *ptr |= B00000100; // Plane N R: bit 2
      if (g & bit)
        *ptr |= B00001000; // Plane N G: bit 3
      if (b & bit)
        *ptr |= B00010000; // Plane N B: bit 4
      ptr += WIDTH;        // Advance to next bit plane
    }
  } else {
    // Data for the lower half of the display is stored in the upper
    // bits, except for the plane 0 stuff, using 2 least bits.
    ptr = &matrixbuff[backindex][(y - nRows) * WIDTH * (nPlanes - 1) + x];
    *ptr &= ~B00000011; // Plane 0 G,B mask out in one op
    if (r & 1)
      ptr[WIDTH] |= B00000010; // Plane 0 R: 32 bytes ahead, bit 1
    else
      ptr[WIDTH] &= ~B00000010; // Plane 0 R unset; mask out
    if (g & 1)
      *ptr |= B00000001; // Plane 0 G: bit 0
    if (b & 1)
      *ptr |= B00000010; // Plane 0 B: bit 0
    for (; bit < limit; bit <<= 1) {
      *ptr &= ~B11100000; // Mask out R,G,B in one op
      if (r & bit)
        *ptr |= B00100000; // Plane N R: bit 5
      if (g & bit)
        *ptr |= B01000000; // Plane N G: bit 6
      if (b & bit)
        *ptr |= B10000000; // Plane N B: bit 7
      ptr += WIDTH;        // Advance to next bit plane
    }
  }
}

// -------------------- Interrupt handler stuff --------------------

IRAM_ATTR void IRQ_HANDLER(void *arg) {
  int timer_idx = (int)arg;
  /* Retrieve the interrupt status and the counter value
           from the timer that reported the interrupt */
  uint32_t intr_status = TIMERG1.int_st_timers.val;
  activePanel->updateDisplay(); // Call refresh func for active display
  /* Clear the interrupt
                   and update the alarm time for the timer with without reload
   */
  if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
    TIMERG1.int_clr_timers.t0 = 1;
  }
  /* After the alarm has been triggered
   we need enable it again, so it is triggered the next time */
  TIMERG1.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}


void HUB75::init(uint8_t rows, uint8_t a, uint8_t b, uint8_t c,
                          uint8_t clk, uint8_t lat, uint8_t oe, boolean dbuf,
                          uint8_t width,
                          uint8_t *pinlist
) {
 // R1, G1, B1, R2, G2, B2 pins
  static const uint8_t defaultrgbpins[] = {4, 12, 13, 14, 15, 21};
  memcpy(rgbpins, pinlist ? pinlist : defaultrgbpins, sizeof rgbpins);

  nRows = rows; // Number of multiplexed rows; actual height is 2X this

  // Allocate and initialize matrix buffer:
  int buffsize = width * nRows * 3, // x3 = 3 bytes holds 4 planes "packed"
      allocsize = (dbuf == true) ? (buffsize * 2) : buffsize;
  if (NULL == (matrixbuff[0] = (uint8_t *)malloc(allocsize)))
    return;
  memset(matrixbuff[0], 0, allocsize);
  // If not double-buffered, both buffers then point to the same address:
  matrixbuff[1] = (dbuf == true) ? &matrixbuff[0][buffsize] : matrixbuff[0];


  // Save pin numbers for use by begin() method later.
  _a = a;
  _b = b;
  _c = c;
  _d = d;
  _clk = clk;
  _lat = lat;
  _oe = oe;

  // Look up port registers and pin masks ahead of time,
  // avoids many slow digitalWrite() calls later.
  clkmask = digitalPinToBitMask(clk);
  latport = portOutputRegister(digitalPinToPort(lat));
  latmask = digitalPinToBitMask(lat);
  oeport = portOutputRegister(digitalPinToPort(oe));
  oemask = digitalPinToBitMask(oe);
  addraport = portOutputRegister(digitalPinToPort(a));
  addramask = digitalPinToBitMask(a);
  addrbport = portOutputRegister(digitalPinToPort(b));
  addrbmask = digitalPinToBitMask(b);
  addrcport = portOutputRegister(digitalPinToPort(c));
  addrcmask = digitalPinToBitMask(c);
  addrdport = portOutputRegister(digitalPinToPort(d));
  addrdmask = digitalPinToBitMask(d);
  plane = nPlanes - 1;
  row = nRows - 1;
  swapflag = false;
  backindex = 0; // Array index of back buffer
}

size_t HUB75::get_buffer_length_() {
  return size_t(this->get_width_internal()) * size_t(this->get_height_internal()) / 8u;
}

int HUB75::get_width_internal() { return width_; }
// TODO MAKE IT ACTUALLY CALCULATE HEIGHT
int HUB75::get_height_internal() { return 32; }






}  // namespace hub75
}  // namespace esphome
