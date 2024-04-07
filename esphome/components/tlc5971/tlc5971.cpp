#include "tlc5971.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tlc5971 {

static const char *const TAG = "tlc5971";

void TLC5971::setup() {
  this->data_pin_->setup();
  this->data_pin_->digital_write(true);
  this->clock_pin_->setup();
  this->clock_pin_->digital_write(true);

  this->pwm_amounts_.resize(this->num_chips_ * N_CHANNELS_PER_CHIP, 0);

  ESP_LOGCONFIG(TAG, "Done setting up TLC5971 output component.");
}
void TLC5971::dump_config() {
  ESP_LOGCONFIG(TAG, "TLC5971:");
  LOG_PIN("  Data Pin: ", this->data_pin_);
  LOG_PIN("  Clock Pin: ", this->clock_pin_);
  ESP_LOGCONFIG(TAG, "  Number of chips: %u", this->num_chips_);
}

void TLC5971::loop() {
  if (!this->update_) 
    return;

  uint32_t command;

  // Magic word for write
  command = 0x25;

  command <<= 5;
  // OUTTMG = 1, EXTGCK = 0, TMGRST = 1, DSPRPT = 1, BLANK = 0 -> 0x16
  command |= 0x16;

  command <<= 7;
  command |= 0x7F;  // default 100% brigthness

  command <<= 7;
  command |= 0x7F;  // default 100% brigthness

  command <<= 7;
  command |= 0x7F;  // default 100% brigthness

  for (uint8_t n = 0; n < num_chips_; n++) {
    this->transfer(command >> 24);
    this->transfer(command >> 16);
    this->transfer(command >> 8);
    this->transfer(command);

    // 12 channels per TLC59711
    for (int8_t c = 11; c >= 0; c--) {
      // 16 bits per channel, send MSB first
      this->transfer(pwm_amounts_.at(n * 12 + c) >> 8);
      this->transfer(pwm_amounts_.at(n * 12 + c));
    }
  }

  this->update_ = false;
}

void TLC5971::transfer(uint8_t send) {
  uint8_t data = send;
  transfer(&data, 1);
}

void TLC5971::transfer(uint8_t *buffer, size_t len) {
  uint8_t startbit = 0x80;

  bool towrite, lastmosi = !(buffer[0] & startbit);
  uint8_t bitdelay_us = (1000000 / 1000000) / 2;

  for (size_t i = 0; i < len; i++) {
    uint8_t reply = 0;
    uint8_t send = buffer[i];

    for (uint8_t b = startbit; b != 0; b = b >> 1) {
      if (bitdelay_us) {
        delayMicroseconds(bitdelay_us);
      }

      towrite = send & b;
      if ((lastmosi != towrite)) {
        this->data_pin_->digital_write(towrite);
        lastmosi = towrite;
      }

      this->clock_pin_->digital_write(true);

      if (bitdelay_us) {
        delayMicroseconds(bitdelay_us);
      }

      this->clock_pin_->digital_write(false);
    }
  }
}
}  // namespace tlc5971
}  // namespace esphome
