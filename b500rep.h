#ifndef B500REP_H
#define B500REP_H

#include "rs232_bus.h"

#include "runnable.h"

struct snap_header
{
	uint8_t preamble;       //0x53
	uint8_t sync;           //0x54
	uint8_t hdb2;           //header definition byte 2
	uint8_t hdb1;           //header definition byte 1
	uint8_t dab1;           //dst address byte 1
	uint8_t sab1;           //src address byte 1
} __attribute__ ((__packed__));

struct snap_crc
{
	uint8_t crc_high;
	uint8_t crc_low;
} __attribute__ ((__packed__));


struct adu_data
{
	uint8_t lift_low;
	uint8_t lift_high;
	uint8_t led_bits;
	uint8_t lift_unit;
	uint8_t audio_level;
	uint8_t avg_low;
	uint8_t avg_high;
	uint8_t lift_mode;
} __attribute__ ((__packed__));


class b500rep : 
	public runnable
{
	private:
	rs232_bus & m_rs232;
	int m_needle;
	bool m_downdown;
	bool m_down;
	bool m_up;
	bool m_upup;
	bool m_minus;
	bool m_plus;

	uint16_t snap_calc_crc(const uint8_t * buf, uint16_t len) const;
	uint8_t calc_led_bits() const;

	public:
	b500rep(rs232_bus & rs232);
	virtual ~b500rep();


	void downdown(bool value);
	void down(bool value);
	void up(bool value);
	void upup(bool value);
	void minus(bool value);
	void plus(bool value);
	
	void needle(int16_t value);
	void vario(float value);
	void update();
	void run();
};

#endif
