#include "b500rep.h"

#include <stdlib.h>
#include <unistd.h>

b500rep::b500rep(rs232_bus & rs232) : 
	m_rs232(rs232), 
	m_needle(false), 
	m_downdown(false), 
	m_down(false), 
	m_up(false),
	m_upup(false),
	m_minus(false),
	m_plus(false)
{
	//Set baudrate to 38400, disable hardware flow control, 8 data bits, no parity, 1 stop bit
	m_rs232.set_baudrate_parity(38400, false, 8, false, 1);
	this->update();
}

b500rep::~b500rep()
{
}

//Calculate the CCITT-CRC16 checksum
uint16_t
b500rep::snap_calc_crc(const uint8_t * buf, const uint16_t len) const
{
	const uint16_t crc_poly = 0x1021;
	uint16_t temp_crc       = 0x0000;
	uint16_t crc            = 0x0000;

	for(int i=0; i<len; i++)
	{
		const uint8_t input_byte = buf[i];

		temp_crc = input_byte;
		temp_crc = temp_crc << 8;
		crc = temp_crc ^ crc;

		for(uint8_t j=0; j<8; j++)
		{
			if(crc > 0x7fff) //0x7fff 32767. Checks if the most significant bit is set to 1.
			{
				crc = crc << 1;
				crc = crc ^ crc_poly;
			}
			else
			{
				crc = crc << 1;
			}
		}
	}
	return crc;
}

//Convert the different led statusses into the appropriate byte to be passed to the B500REP
uint8_t
b500rep::calc_led_bits() const
{
	uint8_t led_mask = 0x00;

	led_mask |= (m_downdown	& 0x01) << 0;
	led_mask |= (m_down	& 0x01) << 1;
	led_mask |= (m_up	& 0x01) << 2;
	led_mask |= (m_upup	& 0x01) << 3;
	led_mask |= (m_minus	& 0x01) << 4;
	led_mask |= (m_plus	& 0x01) << 5;

	return led_mask;	
}

void
b500rep::downdown(bool value)
{
	m_downdown = value;
}

void
b500rep::down(bool value)
{
	m_down = value;
}

void
b500rep::up(bool value)
{
	m_up = value;
}

void
b500rep::upup(bool value)
{
	m_upup = value;
}

void
b500rep::minus(bool value)
{
	m_minus = value;
}

void
b500rep::plus(bool value)
{
	m_plus = value;
}

//Set needle position. It ranges from -10000 (-5) to 12000 (+6). 2000 counts is roughly 1 count on the scale.
//Calibration is needed to make the needle very accurate
void
b500rep::needle(int16_t value)
{
	if(value > 12000) value = 12000;
	if(value < -10000) value = -10000;
	m_needle = value;
}

//A very rough approximation, just a first try. If you require better accuracy, calibrate.
//I'll probably do this later myself aswell.
void
b500rep::vario(float value)
{
	const float ratio = 12000.0f/6.0f;
	int16_t rawval = (int16_t)(value * ratio);
	this->needle(rawval);
}

/* Update the B500REP from this objects's status. This function is called at roughly 10Hz. Faster doesn't seem to improve anything.
It is required to continuously call this function, in order to achieve accurate needle positions. If this function is only called once, the needle will 
either too early or too late. Calling this function again makes the needle move to the correct position.
I don't care about race conditions (yet), since this is just a proof-of-concept. If any updates to this object's members are not atomic, locking is needed.
*/
void
b500rep::update()
{
	const unsigned int len = sizeof(struct snap_header) + sizeof(struct adu_data) + sizeof(struct snap_crc);
	uint8_t * buf = (uint8_t *)malloc(len);

	struct snap_header * header = (struct snap_header *)buf;
	struct adu_data * data = (struct adu_data *)(buf + sizeof(struct snap_header));
	struct snap_crc * crc = (struct snap_crc *)(buf + sizeof(struct snap_header) + sizeof(struct adu_data));

	const uint8_t crc_offset	= 2;    //skip the first two bytes of each packet for the CRC: preamble and sync
	const uint8_t * crc_payload	= (buf + crc_offset);
	const uint16_t crc_len		= len - (sizeof(struct snap_crc) + crc_offset);

	const uint8_t led_mask 		= this->calc_led_bits();

	header->preamble	= 0x53;
	header->sync		= 0x54;
	header->hdb2		= 0x50;
	header->hdb1		= 0x48;
	header->dab1		= 0x06;
	header->sab1		= 0x02;
	
	data->lift_high		= (m_needle >> 8) & 0x00ff;
	data->lift_low		= (m_needle >> 0) & 0x00ff;
	data->led_bits		= led_mask;
	data->audio_level	= 0x00;
	data->avg_high		= data->lift_high;
	data->avg_low		= data->lift_low;
	data->lift_mode		= 0x00;
	
	const uint16_t crc_val	= snap_calc_crc(crc_payload, crc_len);
	
	crc->crc_high		= (crc_val >> 8) & 0x00ff;
	crc->crc_low		= (crc_val >> 0) & 0x00ff;

	const int ret = m_rs232.write(buf, len);
	free((void *)buf);
}

//A wrapper function for my threading-code. This calls the update function at 10Hz.
void
b500rep::run()
{
	while(true)
	{
		this->update();
		usleep(100000);
	}
}
