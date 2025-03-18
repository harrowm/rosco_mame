// license:BSD-3-Clause
// copyright-holders:Chris Hanson
/*
 * rosco_m68k.cpp - rosco_m68k
 *
 * Created on: September 5, 2022
 *     Author: Chris Hanson
 */

#include "emu.h"

#include "bus/ata/ataintf.h"
#include "bus/rs232/rs232.h"
// #include "cpu/m68000/m68000.h"
#include "cpu/m68000/m68010.h"
#include "machine/mc68681.h"
#include "machine/spi_sdcard.h"

#include "debugger.h"

#define VERBOSE	 1
#define LOG_OUTPUT_FUNC printf
#include "logmacro.h"


#ifdef _MSC_VER
#define FUNCNAME __func__
#else
#define FUNCNAME __PRETTY_FUNCTION__
#endif


#define LOG_CHANGE(p,v) \
do { \
	if (p != v) LOG("%s: change " #p " to %d\n", FUNCNAME, (int)v); \
} \
while (0)


class rosco_m68k_state : public driver_device
{
public:
	rosco_m68k_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_duart(*this, "duart")
		, m_terminal(*this, "terminal")
		, m_host(*this, "host")
		, m_sdcard(*this, "sdcard")
		, m_ata(*this, "ata")
	{ }

	void rosco_m68k(machine_config &config);

private:
	required_device<cpu_device> m_maincpu;
	required_device<xr68c681_device> m_duart;

	required_device<rs232_port_device> m_terminal;
	required_device<rs232_port_device> m_host;

	required_device<spi_sdcard_sdhc_device> m_sdcard;
	
	required_device<ata_interface_device> m_ata;

	void write_red_led(int state);
	void write_green_led(int state);

	void mem_map(address_map &map);
	void cpu_space_map(address_map &map);

	virtual void machine_start() override;
	virtual void machine_reset() override;

	// Pointer to System ROMs needed by bootvect_r and masking RAM buffer for post reset accesses
	uint16_t *m_sysrom = nullptr;
	uint16_t m_sysram[8]{};
	uint16_t bootvect_r(offs_t offset);
	void bootvect_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
};


/* Input ports */
static INPUT_PORTS_START( rosco_m68k )
INPUT_PORTS_END


/* Terminal default settings. */
static DEVICE_INPUT_DEFAULTS_START(terminal)
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD", 0xff, RS232_BAUD_115200 )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD", 0xff, RS232_BAUD_115200 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_8 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY", 0xff, RS232_PARITY_NONE )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
DEVICE_INPUT_DEFAULTS_END


void rosco_m68k_state::rosco_m68k(machine_config &config)
{
	M68010(config, m_maincpu, 10_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &rosco_m68k_state::mem_map);
	m_maincpu->set_addrmap(m68000_base_device::AS_CPU_SPACE, &rosco_m68k_state::cpu_space_map);

	// Set up DUART, both binding to serial ports and handling GPIO.
	// IP0 = CTS_A
	// IP1 = CTS_B
	// IP2 = SPI_CIPO
	// IP3 = ???
	// IP4 = ???
	// IP5 = ???
	//
	// OP0 = RTS_A
	// OP1 = RTS_B
	// OP2 = SPI_CS
	// OP3 = RED_LED
	// OP4 = SPI_SCK
	// OP5 = GREEN_LED
	// OP6 = SPI_COPI
	// OP7 = SPI_CS1

	// XR68C681(config, m_duart, 10_MHz_XTAL);
	XR68C681(config, m_duart, 3.6864_MHz_XTAL);
	m_duart->irq_cb().set_inputline(m_maincpu, M68K_IRQ_4);

	RS232_PORT(config, m_terminal, default_rs232_devices, "terminal");
	m_duart->a_tx_cb().set("terminal", FUNC(rs232_port_device::write_txd));
	m_terminal->rxd_handler().set(m_duart, FUNC(xr68c681_device::rx_a_w));
	m_terminal->set_option_device_input_defaults("terminal", DEVICE_INPUT_DEFAULTS_NAME(terminal));
	m_duart->outport_cb().set("terminal", FUNC(rs232_port_device::write_rts)).bit(0);
	m_terminal->cts_handler().set(m_duart, FUNC(xr68c681_device::ip0_w));

	RS232_PORT(config, m_host, default_rs232_devices, nullptr);
	m_duart->b_tx_cb().set("host", FUNC(rs232_port_device::write_txd));
	m_host->rxd_handler().set(m_duart, FUNC(xr68c681_device::rx_b_w));
	m_duart->outport_cb().append("host", FUNC(rs232_port_device::write_rts)).bit(1);
	m_host->cts_handler().set(m_duart, FUNC(xr68c681_device::ip1_w));

	// m_duart->outport_cb().append(FUNC(rosco_m68k_state::write_red_led)).bit(3);
	// m_duart->outport_cb().append(FUNC(rosco_m68k_state::write_green_led)).bit(5);
	
	SPI_SDCARD(config, m_sdcard, 0);
	m_duart->outport_cb().append(m_sdcard, FUNC(spi_sdcard_sdhc_device::spi_ss_w)).bit(2).invert();
	m_duart->outport_cb().append(m_sdcard, FUNC(spi_sdcard_sdhc_device::spi_clock_w)).bit(4);
	m_duart->outport_cb().append(m_sdcard, FUNC(spi_sdcard_sdhc_device::spi_mosi_w)).bit(6);
	m_sdcard->spi_miso_callback().set(m_duart, FUNC(xr68c681_device::ip2_w));
	
	ATA_INTERFACE(config, m_ata, 0).options(ata_devices, "hdd", nullptr, false);
	m_ata->irq_handler().set_inputline(m_maincpu, M68K_IRQ_4);
}

// WRITE_LINE_MEMBER(rosco_m68k_state::write_red_led)
// {
// 	// Do nothing for now.
// // 	LOG("RED: %d\n", state);
// }

// WRITE_LINE_MEMBER(rosco_m68k_state::write_green_led)
// {
// 	// Do nothing for now.
// // 	LOG("GREEN: %d\n", state);
// }

void rosco_m68k_state::mem_map(address_map &map)
{
	map.unmap_value_high();
	map(0x000000, 0x00000f).ram().w(FUNC(rosco_m68k_state::bootvect_w));       /* After first write we act as RAM */
	map(0x000000, 0x00000f).rom().r(FUNC(rosco_m68k_state::bootvect_r));       /* ROM mirror just during reset */
	map(0x000010, 0x0fffff).ram(); /* 1MB RAM */
	map(0xe00000, 0xefffff).rom().region("monitor", 0); /* 1MB ROM (max) */
	map(0xf00000, 0xf0001f).rw("duart", FUNC(xr68c681_device::read), FUNC(xr68c681_device::write)).umask16(0x00ff);
	map(0xf80040, 0xf8004f).rw("ata", FUNC(ata_interface_device::cs0_r), FUNC(ata_interface_device::cs0_w)).umask16(0xffff);
	map(0xf80050, 0xf8005f).rw("ata", FUNC(ata_interface_device::cs1_r), FUNC(ata_interface_device::cs1_w)).umask16(0xffff);
}

void rosco_m68k_state::cpu_space_map(address_map &map)
{
    map(0xfffff0, 0xffffff).m(m_maincpu, FUNC(m68010_device::autovectors_map));
    map(0xfffff9, 0xfffff9).r(m_duart, FUNC(xr68c681_device::get_irq_vector));
}

void rosco_m68k_state::machine_start()
{
	/* Setup pointer to bootvector in ROM for bootvector handler bootvect_r */
	m_sysrom = (uint16_t *)(memregion("monitor")->base());
}

void rosco_m68k_state::machine_reset()
{
	m_sdcard->spi_clock_w(CLEAR_LINE);

	/* Reset pointer to bootvector in ROM for bootvector handler bootvect_r */
	if (m_sysrom == &m_sysram[0]) /* Condition needed because memory map is not setup first time */
		m_sysrom = (uint16_t*)(memregion("monitor")->base());
}

/* Boot vector handlers, the PCB hardwires the first 16 bytes from 0xfc0000 to 0x0 at reset. */
uint16_t rosco_m68k_state::bootvect_r(offs_t offset){
	return m_sysrom[offset];
}

void rosco_m68k_state::bootvect_w(offs_t offset, uint16_t data, uint16_t mem_mask){
	m_sysram[offset % std::size(m_sysram)] &= ~mem_mask;
	m_sysram[offset % std::size(m_sysram)] |= (data & mem_mask);
	m_sysrom = &m_sysram[0]; // redirect all upcoming accesses to masking RAM until reset.
}


/* ROM definition */
ROM_START( rosco_m68k )
	ROM_REGION16_BE(0x100000, "monitor", 0)
	ROM_LOAD( "rosco_m68k.bin", 0x00000, 0x100000, CRC(e5ab64c9) SHA1(eea400611a274f7137963a90688bf25dd9dc516d))
ROM_END


/* Driver */
/*    YEAR  NAME       PARENT  COMPAT  MACHINE     INPUT       CLASS             INIT        COMPANY  FULLNAME            FLAGS */
COMP( 2020, rosco_m68k, 0,      0,      rosco_m68k, rosco_m68k, rosco_m68k_state, empty_init, "ROSCO", "ROSCO M68K", MACHINE_NO_SOUND_HW | MACHINE_SUPPORTS_SAVE )

// compile with: 
// make SUBTARGET=rosco_m68k SOURCE=./src/mame/rosco/rosco_m68k.cpp
// .. update .. this doesnt include liboptional, so I compiled the whole of make with "make"
//              and went for a cup of coffee .. or two ..
// command line to start executable is (with logging on):
//              ./mame rosco_m68k -hard1 disk/roshdi.img -log -oslog    