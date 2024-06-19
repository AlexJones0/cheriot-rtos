#pragma once
#include <array>
#include <cheri.hh>
#include <cstddef>
#include <cstdint>
#include <debug.hh>
#include <futex.h>
#include <interrupt.h>
#include <optional>
#include <platform-gpio.hh>
#include <platform-spi.hh>
#include <platform/concepts/ethernet.hh>
#include <thread.h>
#include <type_traits>

DECLARE_AND_DEFINE_INTERRUPT_CAPABILITY(EthernetInterruptCap,
                                        EthernetInterrupt,
                                        true,
                                        true);

/**
 * The driver for KSZ8851 SPI Ethernet MAC.
 */
class Ksz8851Ethernet
{
	/**
	 * Flag set when we're debugging this driver.
	 */
	static constexpr bool DebugEthernet = false;

	/**
	 * Flag set to log messages when frames are dropped.
	 */
	static constexpr bool DebugDroppedFrames = false;

	/**
	 * Helper for conditional debug logs and assertions.
	 */
	using Debug = ConditionalDebug<DebugEthernet, "Ethernet driver">;

	/**
	 * Import the Capability helper from the CHERI namespace.
	 */
	template<typename T>
	using Capability = CHERI::Capability<T>;

	/*
	 * GPIO output pins to be used
	 */
	static constexpr uint32_t EthCsPin  = 13;
	static constexpr uint32_t EthRstPin = 14;

	/**
	 * The location of registers
	 */
	enum class RegisterOffset : uint8_t
	{
		/* Chip configuration register */
		CCR = 0x08,
		/* MAC address low */
		MARL = 0x10,
		/* MAC address middle */
		MARM = 0x12,
		/* MAC address high */
		MARH = 0x14,
		/* On-chip bus control register */
		OBCR = 0x20,
		/* EEPROM control register */
		EEPCR = 0x22,
		/* Memory BIST info register */
		MBIR = 0x24,
		/* Global reset register */
		GRR = 0x26,

		/* Wakeup frame registers omitted */

		/* Transmit control register */
		TXCR = 0x70,
		/* Transmit status register */
		TXSR = 0x72,
		/* Receive control register 1 */
		RXCR1 = 0x74,
		/* Receive control register 2 */
		RXCR2 = 0x76,
		/* TXQ memory information register */
		TXMIR = 0x78,
		/* Receive frame header status register */
		RXFHSR = 0x7C,
		/* Receive frame header byte count register */
		RXFHBCR = 0x7E,
		/* TXQ command register */
		TXQCR = 0x80,
		/* RXQ control register */
		RXQCR = 0x82,
		/* TX frame data pointer register */
		TXFDPR = 0x84,
		/* RX frame data pointer register */
		RXFDPR = 0x86,
		/* RX duration timer threshold register */
		RXDTTR = 0x8C,
		/* RX data byte count threshold register */
		RXDBCTR = 0x8E,
		/* Interrupt enable register */
		IER = 0x90,
		/* Interrupt status register */
		ISR = 0x92,
		/* RX frame count and threshold register */
		RXFCTR = 0x9c,
		/* TX next total frames size register */
		TXNTFSR = 0x9E,

		/* MAC address hash table registers omitted */

		/* Flow control low watermark register */
		FCLWR = 0xB0,
		/* Flow control high watermark register */
		FCHWR = 0xB2,
		/* Flow control overrun watermark register */
		FCOHR = 0xB4,
		/* Chip ID and enable register */
		CIDER = 0xC0,
		/* Chip global control register */
		CGCR = 0xC6,
		/* Indirect access control register */
		IACR = 0xC8,
		/* Indirect access data low register */
		IADLR = 0xD0,
		/* Indirect access data high register */
		IADHR = 0xD2,
		/* Power management event control register */
		PMECR = 0xD4,
		/* Go-sleep & wake-up time register */
		GSWUTR = 0xD4,
		/* PHY reset register */
		PHYRR = 0xD4,
		/* PHY 1 MII-register basic control register */
		P1MBCR = 0xE4,
		/* PHY 1 MII-register basic status register */
		P1MBSR = 0xE6,
		/* PHY 1 PHY ID low register */
		PHY1LR = 0xE8,
		/* PHY 1 PHY ID high register */
		PHY1HR = 0xEA,
		/* PHY 1 auto-negotiation advertisement register */
		P1ANAR = 0xEC,
		/* PHY 1 auto-negoatiation link partner ability register */
		P1ANLPR = 0xEE,
		/* Port 1 PHY special control/status, LinkMD */
		P1SCLMD = 0xF4,
		/* Port 1 control register */
		P1CR = 0xF6,
		/* Port 1 status register */
		P1SR = 0xF8,
	};

	using MACAddress = std::array<uint8_t, 6>;

	enum [[clang::flag_enum]] RxFrameHeaderStatus : uint16_t{
	  RxCrcError                = 1 << 0,
	  RxRuntFrame               = 1 << 1,
	  RxFrameTooLong            = 1 << 2,
	  RxFrameType               = 1 << 3,
	  RxMiiError                = 1 << 4,
	  RxUnicastFrame            = 1 << 5,
	  RxMulticastFrame          = 1 << 6,
	  RxBroadcastFrame          = 1 << 7,
	  RxUdpFrameChecksumStatus  = 1 << 10,
	  RxTcpFrameChecksumStatus  = 1 << 11,
	  RxIpFrameChecksumStatus   = 1 << 12,
	  RxIcmpFrameChecksumStatus = 1 << 13,
	  RxFrameValid              = 1 << 15,
	};

	enum [[clang::flag_enum]] RxQueueControl : uint16_t{
	  ReleaseRxErrorFrame            = 1 << 0,
	  StartDmaAccess                 = 1 << 3,
	  AutoDequeueRxQFrameEnable      = 1 << 4,
	  RxFrameCountThresholdEnable    = 1 << 5,
	  RxDataByteCountThresholdEnable = 1 << 6,
	  RxDurationTimerThresholdEnable = 1 << 7,
	  RxIpHeaderTwoByteOffsetEnable  = 1 << 9,
	  RxFrameCountThresholdStatus    = 1 << 10,
	  RxDataByteCountThresholdstatus = 1 << 11,
	  RxDurationTimerThresholdStatus = 1 << 12,
	};

	enum [[clang::flag_enum]] TxQueueControl : uint16_t{
	  ManualEnqueueTxQFrameEnable = 1 << 0,
	  TxQMemoryAvailableMonitor   = 1 << 1,
	  AutoEnqueueTxQFrameEnable   = 1 << 2,
	};

	enum [[clang::flag_enum]] Interrupt : uint16_t{
	  EnergyDetectInterrupt             = 1 << 2,
	  LinkupDetectInterrupt             = 1 << 3,
	  ReceiveMagicPacketDetectInterrupt = 1 << 4,
	  ReceiveWakeupFrameDetectInterrupt = 1 << 5,
	  TransmitSpaceAvailableInterrupt   = 1 << 6,
	  ReceiveProcessStoppedInterrupt    = 1 << 7,
	  TransmitProcessStoppedInterrupt   = 1 << 8,
	  ReceiveOverrunInterrupt           = 1 << 11,
	  ReceiveInterrupt                  = 1 << 13,
	  TransmitInterrupt                 = 1 << 14,
	  LinkChangeInterruptStatus         = 1 << 15,
	};

	enum [[clang::flag_enum]] Port1Status : uint16_t{
	  Partner10BTHalfDuplexCapability  = 1 << 0,
	  Partner10BTFullDuplexCapability  = 1 << 1,
	  Partner100BTHalfDuplexCapability = 1 << 2,
	  Partner100BTFullDuplexCapability = 1 << 3,
	  PartnerFlowControlCapability     = 1 << 4,
	  LinkGood                         = 1 << 5,
	  AutoNegotiationDone              = 1 << 6,
	  MDIXStatus                       = 1 << 7,
	  OperationDuplex                  = 1 << 9,
	  OperationSpeed                   = 1 << 10,
	  PolarityReverse                  = 1 << 13,
	  HPMDIX                           = 1 << 15,
	};

	/**
	 * The futex used to wait for interrupts when packets are available to
	 * receive.
	 */
	const uint32_t *receiveInterruptFutex;

	inline void set_gpio_output_bit(uint32_t bit, bool value) const
	{
		uint32_t output = gpio()->output;
		output &= ~(1 << bit);
		output |= value << bit;
		gpio()->output = output;
	}

	/**
	 * Read a register from the KSZ8851.
	 */
	uint16_t register_read(RegisterOffset reg) const
	{
		uint8_t addr = static_cast<uint8_t>(reg);
		uint8_t be   = (addr & 0x2) == 0 ? 0b0011 : 0b1100;
		uint8_t bytes[2];
		bytes[0] = (0b00 << 6) | (be << 2) | (addr >> 6);
		bytes[1] = (addr << 2) & 0b11110000;

		set_gpio_output_bit(EthCsPin, false);
		spi()->blocking_write(bytes, 2);
		uint16_t val;
		spi()->blocking_read((uint8_t *)&val, 2);
		set_gpio_output_bit(EthCsPin, true);
		return val;
	}

	/**
	 * Write a register to KSZ8851.
	 */
	void register_write(RegisterOffset reg, uint16_t val) const
	{
		uint8_t addr = static_cast<uint8_t>(reg);
		uint8_t be   = (addr & 0x2) == 0 ? 0b0011 : 0b1100;
		uint8_t bytes[2];
		bytes[0] = (0b01 << 6) | (be << 2) | (addr >> 6);
		bytes[1] = (addr << 2) & 0b11110000;

		set_gpio_output_bit(EthCsPin, 0);
		spi()->blocking_write(bytes, 2);
		spi()->blocking_write((uint8_t *)&val, 2);
		spi()->wait_idle();
		set_gpio_output_bit(EthCsPin, 1);
	}

	void register_set(RegisterOffset reg, uint16_t mask) const
	{
		uint16_t old = register_read(reg);
		register_write(reg, old | mask);
	}

	void register_clear(RegisterOffset reg, uint16_t mask) const
	{
		uint16_t old = register_read(reg);
		register_write(reg, old & ~mask);
	}

	/**
	 * Helper.  Returns a pointer to the SPI device.
	 */
	[[nodiscard, gnu::always_inline]] Capability<volatile SonataSpi> spi() const
	{
		return MMIO_CAPABILITY(SonataSpi, spi2);
	}

	/**
	 * Helper.  Returns a pointer to the GPIO device.
	 */
	[[nodiscard, gnu::always_inline]] Capability<volatile SonataGPIO>
	gpio() const
	{
		return MMIO_CAPABILITY(SonataGPIO, gpio);
	}

	uint16_t frames_to_process = 0;

	public:
	/**
	 * Initialise a reference to the Ethernet device.
	 */
	Ksz8851Ethernet()
	{
		// Reset chip
		set_gpio_output_bit(EthRstPin, false);
		thread_millisecond_wait(150);
		set_gpio_output_bit(EthRstPin, true);

		uint16_t cider = register_read(RegisterOffset::CIDER);
		Debug::log("Chip ID is {}", cider);

		// Check the chip ID. The last nibble is revision ID and can be ignored.
		Debug::Assert((cider & 0xFFF0) == 0x8870, "Unexpected Chip ID");

		// Enable QMU Transmit Frame Data Pointer Auto Increment
		register_write(RegisterOffset::TXFDPR, 0x4000);
		// Enable QMU Transmit flow control / Transmit padding / Transmit CRC,
		// and IP/TCP/UDP checksum generation.
		register_write(RegisterOffset::TXCR, 0x00EE);
		// Enable QMU Receive Frame Data Pointer Auto Increment.
		register_write(RegisterOffset::RXFDPR, 0x4000);
		// Configure Receive Frame Threshold for one frame.
		register_write(RegisterOffset::RXFCTR, 0x0001);
		// Enable QMU Receive flow control / Receive all broadcast frames
		// /Receive unicast frames, and IP/TCP/UDP checksum verification etc.
		register_write(RegisterOffset::RXCR1, 0x7CE0);
		// Enable QMU Receive UDP Lite frame checksum verification, UDP Lite
		// frame checksum generation, IPv6 UDP fragment frame pass, and
		// IPv4/IPv6 UDP UDP checksum field is zero pass. In addition (not in
		// the programmer's guide), enable single-frame data burst.
		register_write(RegisterOffset::RXCR2, 0x009C);
		// Enable QMU Receive Frame Count Threshold / RXQ Auto-Dequeue frame.
		register_write(RegisterOffset::RXQCR,
		               RxQueueControl::RxFrameCountThresholdEnable |
		                 RxQueueControl::AutoDequeueRxQFrameEnable);

		// Programmer's guide have a step to set the chip in half-duplex when
		// negotiation failed, but we omit the step.

		// Restart Port 1 auto-negotiation
		register_set(RegisterOffset::P1CR, 1 << 13);

		// Configure Low Watermark to 6KByte available buffer space out of
		// 12KByte.
		register_write(RegisterOffset::FCLWR, 0x0600);
		// Configure High Watermark to 4KByte available buffer space out of
		// 12KByte.
		register_write(RegisterOffset::FCHWR, 0x0400);

		// Clear the interrupt status
		register_write(RegisterOffset::ISR, 0xFFFF);
		receiveInterruptFutex =
		  interrupt_futex_get(STATIC_SEALED_VALUE(EthernetInterruptCap));
		// Enable Receive interrupt
		register_write(RegisterOffset::IER, ReceiveInterrupt);

		// Enable QMU Transmit.
		register_set(RegisterOffset::TXCR, 1 << 0);
		// Enable QMU Receive.
		register_set(RegisterOffset::RXCR1, 1 << 0);
	}

	Ksz8851Ethernet(const Ksz8851Ethernet &) = delete;
	Ksz8851Ethernet(Ksz8851Ethernet &&)      = delete;

	/**
	 * This device does not have a unique MAC address and so users must provide
	 * a locally administered MAC address if more than one device is present on
	 * the same network.
	 */
	static constexpr bool has_unique_mac_address()
	{
		return false;
	}

	static constexpr MACAddress mac_address_default()
	{
		return {0x3a, 0x30, 0x25, 0x24, 0xfe, 0x7a};
	}

	void mac_address_set(MACAddress address = mac_address_default())
	{
		register_write(RegisterOffset::MARH, (address[0] << 8) | address[1]);
		register_write(RegisterOffset::MARM, (address[2] << 8) | address[3]);
		register_write(RegisterOffset::MARL, (address[4] << 8) | address[5]);
	}

	uint32_t receive_interrupt_value()
	{
		return *receiveInterruptFutex;
	}

	int receive_interrupt_complete(Timeout *timeout,
	                               uint32_t lastInterruptValue)
	{
		// If there are frames to process, do not enter wait.
		if (frames_to_process)
		{
			return 0;
		}

		// Our interrupt is level-triggered; if a frame happens to arrive
		// between `receive_frame` call and we marking interrupt as received,
		// it will trigger again immediately after we acknowledge it.

		// Acknowledge the interrupt in the scheduler.
		interrupt_complete(STATIC_SEALED_VALUE(EthernetInterruptCap));
		if (*receiveInterruptFutex == lastInterruptValue)
		{
			Debug::log("Acknowledged interrupt, sleeping on futex {}",
			           receiveInterruptFutex);
			return futex_timed_wait(
			  timeout, receiveInterruptFutex, lastInterruptValue);
		}
		Debug::log("Scheduler announces interrupt has fired");
		return 0;
	}

	/**
	 * Simple class representing a received Ethernet frame.
	 */
	struct Frame
	{
		uint16_t length;
		uint8_t *buffer;

		Frame(uint16_t length) : length(length)
		{
			buffer = new uint8_t[length];
		}

		Frame(const Frame &) = delete;
		Frame(Frame &&other) : buffer(other.buffer), length(other.length)
		{
			other.buffer = nullptr;
			other.length = 0;
		}

		~Frame()
		{
			if (buffer != nullptr)
			{
				delete[] buffer;
			}
		}
	};

	/**
	 * Check the link status of the PHY.
	 */
	bool phy_link_status()
	{
		uint16_t status = register_read(RegisterOffset::P1SR);
		return (status & Port1Status::LinkGood) != 0;
	}

	std::optional<Frame> receive_frame()
	{
		if (frames_to_process == 0)
		{
			uint16_t isr = register_read(RegisterOffset::ISR);
			if (!(isr & ReceiveInterrupt))
			{
				return std::nullopt;
			}

			// Acknowledge the interrupt
			register_write(RegisterOffset::ISR, ReceiveInterrupt);

			// Read number of frames pending.
			// Note that this is only updated when we acknowledge the interrupt.
			frames_to_process = register_read(RegisterOffset::RXFCTR) >> 8;
		}

		// Get number of frames pending
		for (; frames_to_process; frames_to_process--)
		{
			uint16_t status = register_read(RegisterOffset::RXFHSR);
			uint16_t length = register_read(RegisterOffset::RXFHBCR) & 0xFFF;
			bool     valid =
			  (status & RxFrameValid) &&
			  !(status &
			    (RxCrcError | RxRuntFrame | RxFrameTooLong | RxMiiError |
			     RxUdpFrameChecksumStatus | RxTcpFrameChecksumStatus |
			     RxIpFrameChecksumStatus | RxIcmpFrameChecksumStatus));

			if (!valid && DebugDroppedFrames)
			{
				if (DebugDroppedFrames)
				{
					Debug::log("Dropping frame with status: {}", status);
				}

				drop_error_frame();
				continue;
			}

			if (length == 0)
			{
				if (DebugDroppedFrames)
				{
					Debug::log("Dropping frame with zero length");
				}

				drop_error_frame();
				continue;
			}

			Debug::log("Receiving frame of length {}", length);

			auto frame = Frame(length);

			// Reset QMU RXQ frame pointer to zero and start DMA transfer
			// operation.
			register_write(RegisterOffset::RXFDPR, 0x4000);
			register_set(RegisterOffset::RXQCR, StartDmaAccess);

			// Start receiving via SPI.
			uint8_t cmd = 0b10 << 6;
			set_gpio_output_bit(EthCsPin, 0);
			spi()->blocking_write(&cmd, 1);

			// Initial words are RXFHSR and RXFHBCR which we have already know
			// the value.
			uint8_t dummy[8];
			spi()->blocking_read(dummy, 8);

			spi()->blocking_read(frame.buffer, length);

			// Receive needs to be dword-aligned, read the paddings.
			uint16_t pad = (-length) & 0x3;
			if (pad != 0)
			{
				spi()->blocking_read(dummy, pad);
			}

			set_gpio_output_bit(EthCsPin, 1);

			register_clear(RegisterOffset::RXQCR, StartDmaAccess);
			frames_to_process -= 1;

			return std::move(frame);
		}

		return std::nullopt;
	}

	/**
	 * Send a packet.  This will block if no buffer space is available on
	 * device.
	 *
	 * The third argument is a callback that allows the caller to check the
	 * frame before it's sent but after it's copied into memory that isn't
	 * shared with other compartments.
	 */
	bool send_frame(const uint8_t *buffer, uint16_t length, auto &&check)
	{
		// We must check the frame pointer and its length. Although it
		// is supplied by the firewall which is trusted, the firewall
		// does not check the pointer which is coming from external
		// untrusted components.
		Timeout t{10};
		if ((heap_claim_fast(&t, buffer) < 0) ||
		    (!CHERI::check_pointer<CHERI::PermissionSet{
		       CHERI::Permission::Load}>(buffer, length)))
		{
			return false;
		}

		Debug::log("Sending frame of length {}", length);

		// We would need an intermediate buffer to copy the frame into and
		// callback to the firewall before sending it.
		//
		// Also allocate the space for the start of transmission command (1
		// byte), header (4 bytes) and the padding (to 4 bytes).
		uint16_t padded_length  = (length + 3) & ~0x3;
		auto     transmitBuffer = new uint8_t[padded_length + 5];

		// DMA write SPI command.
		transmitBuffer[0] = 0b11 << 6;
		uint32_t header   = 0x8000 | (static_cast<uint32_t>(length) << 16);
		memcpy(transmitBuffer + 1, reinterpret_cast<uint8_t *>(&header), 4);
		memcpy(transmitBuffer + 5, buffer, length);

		if (!check(transmitBuffer + 5, length))
		{
			delete[] transmitBuffer;
			return false;
		}

		// Wait for the transmit buffer to be available on the device side.
		// This needs to include the header.
		while ((register_read(RegisterOffset::TXMIR) & 0xFFF) < length + 4) {}

		// Start QMU DMA transfer operation.
		register_set(RegisterOffset::RXQCR, StartDmaAccess);

		set_gpio_output_bit(EthCsPin, 0);
		spi()->blocking_write(transmitBuffer, padded_length + 5);
		spi()->wait_idle();
		set_gpio_output_bit(EthCsPin, 1);

		// Stop QMU DMA transfer operation.
		register_clear(RegisterOffset::RXQCR, StartDmaAccess);

		// TxQ manual enqueue.
		register_set(RegisterOffset::TXQCR,
		             TxQueueControl::ManualEnqueueTxQFrameEnable);

		delete[] transmitBuffer;
		return true;
	}

	private:
	void drop_error_frame()
	{
		register_set(RegisterOffset::RXQCR, ReleaseRxErrorFrame);
		// Wait for confirmation of frame release before attempting to process
		// next frame.
		while (register_read(RegisterOffset::RXQCR) & ReleaseRxErrorFrame) {}
	}
};

using EthernetDevice = Ksz8851Ethernet;

static_assert(EthernetAdaptor<EthernetDevice>);
