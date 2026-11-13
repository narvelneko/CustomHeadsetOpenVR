#pragma once
// Pimax "P2" headsets: the 5K and 8K series and the Artisan. They share one USB id (0483:0101,
// 0483 is ST's generic vendor id) and PimaxDirectUSB, which only knows the 0x34A4 headsets,
// can't reach them. This identifies them over USB when PVR (pi_server) isn't available.
// No PVR SDK here.
#include "../Config/Config.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

namespace PimaxP2 {
	constexpr uint16_t kUsbVendorId = 0x0483;
	// also the PVR product id of these headsets
	constexpr uint16_t kUsbProductId = 0x0101;

	struct Identity {
		// from the USB product string (TypeFromUsbProductName); None if the model isn't known from it
		Config::HeadsetType headsetType = Config::HeadsetType::None;
		// USB product string, eg: "Pimax P2N" (8K X)
		std::string productName;
		// USB serial number (the same as PVR's serial number on the tested 8K X)
		std::string serialNumber;
	};

	// a serial number for the log, with the last 4 characters replaced by "XXXX"
	std::string MaskSerial(std::string_view serial);

	// The model from a P2 USB product string. These are board codes, not model names, and only the
	// 8K X's ("Pimax P2N") is known; anything else is None.
	Config::HeadsetType TypeFromUsbProductName(std::string_view usbProductName);

	// Looks for a P2 headset on USB. Only devices whose product string starts with "Pimax" count;
	// headsetType is None when the product string isn't a known one.
	// Rate limited: within 5 s of the last search it returns nothing without searching.
	std::optional<Identity> Identify();

	// For EnsurePvrSession: pi_server reports a 0x0 panel for ~5 s after it starts (seen on the 8K X),
	// and the resolution, IPD and distortion of a P2 headset aren't usable until then. False while a P2
	// headset (PVR product id 0x0101) reports a 0x0 panel, for up to 15 s per wait (logged once);
	// true at once for any other product id.
	bool IsPvrPanelReady(int pvrProductId, int panelWidth, int panelHeight);
	// From now on IsPvrPanelReady waits for the panel without the 15 s limit: the headset runs on saved
	// data meanwhile (PimaxP2PvrData), which is better than PVR without a panel
	void WaitForPanelWithoutLimit();

	// Experimental (displayPowerWithoutPimax): switches the 8K X's display on without Pimax's software.
	// After a power cycle the display stays off until pi_server has opened the headset. A thread sends
	// the display-on sequence (HID feature reports on interface 0) and then a keepalive every second;
	// without the keepalive the display turns off about 5 minutes later. It pauses while pi_server.exe
	// runs (checked every 5 s) or PVR is connected (NotePvrConnected), and switches the display on again
	// when they are gone.
	// Only USB devices whose product string is the 8K X's ("Pimax P2N") are written to.
	// Does nothing unless the 8K X is enabled and its displayPowerWithoutPimax is set, or if it is
	// already running. Call when the driver starts, after the config is loaded.
	void StartDisplayPower();
	// The display appears about 3.4 s after the sequence, and SteamVR looks for it when the HMD is
	// activated. Call before activating the HMD: waits until 4 s have passed since the display-on
	// sequence was first sent (if it was). Returns at once when the display power isn't running.
	void WaitForDisplayBeforeActivation();
	// Called while PVR is connected (PVR thread, every 20 ms); the display power counts PVR as connected
	// until a second after the last call
	void NotePvrConnected();

	// The raw IPD value the 8K X reports: HID feature report 0xF0 on interface 0, read with GET (nothing is
	// written), bytes 36-37 little-endian, of the headset with this USB serial number (any 8K X if empty).
	// Opens the interface for each read and closes it again, so it never holds the headset when pi_server
	// wants it. nullopt when it can't be read; when the headset isn't found it is looked for again only
	// after 10 s.
	std::optional<uint16_t> ReadIpdRaw(std::string_view serial);
	// Calls tick five times a second from a thread of its own, until Stop (an exception from tick is
	// logged, and the next tick runs as usual). Does nothing if already running.
	void StartIpdFollower(std::function<void()> tick);

	// Stops the display power and the IPD follower; called when the driver shuts down
	void Stop();
}
