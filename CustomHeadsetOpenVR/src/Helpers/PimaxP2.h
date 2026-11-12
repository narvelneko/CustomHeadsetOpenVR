#pragma once
// Pimax "P2" headsets: the 5K and 8K series and the Artisan. They share one USB id (0483:0101,
// 0483 is ST's generic vendor id) and PimaxDirectUSB, which only knows the 0x34A4 headsets,
// can't reach them. This identifies them over USB when PVR (pi_server) isn't available.
// No PVR SDK here.
#include "../Config/Config.h"

#include <cstdint>
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
}
