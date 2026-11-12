#include "PimaxP2.h"

#include <hidapi/hidapi.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <set>

#include "../Driver/DriverLog.h"

namespace PimaxP2 {

// hidapi initialises itself on first use, which isn't thread safe; held around this file's
// hid_enumerate and hid_open calls, which can run on different threads
static std::mutex s_hidLock;

// the USB strings are ASCII
static std::string Narrow(const wchar_t* text){
	std::string result;
	for(const wchar_t* c = text; c && *c; c++){
		result += (char)*c;
	}
	return result;
}

std::string MaskSerial(std::string_view serial){
	if(serial.length() > 4){
		return std::string(serial.substr(0, serial.length() - 4)) + "XXXX";
	}
	return std::string(serial);
}

Config::HeadsetType TypeFromUsbProductName(std::string_view usbProductName){
	if(usbProductName == "Pimax P2N"){
		return Config::HeadsetType::Pimax8KX;
	}
	return Config::HeadsetType::None;
}

std::optional<Identity> Identify(){
	static std::mutex s_lock;
	static std::optional<std::chrono::steady_clock::time_point> s_lastRun;
	// product strings of 0483:0101 devices that aren't Pimax headsets, logged once each
	static std::set<std::string> s_ignoredProducts;
	// the last headset logged, to log each one once while identification is repeated
	static std::string s_lastLogged;
	std::lock_guard<std::mutex> lock(s_lock);
	const auto now = std::chrono::steady_clock::now();
	if(s_lastRun && now - *s_lastRun < std::chrono::seconds(5)){
		return std::nullopt;
	}
	s_lastRun = now;

	std::lock_guard<std::mutex> hidLock(s_hidLock);
	hid_device_info* devices = hid_enumerate(kUsbVendorId, kUsbProductId);
	if(!devices){
		return std::nullopt;
	}
	std::optional<Identity> result;
	for(hid_device_info* device = devices; device && !result; device = device->next){
		std::string productName = Narrow(device->product_string);
		if(productName.rfind("Pimax", 0) != 0){
			if(s_ignoredProducts.insert(productName).second){
				DriverLog("Ignoring USB device %04x:%04x '%s': not a Pimax headset", kUsbVendorId, kUsbProductId, productName.c_str());
			}
			continue;
		}
		Identity identity;
		identity.headsetType = TypeFromUsbProductName(productName);
		identity.productName = productName;
		identity.serialNumber = Narrow(device->serial_number);
		result = identity;
	}
	hid_free_enumeration(devices);
	if(result && s_lastLogged != result->productName + "/" + result->serialNumber){
		s_lastLogged = result->productName + "/" + result->serialNumber;
		DriverLog("Found P2 headset over USB: '%s', serial %s, type %d", result->productName.c_str(),
			MaskSerial(result->serialNumber).c_str(), (int)result->headsetType);
	}
	return result;
}

static std::atomic<bool> s_waitForPanelWithoutLimit = false;

void WaitForPanelWithoutLimit(){
	s_waitForPanelWithoutLimit = true;
}

bool IsPvrPanelReady(int pvrProductId, int panelWidth, int panelHeight){
	if(pvrProductId != kUsbProductId){
		return true;
	}
	// EnsurePvrSession runs on several threads
	static std::mutex s_lock;
	static std::optional<std::chrono::steady_clock::time_point> s_waitStart;
	static std::chrono::steady_clock::time_point s_lastCall;
	static bool s_logged = false;
	std::lock_guard<std::mutex> lock(s_lock);
	const auto now = std::chrono::steady_clock::now();
	if(panelWidth != 0 && panelHeight != 0){
		s_waitStart.reset();
		return true;
	}
	// while waiting, the PVR thread asks every 20 ms; a longer gap means PVR went away meanwhile
	// (eg: pi_server was restarted), so the wait starts over
	if(!s_waitStart || now - s_lastCall > std::chrono::seconds(5)){
		s_waitStart = now;
		s_logged = false;
	}
	s_lastCall = now;
	if(s_waitForPanelWithoutLimit || now - *s_waitStart < std::chrono::seconds(15)){
		if(!s_logged){
			DriverLog("PVR connected but the panel isn't ready yet (0x0); waiting");
			s_logged = true;
		}
		return false;
	}
	DriverLog("PVR still reports a 0x0 panel after 15 s; connecting anyway");
	s_waitStart.reset();
	return true;
}

}
