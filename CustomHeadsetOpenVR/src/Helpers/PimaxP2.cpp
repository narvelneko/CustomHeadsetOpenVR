#include "PimaxP2.h"

#include <hidapi/hidapi.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cwchar>
#include <exception>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <utility>

#include "../Driver/DriverLog.h"
#ifdef _WIN32
#include <windows.h>
#include <tlhelp32.h>
#endif

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

// last time PVR was seen connected (NotePvrConnected), in ms of steady_clock; -1 if never
static std::atomic<int64_t> s_pvrSeenMs = -1;

static int64_t SteadyMs(){
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void NotePvrConnected(){
	s_pvrSeenMs = SteadyMs();
}

static bool IsPvrConnected(){
	const int64_t seen = s_pvrSeenMs;
	return seen >= 0 && SteadyMs() - seen < 1000;
}

// The display-on sequence is the one Coreforge's Monado pimax driver sends to a P2N (8K X)
// (init_display_8kx): 64-byte feature reports, report id 0xF0, on interface 0. Tested on a Pimax
// Vision 8K X after a power cycle with no Pimax software: the display appears about 3.4 s after the
// sequence. Power (0x1E) alone isn't enough. Sending the sequence to a display that is already on
// does no harm. The keepalive must be repeated every second, or the display turns off about 5 minutes
// later (and the NVIDIA driver stalls if SteamVR is drawing to it). Reading report 0xF0 back returns
// the same bytes with the display on or off, so the power state can't be checked.
class DisplayPower {
public:
	DisplayPower(){
		thread = std::thread(&DisplayPower::Run, this);
	}

	// only destroyed after StopDisplayPower (see DisplayPowerHolder)
	~DisplayPower(){
		Stop();
	}

	DisplayPower(const DisplayPower&) = delete;
	DisplayPower& operator=(const DisplayPower&) = delete;

	void Stop(){
		{
			std::lock_guard<std::mutex> lock(mutex);
			stopping = true;
			wake.notify_all();
		}
		if(thread.joinable()){
			thread.join();
		}
	}

	void WaitBeforeActivation(){
		std::unique_lock<std::mutex> lock(mutex);
		// the first attempt (open + sequence) takes milliseconds; don't hang activation if the USB stack does
		wake.wait_for(lock, std::chrono::seconds(3), [this]{ return firstAttemptDone || stopping; });
		if(!firstSequenceSentAt){
			return;
		}
		const auto ready = *firstSequenceSentAt + std::chrono::seconds(4);
		const auto start = std::chrono::steady_clock::now();
		if(start >= ready){
			return;
		}
		wake.wait_until(lock, ready, [this]{ return stopping; });
		const double waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
		DriverLog("P2 display power: waited %.1f s before activating the HMD, for the display to appear", waited);
	}

private:
	static constexpr int kMaxWriteFailures = 5;

	static bool IsPiServerRunning(){
#ifdef _WIN32
		HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
		if(snapshot == INVALID_HANDLE_VALUE){
			return false;
		}
		PROCESSENTRY32W entry = {};
		entry.dwSize = sizeof(entry);
		bool found = false;
		for(BOOL ok = Process32FirstW(snapshot, &entry); ok; ok = Process32NextW(snapshot, &entry)){
			if(_wcsicmp(entry.szExeFile, L"pi_server.exe") == 0){
				found = true;
				break;
			}
		}
		CloseHandle(snapshot);
		return found;
#else
		return false;
#endif
	}

	// opens interface 0 of the 8K X, or returns null
	hid_device* Open(){
		std::lock_guard<std::mutex> hidLock(s_hidLock);
		hid_device_info* devices = hid_enumerate(kUsbVendorId, kUsbProductId);
		hid_device* handle = nullptr;
		for(hid_device_info* device = devices; device && !handle; device = device->next){
			if(device->interface_number != 0){
				continue;
			}
			const std::string productName = Narrow(device->product_string);
			if(TypeFromUsbProductName(productName) != Config::HeadsetType::Pimax8KX){
				continue;
			}
			handle = hid_open_path(device->path);
			if(handle){
				DriverLog("P2 display power: opened '%s' (serial %s)", productName.c_str(), MaskSerial(Narrow(device->serial_number)).c_str());
			}
		}
		hid_free_enumeration(devices);
		return handle;
	}

	static bool Send(hid_device* handle, unsigned char command, int index1 = -1, unsigned char value1 = 0, int index2 = -1, unsigned char value2 = 0){
		unsigned char report[64] = {};
		report[0] = 0xF0;
		report[3] = command;
		if(index1 >= 0){
			report[index1] = value1;
		}
		if(index2 >= 0){
			report[index2] = value2;
		}
		return hid_send_feature_report(handle, report, sizeof(report)) == (int)sizeof(report);
	}

	static bool SendDisplayOn(hid_device* handle){
		return Send(handle, 0x0C, 35, 0x18, 36, 0xF6)	// init
			&& Send(handle, 0x13)						// parallel projections off
			&& Send(handle, 0x1E, 55, 0x01)				// display power
			&& Send(handle, 0x0D);						// keepalive
	}

	// waits, returns false when stopping
	bool WaitOrStop(std::chrono::steady_clock::duration duration){
		std::unique_lock<std::mutex> lock(mutex);
		return !wake.wait_for(lock, duration, [this]{ return stopping; });
	}

	void FirstAttemptDone(){
		{
			std::lock_guard<std::mutex> lock(mutex);
			if(firstAttemptDone){
				return;
			}
			firstAttemptDone = true;
			wake.notify_all();
		}
	}

	void Run(){
		enum class State { Starting, On, Paused, NotFound, Failed };
		State state = State::Starting;
		hid_device* handle = nullptr;
		bool sendDisplayOn = true;
		int failures = 0;
		bool piServerRunning = false;
		std::chrono::steady_clock::time_point nextPiServerCheck = {};
		std::chrono::steady_clock::time_point nextOpen = {};
		do{
			const auto now = std::chrono::steady_clock::now();
			if(now >= nextPiServerCheck){
				piServerRunning = IsPiServerRunning();
				nextPiServerCheck = now + std::chrono::seconds(5);
			}
			const bool pvrConnected = IsPvrConnected();
			if(piServerRunning || pvrConnected){
				// pi_server switches the display on and keeps it on; don't write alongside it
				if(state != State::Paused){
					DriverLog("P2 display power: paused, %s handles the display", piServerRunning ? "pi_server.exe" : "PVR");
					state = State::Paused;
				}
				if(handle){
					hid_close(handle);
					handle = nullptr;
				}
				FirstAttemptDone();
				continue;
			}
			if(!handle){
				if(now < nextOpen){
					continue;
				}
				handle = Open();
				if(!handle){
					if(state != State::NotFound){
						DriverLog("P2 display power: can't open the 8K X's HID interface 0; retrying every 5 s");
						state = State::NotFound;
					}
					nextOpen = now + std::chrono::seconds(5);
					FirstAttemptDone();
					continue;
				}
				// it may have been unplugged, power cycled or switched off by pi_server
				sendDisplayOn = true;
				failures = 0;
			}
			if(sendDisplayOn){
				if(SendDisplayOn(handle)){
					sendDisplayOn = false;
					failures = 0;
					bool first = false;
					{
						std::lock_guard<std::mutex> lock(mutex);
						if(!firstSequenceSentAt){
							firstSequenceSentAt = std::chrono::steady_clock::now();
							first = true;
						}
					}
					DriverLog("P2 display power: display-on sequence sent%s; keepalive every second from now on",
						first ? " (pi_server.exe not running)" : "");
					state = State::On;
				}else{
					failures++;
				}
			}else if(Send(handle, 0x0D)){
				if(failures > 0 && state == State::On){
					DriverLog("P2 display power: keepalive works again after %d failures", failures);
				}
				failures = 0;
			}else{
				failures++;
				if(failures == 1){
					DriverLog("P2 display power: keepalive failed");
				}
			}
			if(failures >= kMaxWriteFailures){
				DriverLog("P2 display power: %d writes failed in a row; closing the headset, reopening in 5 s", failures);
				hid_close(handle);
				handle = nullptr;
				failures = 0;
				state = State::Failed;
				nextOpen = std::chrono::steady_clock::now() + std::chrono::seconds(5);
			}
			FirstAttemptDone();
		}while(WaitOrStop(std::chrono::seconds(1)));
		if(handle){
			hid_close(handle);
		}
		DriverLog("P2 display power: stopped");
	}

	std::mutex mutex;
	std::condition_variable wake;
	// guarded by mutex
	bool stopping = false;
	bool firstAttemptDone = false;
	std::optional<std::chrono::steady_clock::time_point> firstSequenceSentAt;
	// started last, after the members it uses
	std::thread thread;
};

static std::mutex s_displayPowerLock;
// Never destroyed: if StopDisplayPower (driver cleanup) didn't run, a static destructor would join the
// thread at process exit, possibly under the loader lock (deadlock) or after the thread was killed.
// The object and its thread then simply live until the process ends.
static std::shared_ptr<DisplayPower>& DisplayPowerHolder(){
	static std::shared_ptr<DisplayPower>* s_holder = new std::shared_ptr<DisplayPower>();
	return *s_holder;
}

void StartDisplayPower(){
	// only the 8K X's display-on sequence is known (and tested)
	const Config::PimaxHeadsetConfig& config = driverConfig.pimax8KX;
	if(!config.enable || !config.displayPowerWithoutPimax){
		return;
	}
	std::lock_guard<std::mutex> lock(s_displayPowerLock);
	if(DisplayPowerHolder()){
		return;
	}
	{
		std::lock_guard<std::mutex> hidLock(s_hidLock);
		hid_init();
	}
	DriverLog("P2 display power: starting (displayPowerWithoutPimax, experimental)");
	DisplayPowerHolder() = std::make_shared<DisplayPower>();
}

static void StopDisplayPower(){
	std::shared_ptr<DisplayPower> displayPower;
	{
		std::lock_guard<std::mutex> lock(s_displayPowerLock);
		displayPower = std::move(DisplayPowerHolder());
		DisplayPowerHolder() = nullptr;
	}
	if(displayPower){
		displayPower->Stop();
	}
}

void WaitForDisplayBeforeActivation(){
	std::shared_ptr<DisplayPower> displayPower;
	{
		std::lock_guard<std::mutex> lock(s_displayPowerLock);
		displayPower = DisplayPowerHolder();
	}
	if(displayPower){
		displayPower->WaitBeforeActivation();
	}
}

// for ReadIpdRaw, guarded by s_hidLock: the path of the 8K X's interface 0, found once (again after it
// can't be opened), and when it was last looked for in vain
static std::string s_ipdDevicePath;
static std::optional<std::chrono::steady_clock::time_point> s_ipdDeviceMissedAt;

std::optional<uint16_t> ReadIpdRaw(std::string_view serial){
	std::lock_guard<std::mutex> hidLock(s_hidLock);
	if(s_ipdDevicePath.empty()){
		const auto now = std::chrono::steady_clock::now();
		if(s_ipdDeviceMissedAt && now - *s_ipdDeviceMissedAt < std::chrono::seconds(10)){
			return std::nullopt;
		}
		hid_init();
		hid_device_info* devices = hid_enumerate(kUsbVendorId, kUsbProductId);
		for(hid_device_info* device = devices; device && s_ipdDevicePath.empty(); device = device->next){
			if(device->interface_number == 0 && device->path && TypeFromUsbProductName(Narrow(device->product_string)) == Config::HeadsetType::Pimax8KX
				&& (serial.empty() || Narrow(device->serial_number) == serial)){
				s_ipdDevicePath = device->path;
			}
		}
		hid_free_enumeration(devices);
		if(s_ipdDevicePath.empty()){
			s_ipdDeviceMissedAt = now;
			return std::nullopt;
		}
		s_ipdDeviceMissedAt.reset();
	}
	hid_device* device = hid_open_path(s_ipdDevicePath.c_str());
	if(!device){
		// unplugged, or another device now
		s_ipdDevicePath.clear();
		return std::nullopt;
	}
	unsigned char report[64] = {};
	report[0] = 0xF0;
	const int length = hid_get_feature_report(device, report, sizeof(report));
	hid_close(device);
	if(length < 38){
		return std::nullopt;
	}
	return (uint16_t)(report[36] | (report[37] << 8));
}

// Calls tick about once a second until stopped
class IpdFollower {
public:
	explicit IpdFollower(std::function<void()> tick) : tick(std::move(tick)) {
		thread = std::thread(&IpdFollower::Run, this);
	}

	~IpdFollower(){
		Stop();
	}

	IpdFollower(const IpdFollower&) = delete;
	IpdFollower& operator=(const IpdFollower&) = delete;

	void Stop(){
		{
			std::lock_guard<std::mutex> lock(mutex);
			stopping = true;
			wake.notify_all();
		}
		if(thread.joinable()){
			thread.join();
		}
	}

private:
	void Run(){
		std::unique_lock<std::mutex> lock(mutex);
		while(!stopping){
			lock.unlock();
			try{
				tick();
			}catch(const std::exception& e){
				DriverLog("IPD follower: %s", e.what());
			}catch(...){
				DriverLog("IPD follower: unknown exception");
			}
			lock.lock();
			wake.wait_for(lock, std::chrono::seconds(1), [this]{ return stopping; });
		}
	}

	const std::function<void()> tick;
	std::mutex mutex;
	std::condition_variable wake;
	// guarded by mutex
	bool stopping = false;
	// started last, after the members it uses
	std::thread thread;
};

static std::mutex s_ipdFollowerLock;
// never destroyed, like DisplayPowerHolder
static std::unique_ptr<IpdFollower>& IpdFollowerHolder(){
	static std::unique_ptr<IpdFollower>* s_holder = new std::unique_ptr<IpdFollower>();
	return *s_holder;
}

void StartIpdFollower(std::function<void()> tick){
	std::lock_guard<std::mutex> lock(s_ipdFollowerLock);
	if(!IpdFollowerHolder()){
		IpdFollowerHolder() = std::make_unique<IpdFollower>(std::move(tick));
	}
}

void Stop(){
	StopDisplayPower();
	std::unique_ptr<IpdFollower> follower;
	{
		std::lock_guard<std::mutex> lock(s_ipdFollowerLock);
		follower = std::move(IpdFollowerHolder());
	}
	follower.reset();
}

}
