#include "PimaxP2PvrData.h"

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <shared_mutex>
#include <string>

#include "../Config/ConfigLoader.h"
#include "../Driver/DriverLog.h"
#include "PimaxP2.h"
#include "PvrCache.h"

namespace PimaxP2 {

// Set once IdentifyWithoutPvr claims a headset; the other identified values are written before it and
// not changed afterwards.
static std::atomic<bool> s_identified = false;
static std::atomic<Config::HeadsetType> s_identifiedType = Config::HeadsetType::None;
// per eye, as identified
static std::atomic<uint32_t> s_identifiedWidth = 0;
static std::atomic<uint32_t> s_identifiedHeight = 0;
// the Saved* functions answer instead of PVR: from identification until PVR connects, and again after
// PVR went away (PvrTookOver, PvrWentAway)
static std::atomic<bool> s_useSavedData = false;
#ifdef PVR_EXISTS
// the checked saved data of the identified headset, or null; owned by s_loadedData
static std::atomic<const PvrCache::Data*> s_savedData = nullptr;
// guards s_loadedData and s_identifiedSerial
static std::mutex s_savedDataLock;
// saved data per serial number (null if there is none or it can't be used); entries are never freed,
// the Saved* functions may be reading them
static std::map<std::string, std::unique_ptr<const PvrCache::Data>> s_loadedData;
static std::string s_identifiedSerial;

// <config folder>/pvr-data-<serial>.json; empty if the serial can't name a file
static std::string SavedPvrDataPath(std::string_view serial){
	const std::string name = PvrCache::FileName(serial);
	return name.empty() ? "" : driverConfigLoader.GetConfigFolder() + name;
}

// The saved PVR data of a P2 headset found over USB, checked against it (product id and USB serial
// number); null if there is none or it can't be used. Each serial number's file is read and logged
// once; the result is kept.
static const PvrCache::Data* LoadSavedPvrData(const Identity& identity){
	std::lock_guard<std::mutex> lock(s_savedDataLock);
	const auto found = s_loadedData.find(identity.serialNumber);
	if(found != s_loadedData.end()){
		return found->second.get();
	}
	std::unique_ptr<const PvrCache::Data>& result = s_loadedData[identity.serialNumber];
	const std::string serial = MaskSerial(identity.serialNumber);
	const std::string path = SavedPvrDataPath(identity.serialNumber);
	if(path.empty()){
		DriverLog("No saved PVR data: the USB serial number '%s' can't be used to find it", serial.c_str());
		return nullptr;
	}
	auto data = std::make_unique<PvrCache::Data>();
	std::string error;
	if(!PvrCache::Load(path, *data, error)){
		if(error == "no file"){
			DriverLog("No saved PVR data for serial %s; run the headset with Pimax Play once to save it", serial.c_str());
		}else{
			DriverLog("Not using the saved PVR data for serial %s: %s", serial.c_str(), error.c_str());
		}
		return nullptr;
	}
	error = PvrCache::CheckHeadset(*data, kUsbProductId, identity.serialNumber);
	if(!error.empty()){
		DriverLog("Not using the saved PVR data for serial %s: %s", serial.c_str(), error.c_str());
		return nullptr;
	}
	result = std::move(data);
	return result.get();
}
#endif

bool IdentifyWithoutPvr(PimaxInfo& info){
	static std::mutex s_lock;
	std::lock_guard<std::mutex> lock(s_lock);
	if(info.connected || info.pvrConnected || info.headsetType != Config::HeadsetType::None){
		// found already (PimaxDirectUSB, PVR or here)
		return false;
	}
	std::optional<Identity> identity = Identify();
	if(!identity){
		return false;
	}
	// headsets that are not claimed, logged once each
	static std::set<std::string> s_loggedRejections;
	const auto reject = [&](const char* reason){
		if(s_loggedRejections.insert(identity->productName + "/" + identity->serialNumber + "/" + reason).second){
			DriverLog("Not using P2 headset '%s' (serial %s) without PVR: %s", identity->productName.c_str(),
				MaskSerial(identity->serialNumber).c_str(), reason);
		}
		return false;
	};
	Config::HeadsetType headsetType = identity->headsetType;
	uint32_t resolutionX = 0, resolutionY = 0;
	float ipd = 0, cantingAngle = 0;
#ifdef PVR_EXISTS
	const PvrCache::Data* savedData = LoadSavedPvrData(*identity);
	if(savedData){
		if(headsetType == Config::HeadsetType::None){
			// the model from the PVR product name saved with the data (eg: a 5K Plus or an Artisan that
			// has run with Pimax Play once)
			headsetType = TypeFromProduct(kUsbProductId, savedData->productName, "");
		}
		resolutionX = (uint32_t)savedData->panelWidth / 2;
		resolutionY = (uint32_t)savedData->panelHeight;
		ipd = savedData->Ipd();
		cantingAngle = savedData->CantingAngle();
	}
#endif
	if(headsetType == Config::HeadsetType::None){
		return reject("its model isn't known from the USB product string (only the 8K X's, 'Pimax P2N', is) and there is no saved PVR data for it");
	}
	Config::BaseHeadsetConfig* config = driverConfig.ConfigFromHeadsetType(headsetType);
	if(!config || !config->enable){
		return reject("its headset type is not enabled in the settings");
	}
	if(headsetType == Config::HeadsetType::Pimax8KX && (resolutionX == 0 || resolutionY == 0)){
		// 8K X panel mode 4320x3168 (both eyes); PVR reports the same once connected
		resolutionX = 2160;
		resolutionY = 3168;
	}
	if(resolutionX == 0 || resolutionY == 0){
		return reject("its panel size isn't known; run it with Pimax Play once");
	}
	{
		// EnsurePvrSession connects PVR under a shared lock, so this doesn't interleave with it; PVR's
		// values win if it connected meanwhile
		std::lock_guard<std::shared_mutex> pvrLock(PimaxCommon::pvrLock);
		if(info.pvrConnected || info.connected){
			return false;
		}
		s_identifiedType = headsetType;
		s_identifiedWidth = resolutionX;
		s_identifiedHeight = resolutionY;
#ifdef PVR_EXISTS
		{
			std::lock_guard<std::mutex> savedDataLock(s_savedDataLock);
			s_identifiedSerial = identity->serialNumber;
		}
		s_savedData = savedData;
#endif
		// the image from the saved data works; PVR's isn't usable before its panel is ready
		WaitForPanelWithoutLimit();
		s_useSavedData = true;
		s_identified = true;
		info.headsetType = headsetType;
		info.resolutionX = resolutionX;
		info.resolutionY = resolutionY;
		if(ipd > 0){
			info.ipd = ipd;
			info.cantingAngle = cantingAngle;
		}
		// P2 headsets have no eye tracking
		info.hasEyeTracking = false;
		info.connected = true;
	}
#ifdef PVR_EXISTS
	if(savedData){
		const std::string captured = savedData->CaptureTimeString();
		DriverLog("Using saved PVR data for '%s' (serial %s) from %s, %d days old: panel %dx%d, IPD %.1f mm, canting %.2f deg",
			savedData->productName.c_str(), MaskSerial(identity->serialNumber).c_str(), captured.c_str(), savedData->AgeDays(),
			savedData->panelWidth, savedData->panelHeight, ipd, cantingAngle);
		DriverLog("IPD from %s; reopen Pimax Play once to refresh after changing the IPD", captured.c_str());
	}
#endif
	DriverLog("Identified P2 headset over USB without PVR: '%s' (%04x:%04x), type %d, %ux%u per eye", identity->productName.c_str(),
		kUsbVendorId, kUsbProductId, (int)headsetType, resolutionX, resolutionY);
	return true;
}

#ifdef PVR_EXISTS
bool PvrTookOver(){
	if(!s_identified || !s_useSavedData || !PimaxCommon::GetInfo().pvrConnected){
		return false;
	}
	{
		// EnsurePvrSession sets pvrConnected before it has written PVR's resolution, IPD and
		// view_rotation_fix, under a shared lock: wait for it to finish
		std::lock_guard<std::shared_mutex> lock(PimaxCommon::pvrLock);
	}
	const PimaxInfo info = PimaxCommon::GetInfo();
	if(!info.pvrConnected){
		return false;
	}
	s_useSavedData = false;
	DriverLog("PVR connected: using it instead of the saved data");
	if(info.resolutionX != s_identifiedWidth || info.resolutionY != s_identifiedHeight){
		// the render resolution SteamVR was given at activation doesn't change until it restarts
		DriverLog("PVR reports %ux%u per eye, the headset was activated with %ux%u; restart SteamVR to use PVR's size",
			info.resolutionX, info.resolutionY, (uint32_t)s_identifiedWidth, (uint32_t)s_identifiedHeight);
	}
	return true;
}

bool PvrWentAway(){
	if(!s_identified || s_useSavedData || PimaxCommon::GetInfo().pvrConnected){
		return false;
	}
	s_useSavedData = true;
	DriverLog("PVR disconnected: using the saved data again");
	return true;
}

static int EyeIndex(vr::EVREye eye){
	return eye == vr::Eye_Left ? 0 : 1;
}

// view_rotation_fix as the headset shims set it in PVR (parallelProjection); read on each call (cheap)
// so a changed setting takes effect when the distortion is rebuilt
static int ViewRotationFix(){
	const Config::BaseHeadsetConfig* config = driverConfig.ConfigFromHeadsetType(s_identifiedType);
	return (config ? config->parallelProjection : driverConfig.dreamAir.parallelProjection) ? 1 : 0;
}

bool SavedEyeFov(vr::EVREye eye, pvrFovPort& fov){
	if(!s_useSavedData){
		return false;
	}
	const PvrCache::Data* data = s_savedData;
	if(!data){
		fov.LeftTan = fov.RightTan = fov.UpTan = fov.DownTan = 1.f;
		return true;
	}
	const PvrCache::Fov& saved = data->eyes[ViewRotationFix()][EyeIndex(eye)].fov;
	fov.LeftTan = saved.left;
	fov.RightTan = saved.right;
	fov.UpTan = saved.up;
	fov.DownTan = saved.down;
	return true;
}

bool SavedDistortedUV(vr::EVREye eye, pvrVector2f uv, pvrVector2f outUV[3]){
	if(!s_useSavedData){
		return false;
	}
	const PvrCache::Data* data = s_savedData;
	const int viewRotationFix = ViewRotationFix();
	for(int channel = 0; channel < 3; channel++){
		outUV[channel] = uv;
		if(data){
			data->DistortedUV(viewRotationFix, EyeIndex(eye), channel, uv.x, uv.y, outUV[channel].x, outUV[channel].y);
		}
	}
	return true;
}

bool SavedFovTextureSize(pvrSizei& size){
	if(!s_useSavedData){
		return false;
	}
	const PvrCache::Data* data = s_savedData;
	if(!data){
		size.w = (int)s_identifiedWidth;
		size.h = (int)s_identifiedHeight;
		return true;
	}
	const PvrCache::Eye& saved = data->eyes[ViewRotationFix()][0];
	size.w = saved.textureWidth;
	size.h = saved.textureHeight;
	return true;
}
#endif

}
