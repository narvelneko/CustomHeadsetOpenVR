#include "PimaxP2PvrData.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <shared_mutex>
#include <string>
#include <vector>

#include "../Config/ConfigLoader.h"
#include "../Driver/DriverLog.h"
#include "MiscHelper.h"
#include "PimaxP2.h"
#include "PvrCache.h"
#include "PvrProfile.h"

namespace PimaxP2 {

// Set once IdentifyWithoutPvr claims a headset; the other identified values are written before it and
// not changed afterwards.
static std::atomic<bool> s_identified = false;
static std::atomic<Config::HeadsetType> s_identifiedType = Config::HeadsetType::None;
// per eye, as identified
static std::atomic<uint32_t> s_identifiedWidth = 0;
static std::atomic<uint32_t> s_identifiedHeight = 0;
// the Saved* functions answer instead of PVR: from identification until PVR connects, and again after
// PVR went away (PvrTookOver, SavedDataChanged)
static std::atomic<bool> s_useSavedData = false;
#ifdef PVR_EXISTS
// The saved data the Saved* functions answer from (null: none, the fallback). A reader copies the
// pointer under the lock, so the data lives at least until that reader is done with it.
static std::mutex s_savedDataPointerLock;
static std::shared_ptr<const PvrCache::Data> s_savedData;

static std::shared_ptr<const PvrCache::Data> SavedData(){
	std::lock_guard<std::mutex> lock(s_savedDataPointerLock);
	return s_savedData;
}

static void SetSavedData(std::shared_ptr<const PvrCache::Data> data){
	std::lock_guard<std::mutex> lock(s_savedDataPointerLock);
	s_savedData = std::move(data);
}

// guards s_loadedData, s_loadedProfiles and s_identifiedSerial
static std::mutex s_savedDataLock;
// saved data per serial number (null if there is none or it can't be used)
static std::map<std::string, std::shared_ptr<const PvrCache::Data>> s_loadedData;
static std::string s_identifiedSerial;

// an IPD profile with its HID calibration, made once
struct LoadedProfile {
	PvrProfile::Profile profile;
	PvrProfile::HidCalibration calibration;
};
// profiles per serial number (null if there is none or it can't be used); never freed
static std::map<std::string, std::unique_ptr<const LoadedProfile>> s_loadedProfiles;
// The IPD profile of the identified headset, or null: the saved data is then the profile blended for
// the headset's IPD (FollowIpd, 8K X only; other P2 headsets keep the fixed IPD)
static std::atomic<const LoadedProfile*> s_profile = nullptr;
// metres: the IPD the saved data was blended for, and the IPD used when the headset's was never read
// (the single saved capture's, i.e. the last one PVR had, or the middle capture)
static std::atomic<float> s_appliedIpd = 0.f;
static std::atomic<float> s_fixedIpd = 0.f;
// Written at identification, before the follower thread starts: the USB serial number whose IPD is
// read, and the texture sizes [view_rotation_fix][eye] (width, height) every blend keeps, those of the
// first one, so that the render target size doesn't change with the IPD
static std::string s_hidSerial;
static int s_pinnedTexture[2][2][2] = {};
// the identified headset's info, to keep its IPD current while there is no PVR (as PvrThread does with PVR's)
static PimaxInfo* s_info = nullptr;
// set when the saved data changed (IPD); SavedDataChanged reports it
static std::atomic<bool> s_savedDataChanged = false;

// <config folder>/pvr-data-<serial>.json; empty if the serial can't name a file
static std::string SavedPvrDataPath(std::string_view serial){
	const std::string name = PvrCache::FileName(serial);
	return name.empty() ? "" : driverConfigLoader.GetConfigFolder() + name;
}

// The saved PVR data of a P2 headset found over USB, checked against it (product id and USB serial
// number); null if there is none or it can't be used. Each serial number's file is read and logged
// once; the result is kept.
static std::shared_ptr<const PvrCache::Data> LoadSavedPvrData(const Identity& identity, bool logMissing){
	std::lock_guard<std::mutex> lock(s_savedDataLock);
	const auto found = s_loadedData.find(identity.serialNumber);
	if(found != s_loadedData.end()){
		return found->second;
	}
	std::shared_ptr<const PvrCache::Data>& result = s_loadedData[identity.serialNumber];
	const std::string serial = MaskSerial(identity.serialNumber);
	const std::string path = SavedPvrDataPath(identity.serialNumber);
	if(path.empty()){
		DriverLog("No saved PVR data: the USB serial number '%s' can't be used to find it", serial.c_str());
		return nullptr;
	}
	auto data = std::make_shared<PvrCache::Data>();
	std::string error;
	if(!PvrCache::Load(path, *data, error)){
		if(error == "no file"){
			if(logMissing){
				DriverLog("No saved PVR data for serial %s; run the headset with Pimax Play once to save it", serial.c_str());
			}
		}else{
			DriverLog("Not using the saved PVR data for serial %s: %s", serial.c_str(), error.c_str());
		}
		return nullptr;
	}
	error = PvrCache::CheckHeadset(*data, kUsbProductId, identity.serialNumber);
	if(error.empty()){
		error = PvrProfile::CheckBounds(*data);
	}
	if(!error.empty()){
		DriverLog("Not using the saved PVR data for serial %s: %s", serial.c_str(), error.c_str());
		return nullptr;
	}
	result = std::move(data);
	return result;
}

// The IPD profile of a P2 headset found over USB, checked against it like LoadSavedPvrData; null if
// there is none or it can't be used. Read and logged once per serial number.
static const LoadedProfile* LoadProfile(const Identity& identity){
	std::lock_guard<std::mutex> lock(s_savedDataLock);
	const auto found = s_loadedProfiles.find(identity.serialNumber);
	if(found != s_loadedProfiles.end()){
		return found->second.get();
	}
	std::unique_ptr<const LoadedProfile>& result = s_loadedProfiles[identity.serialNumber];
	const std::string serial = MaskSerial(identity.serialNumber);
	const std::string name = PvrProfile::FileName(identity.serialNumber);
	if(name.empty()){
		return nullptr;
	}
	auto loaded = std::make_unique<LoadedProfile>();
	PvrProfile::Profile& profile = loaded->profile;
	std::string error;
	if(!PvrProfile::Load(driverConfigLoader.GetConfigFolder() + name, profile, error)){
		if(error == "no file"){
			DriverLog("No IPD profile for serial %s (pvrcache sweep makes one)", serial.c_str());
		}else{
			DriverLog("Not using the IPD profile for serial %s: %s", serial.c_str(), error.c_str());
		}
		return nullptr;
	}
	error = PvrProfile::CheckHeadset(profile, kUsbProductId, identity.serialNumber);
	if(!error.empty()){
		DriverLog("Not using the IPD profile for serial %s: %s", serial.c_str(), error.c_str());
		return nullptr;
	}
	loaded->calibration = PvrProfile::MakeHidCalibration(profile);
	const float gap = PvrProfile::LargestGap(profile);
	if(gap > 0.004f){
		DriverLog("Warning: the IPD profile for serial %s has a gap of %.1f mm between captures; the lens correction is less exact there (sweep again, slowly)",
			serial.c_str(), gap * 1000.f);
	}
	const PvrCache::Eye& first = profile.captures.front().data.eyes[0][0];
	for(const PvrProfile::Capture& capture : profile.captures){
		const PvrCache::Eye& eye = capture.data.eyes[0][0];
		if(eye.textureWidth != first.textureWidth || eye.textureHeight != first.textureHeight){
			DriverLog("The IPD profile's texture sizes differ between captures (%dx%d at %.1f mm, %dx%d at %.1f mm); the size at activation is kept",
				first.textureWidth, first.textureHeight, profile.MinIpd() * 1000.f, eye.textureWidth, eye.textureHeight, capture.pvrIpd * 1000.f);
			break;
		}
	}
	result = std::move(loaded);
	return result.get();
}

static float ClampToProfile(const PvrProfile::Profile& profile, float ipd){
	return ipd < profile.MinIpd() ? profile.MinIpd() : ipd > profile.MaxIpd() ? profile.MaxIpd() : ipd;
}

// The profile blended for an IPD, with the pinned texture sizes
static std::shared_ptr<const PvrCache::Data> MakeBlend(const PvrProfile::Profile& profile, float ipd){
	auto data = std::make_shared<PvrCache::Data>(PvrProfile::Blend(profile, ipd));
	for(int vrf = 0; vrf < 2; vrf++){
		for(int e = 0; e < 2; e++){
			data->eyes[vrf][e].textureWidth = s_pinnedTexture[vrf][e][0];
			data->eyes[vrf][e].textureHeight = s_pinnedTexture[vrf][e][1];
		}
	}
	return data;
}

struct IpdReading {
	bool valid = false;
	// metres
	float ipd = 0;
	const char* source = "";
	// HID raw value, -1 if none
	int raw = -1;
};

// The headset's IPD over HID: calibrated with the profile's captures (or Monado's formula if none has a
// raw value). Not valid if it can't be read or isn't plausible.
static IpdReading ReadIpd(const LoadedProfile& loaded, std::string_view serial){
	IpdReading reading;
	const std::optional<uint16_t> raw = ReadIpdRaw(serial);
	if(!raw){
		return reading;
	}
	reading.raw = *raw;
	if(PvrProfile::CalibratedIpd(loaded.calibration, *raw, reading.ipd)){
		reading.source = "HID, calibrated with the profile";
	}else{
		reading.ipd = PvrProfile::HidFormulaIpd(*raw);
		reading.source = "HID, Monado's formula (no raw values in the profile)";
	}
	reading.valid = reading.ipd > 0.045f && reading.ipd < 0.085f;
	return reading;
}

// Runs five times a second on PimaxP2's IPD follower thread (8K X with a profile). While there is no
// PVR it blends the profile again when PvrProfile::IpdFilter says so. When HID can't be read it holds
// the applied IPD; the fixed IPD is used only while HID was never read.
static void FollowIpd(){
	// only this thread uses these
	static std::string s_lastSource;
	static bool s_hidEverRead = false;
	static PvrProfile::IpdFilter s_filter;
	const LoadedProfile* loaded = s_profile;
	if(!loaded || !s_useSavedData){
		// PVR gives the IPD while it is connected
		s_filter.Reset();
		return;
	}
	const PvrProfile::Profile& profile = loaded->profile;
	const IpdReading reading = ReadIpd(*loaded, s_hidSerial);
	float ipd = 0;
	std::string source;
	if(reading.valid){
		if(!s_hidEverRead){
			// forget the fixed IPD's readings
			s_filter.Reset();
			s_hidEverRead = true;
		}
		ipd = reading.ipd;
		source = reading.source;
	}else if(s_hidEverRead){
		source = "HID can't be read now: holding the IPD";
	}else{
		ipd = s_fixedIpd;
		source = "fixed, the last known IPD (the headset's can't be read)";
	}
	if(s_lastSource != source){
		s_lastSource = source;
		DriverLog("IPD source: %s", source.c_str());
	}
	if(!reading.valid && s_hidEverRead){
		s_filter.Hold();
		return;
	}
	const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
	const float applied = s_appliedIpd;
	if(!s_filter.Update(ipd, seconds, applied, profile.MinIpd(), profile.MaxIpd(), ipd)){
		return;
	}
	std::shared_ptr<const PvrCache::Data> blend = MakeBlend(profile, ipd);
	const float ipdMm = blend->Ipd();
	const float cantingAngle = blend->CantingAngle();
	{
		// EnsurePvrSession writes PVR's values under a shared lock; they win if it connected meanwhile
		std::lock_guard<std::shared_mutex> pvrLock(PimaxCommon::pvrLock);
		if(!s_useSavedData || s_info->pvrConnected){
			return;
		}
		SetSavedData(std::move(blend));
		s_info->ipd = ipdMm;
		s_info->cantingAngle = cantingAngle;
	}
	DriverLog("IPD %.1f -> %.1f mm (%s, latest HID raw %d): rebuilding the distortion from the profile", applied * 1000.f,
		ipd * 1000.f, source.c_str(), reading.raw);
	s_appliedIpd = ipd;
	s_savedDataChanged = true;
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
	// the IPD profile if there is one, otherwise the single saved capture (whose IPD is then the fallback IPD)
	const LoadedProfile* loaded = LoadProfile(*identity);
	const std::shared_ptr<const PvrCache::Data> singleData = LoadSavedPvrData(*identity, !loaded);
	std::shared_ptr<const PvrCache::Data> savedData = singleData;
	// only the 8K X's IPD can be read over HID
	const bool followHid = loaded && identity->headsetType == Config::HeadsetType::Pimax8KX;
	IpdReading ipdReading;
	float fixedIpd = 0, initialIpd = 0;
	if(loaded){
		const PvrProfile::Profile& profile = loaded->profile;
		fixedIpd = ClampToProfile(profile, singleData ? PvrProfile::PvrIpd(*singleData) : profile.captures[profile.captures.size() / 2].pvrIpd);
		if(followHid){
			ipdReading = ReadIpd(*loaded, identity->serialNumber);
		}
		initialIpd = ipdReading.valid ? ClampToProfile(profile, ipdReading.ipd) : fixedIpd;
		if(!ipdReading.valid){
			ipdReading.source = followHid ? "fixed, the last known IPD (the headset's can't be read yet)" : "fixed, the last known IPD (only the 8K X's can be read)";
		}
		// the first blend's texture sizes are kept for every blend
		const PvrCache::Data first = PvrProfile::Blend(profile, initialIpd);
		for(int vrf = 0; vrf < 2; vrf++){
			for(int e = 0; e < 2; e++){
				s_pinnedTexture[vrf][e][0] = first.eyes[vrf][e].textureWidth;
				s_pinnedTexture[vrf][e][1] = first.eyes[vrf][e].textureHeight;
			}
		}
		savedData = MakeBlend(profile, initialIpd);
	}
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
		s_hidSerial = identity->serialNumber;
		SetSavedData(savedData);
		s_profile = loaded;
		s_appliedIpd = initialIpd;
		s_fixedIpd = fixedIpd;
		s_info = &info;
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
	if(loaded){
		const PvrProfile::Profile& profile = loaded->profile;
		DriverLog("Using the IPD profile for '%s' (serial %s): %d captures, IPD %.1f-%.1f mm; IPD %.1f mm (%s, HID raw %d)",
			profile.productName.c_str(), MaskSerial(identity->serialNumber).c_str(), (int)profile.captures.size(),
			profile.MinIpd() * 1000.f, profile.MaxIpd() * 1000.f, initialIpd * 1000.f, ipdReading.source, ipdReading.raw);
		if(followHid){
			StartIpdFollower(FollowIpd);
		}
	}else if(savedData){
		const std::string captured = savedData->CaptureTimeString();
		DriverLog("Using saved PVR data for '%s' (serial %s) from %s, %d days old: panel %dx%d, IPD %.1f mm, canting %.2f deg",
			savedData->productName.c_str(), MaskSerial(identity->serialNumber).c_str(), captured.c_str(), savedData->AgeDays(),
			savedData->panelWidth, savedData->panelHeight, ipd, cantingAngle);
		DriverLog("IPD from %s; reopen Pimax Play once to refresh after changing the IPD", captured.c_str());
	}
#endif
	if(headsetType == Config::HeadsetType::Pimax8KX && !driverConfig.pimax8KX.displayPowerWithoutPimax){
		DriverLog("If the 8K X's display isn't found after a power cycle, displayPowerWithoutPimax (experimental) can switch it on");
	}
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

bool SavedDataChanged(){
	if(!s_identified){
		return false;
	}
	if(s_useSavedData){
		return s_savedDataChanged.exchange(false);
	}
	if(PimaxCommon::GetInfo().pvrConnected){
		return false;
	}
	s_useSavedData = true;
	s_savedDataChanged = false;
	DriverLog("PVR disconnected: using the saved data again");
	if(s_profile){
		// PVR's last IPD and canting are still in PimaxInfo: those of the blend in use go with the mesh
		const std::shared_ptr<const PvrCache::Data> data = SavedData();
		std::lock_guard<std::shared_mutex> pvrLock(PimaxCommon::pvrLock);
		if(data && !s_info->pvrConnected){
			s_info->ipd = data->Ipd();
			s_info->cantingAngle = data->CantingAngle();
		}
	}
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
	const std::shared_ptr<const PvrCache::Data> data = SavedData();
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
	const std::shared_ptr<const PvrCache::Data> data = SavedData();
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
	const std::shared_ptr<const PvrCache::Data> data = SavedData();
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

static bool IsP2HeadsetType(Config::HeadsetType type){
	return type == Config::HeadsetType::Pimax5KPlus || type == Config::HeadsetType::Pimax5KSuper
		|| type == Config::HeadsetType::Pimax8KX || type == Config::HeadsetType::Pimax8KPlus
		|| type == Config::HeadsetType::PimaxArtisan;
}

// The view_rotation_fix that the headset shims keep set in PVR (parallelProjection), with the same
// config (and fallback) they use
static int ConfiguredViewRotationFix(){
	return PimaxCommon::GetHeadsetConfig().parallelProjection ? 1 : 0;
}

// FOV, HmdToEyePose and texture size of one eye, for the view_rotation_fix currently set in PVR. Needs pvrLock.
static void ReadPvrEye(pvrSessionHandle session, int e, PvrCache::Eye& eye){
	pvrEyeRenderInfo eyeInfo = {};
	pvr_getEyeRenderInfo(session, (pvrEyeType)e, &eyeInfo);
	pvrSizei textureSize = {};
	pvr_getFovTextureSize(session, (pvrEyeType)e, eyeInfo.Fov, 1.f, &textureSize);
	eye.fov.left = eyeInfo.Fov.LeftTan;
	eye.fov.right = eyeInfo.Fov.RightTan;
	eye.fov.up = eyeInfo.Fov.UpTan;
	eye.fov.down = eyeInfo.Fov.DownTan;
	eye.position[0] = eyeInfo.HmdToEyePose.Position.x;
	eye.position[1] = eyeInfo.HmdToEyePose.Position.y;
	eye.position[2] = eyeInfo.HmdToEyePose.Position.z;
	eye.orientation[0] = eyeInfo.HmdToEyePose.Orientation.x;
	eye.orientation[1] = eyeInfo.HmdToEyePose.Orientation.y;
	eye.orientation[2] = eyeInfo.HmdToEyePose.Orientation.z;
	eye.orientation[3] = eyeInfo.HmdToEyePose.Orientation.w;
	eye.textureWidth = textureSize.w;
	eye.textureHeight = textureSize.h;
}

// if the FOV, eye pose and texture size match (the file keeps ~6 significant digits)
static bool SameEyeValues(const PvrCache::Eye& a, const PvrCache::Eye& b){
	const auto similar = [](float x, float y){ return std::fabs(x - y) <= 1e-4f; };
	bool same = similar(a.fov.left, b.fov.left) && similar(a.fov.right, b.fov.right)
		&& similar(a.fov.up, b.fov.up) && similar(a.fov.down, b.fov.down)
		&& a.textureWidth == b.textureWidth && a.textureHeight == b.textureHeight;
	for(int i = 0; i < 3; i++){
		same = same && similar(a.position[i], b.position[i]);
	}
	for(int i = 0; i < 4; i++){
		same = same && similar(a.orientation[i], b.orientation[i]);
	}
	return same;
}

// If the saved file already holds what PVR reports now: compares the cheap values (panel size, and
// each eye's FOV, eye pose and texture size for the view_rotation_fix that is set) without changing
// anything in PVR. The distortion grids aren't compared; they follow from the same lens and IPD.
static bool SavedPvrDataIsCurrent(pvrSessionHandle session, const std::string& path, const pvrHmdInfo& info){
	PvrCache::Data saved;
	std::string error;
	if(!PvrCache::Load(path, saved, error) || !PvrCache::CheckHeadset(saved, info.ProductId, info.SerialNumber).empty()){
		return false;
	}
	const int viewRotationFix = ConfiguredViewRotationFix();
	shared_lock_guard<std::shared_mutex> lock(PimaxCommon::pvrLock);
	if(pvr_getIntConfig(session, "view_rotation_fix", -1) != viewRotationFix){
		return false;
	}
	pvrDisplayInfo displayInfo = {};
	pvr_getEyeDisplayInfo(session, pvrEye_Left, &displayInfo);
	if(saved.productName != info.ProductName || saved.panelWidth != displayInfo.width || saved.panelHeight != displayInfo.height){
		return false;
	}
	for(int e = 0; e < 2; e++){
		PvrCache::Eye live;
		ReadPvrEye(session, e, live);
		if(!SameEyeValues(live, saved.eyes[viewRotationFix][e])){
			return false;
		}
	}
	return true;
}

// Reads what PimaxDistortionProfile takes from PVR, for both view_rotation_fix values, then sets the
// configured value (parallelProjection) again; a shim may have changed it meanwhile, which discards the
// read. Runs in the PVR thread, where RunPvrBackground also sets view_rotation_fix. It takes about a
// second (2 x 200 ms settle time and 4 x 65 x 65 samples), during which the PVR thread (buttons,
// proximity, IPD) waits and sPimaxCommonInstanceMutex stays locked, so creating or destroying a shim
// waits too. That happens once per PVR connection or IPD change, and only when the file is out of date.
static bool CapturePvrData(pvrSessionHandle session, PvrCache::Data& data, std::string& error){
	shared_lock_guard<std::shared_mutex> lock(PimaxCommon::pvrLock);
	pvrHmdInfo info = {};
	pvr_getHmdInfo(session, &info);
	pvrDisplayInfo displayInfo = {};
	pvr_getEyeDisplayInfo(session, pvrEye_Left, &displayInfo);
	data.productName = info.ProductName;
	data.productId = info.ProductId;
	data.serial = info.SerialNumber;
	data.captureTime = (int64_t)std::time(nullptr);
	data.panelWidth = displayInfo.width;
	data.panelHeight = displayInfo.height;
	data.grid = PvrCache::kDefaultGrid;
	const int n = data.grid;
	error.clear();
	for(int viewRotationFix = 0; viewRotationFix < 2 && error.empty(); viewRotationFix++){
		pvr_setIntConfig(session, "view_rotation_fix", viewRotationFix);
		// give pi_server time to apply it (as the capture tool that was tested on the 8K X did)
		SleepMs(200);
		for(int e = 0; e < 2; e++){
			PvrCache::Eye& eye = data.eyes[viewRotationFix][e];
			ReadPvrEye(session, e, eye);
			eye.failedSamples = 0;
			eye.uv.assign((size_t)n * n * 6, 0.f);
			for(int y = 0; y < n; y++){
				for(int x = 0; x < n; x++){
					pvrVector2f outUV[3] = {};
					const pvrVector2f inUV = { (float)x / (n - 1), (float)y / (n - 1) };
					if(pvr_getHmdDistortedUV(session, (pvrEyeType)e, inUV, outUV) != pvr_success){
						eye.failedSamples++;
					}
					float* sample = &eye.uv[((size_t)y * n + x) * 6];
					for(int c = 0; c < 3; c++){
						sample[c * 2] = outUV[c].x;
						sample[c * 2 + 1] = outUV[c].y;
					}
				}
			}
		}
		if(pvr_getIntConfig(session, "view_rotation_fix", -1) != viewRotationFix){
			error = "view_rotation_fix was changed while reading";
		}
	}
	// the configured value, not the one read before: the settings may have changed meanwhile
	pvr_setIntConfig(session, "view_rotation_fix", ConfiguredViewRotationFix());
	if(!error.empty()){
		return false;
	}
	error = PvrCache::Validate(data);
	return error.empty();
}

// Keeps data that was just saved, for the identified headset (when PVR goes away again)
static void KeepSavedPvrData(PvrCache::Data&& data){
	std::shared_ptr<const PvrCache::Data> entry = std::make_shared<const PvrCache::Data>(std::move(data));
	bool identified = false;
	{
		std::lock_guard<std::mutex> lock(s_savedDataLock);
		s_loadedData[entry->serial] = entry;
		identified = s_identified && entry->serial == s_identifiedSerial;
	}
	if(identified){
		if(const LoadedProfile* loaded = s_profile){
			// the profile stays in use; this is now the last known IPD
			s_fixedIpd = ClampToProfile(loaded->profile, PvrProfile::PvrIpd(*entry));
		}else{
			SetSavedData(entry);
		}
	}
}

enum class SavePvrDataResult {
	Saved,
	// up to date already, not wanted, or can't be saved for this headset in this connection
	Skipped,
	// retried later
	Failed,
};

// projectionChanged: set when view_rotation_fix was changed in PVR (and set back)
static SavePvrDataResult SavePvrData(const PimaxInfo& pimaxInfo, bool& projectionChanged){
	const Config::PimaxHeadsetConfig* config = dynamic_cast<Config::PimaxHeadsetConfig*>(driverConfig.ConfigFromHeadsetType(pimaxInfo.headsetType));
	if(!config || !config->savePvrData){
		return SavePvrDataResult::Skipped;
	}
	// connected, so this doesn't try to connect (or lock)
	const pvrSessionHandle session = PimaxCommon::GetPvrSession();
	if(!session){
		return SavePvrDataResult::Failed;
	}
	pvrHmdInfo info = {};
	{
		shared_lock_guard<std::shared_mutex> lock(PimaxCommon::pvrLock);
		pvr_getHmdInfo(session, &info);
	}
	if(info.ProductId != kUsbProductId){
		// only P2 headsets are identified without PVR
		return SavePvrDataResult::Skipped;
	}
	const std::string path = SavedPvrDataPath(info.SerialNumber);
	if(path.empty()){
		DriverLog("Not saving PVR data: the serial number '%s' can't be used in a file name", MaskSerial(info.SerialNumber).c_str());
		return SavePvrDataResult::Skipped;
	}
	if(SavedPvrDataIsCurrent(session, path, info)){
		DriverLog("Saved PVR data for serial %s is up to date", MaskSerial(info.SerialNumber).c_str());
		return SavePvrDataResult::Skipped;
	}
	// Reading it switches view_rotation_fix in PVR for a moment. Only do that while SteamVR draws
	// to the headset through the Lighthouse shim, which sets view_rotation_fix (checked by the caller),
	// and not while PVR renders to it itself (no_render off).
	if(!pimaxInfo.isOpenPortEnabled){
		DriverLog("Not saving PVR data: PVR renders to the headset itself (no_render is off)");
		return SavePvrDataResult::Skipped;
	}
	const auto start = std::chrono::steady_clock::now();
	PvrCache::Data data;
	std::string error;
	projectionChanged = true;
	if(!CapturePvrData(session, data, error)){
		DriverLog("Could not read PVR data to save: %s", error.c_str());
		return SavePvrDataResult::Failed;
	}
	if(!PvrCache::Save(path, data, error)){
		DriverLog("Could not save PVR data: %s", error.c_str());
		return SavePvrDataResult::Failed;
	}
	const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
	DriverLog("Saved PVR data for '%s' (serial %s) in %.1f s: panel %dx%d, IPD %.1f mm, canting %.2f deg", data.productName.c_str(),
		MaskSerial(data.serial).c_str(), seconds, data.panelWidth, data.panelHeight, data.Ipd(), data.CantingAngle());
	KeepSavedPvrData(std::move(data));
	return SavePvrDataResult::Saved;
}

// Saves PVR data (savePvrData) once per PVR connection, and again when the IPD changes, unless the
// saved file is up to date. Waits for the headset to settle after connecting and for an IPD change to finish.
class PvrDataSaver {
public:
	// returns true if view_rotation_fix was changed in PVR (and set back) to read the data
	bool Run(bool lighthouseShimActive){
		const PimaxInfo info = PimaxCommon::GetInfo();
		if(info.pvrConnected){
			// the display power pauses while PVR is connected
			NotePvrConnected();
		}
		if(!info.pvrConnected || !IsP2HeadsetType(info.headsetType)){
			*this = {};
			return false;
		}
		const float ipd = info.ipd;
		const auto now = std::chrono::steady_clock::now();
		if(!connectedSince){
			connectedSince = now;
			lastIpd = ipd;
			ipdChangedAt = now;
		}
		if(std::fabs(ipd - lastIpd) > 0.05f){
			lastIpd = ipd;
			ipdChangedAt = now;
		}
		const bool due = !done || std::fabs(ipd - doneIpd) > 0.2f;
		if(!due || now - *connectedSince < std::chrono::seconds(15) || now - ipdChangedAt < std::chrono::seconds(5)){
			return false;
		}
		if(lastAttempt && now - *lastAttempt < std::chrono::seconds(60)){
			return false;
		}
		if(!lighthouseShimActive){
			// there is one shim per HMD-class device; only the active one saves, and without using up the
			// attempt below
			if(!loggedDeferred){
				DriverLog("PVR data will be saved once the headset runs on Lighthouse in SteamVR");
				loggedDeferred = true;
			}
			return false;
		}
		lastAttempt = now;
		bool projectionChanged = false;
		const SavePvrDataResult result = SavePvrData(info, projectionChanged);
		if(result == SavePvrDataResult::Saved || result == SavePvrDataResult::Skipped){
			done = true;
			doneIpd = ipd;
		}
		return projectionChanged;
	}
private:
	std::optional<std::chrono::steady_clock::time_point> connectedSince;
	std::optional<std::chrono::steady_clock::time_point> lastAttempt;
	std::chrono::steady_clock::time_point ipdChangedAt;
	float lastIpd = 0;
	// saved (or nothing to save) for doneIpd
	bool done = false;
	float doneIpd = 0;
	bool loggedDeferred = false;
};

bool RunPvrDataSaver(bool lighthouseShimActive){
	// only the PVR thread calls this
	static PvrDataSaver s_saver;
	return s_saver.Run(lighthouseShimActive);
}
#endif

}
