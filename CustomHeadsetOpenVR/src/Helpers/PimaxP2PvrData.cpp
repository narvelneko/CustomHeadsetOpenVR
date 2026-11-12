#include "PimaxP2PvrData.h"

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
	std::lock_guard<std::mutex> lock(s_savedDataLock);
	std::unique_ptr<const PvrCache::Data>& entry = s_loadedData[data.serial];
	// the Saved* functions may still be reading the previous data, so it is kept
	static std::vector<std::unique_ptr<const PvrCache::Data>> s_replaced;
	if(entry){
		s_replaced.push_back(std::move(entry));
	}
	entry = std::make_unique<const PvrCache::Data>(std::move(data));
	if(s_identified && entry->serial == s_identifiedSerial){
		s_savedData = entry.get();
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
