#pragma once
// IPD-swept PVR data for Pimax P2 headsets (5K/8K series, Artisan). PVR's lens correction depends
// strongly on the IPD (one capture is ~13 px off one IPD notch away), so a profile holds several
// captures (PvrCache::Data each) at different IPDs, and the driver blends the two around the current
// IPD. The pvrcache tool sweeps the IPD to write it; the single capture (PvrCache) stays as it is.
//
// Self-contained (no driver globals, no PVR SDK) like PvrCache, so tools and tests can use it too.
// File (JSON): <config folder>/pvr-profile-<serial>.json
//   format "chovr-pvr-profile-1", productName, productId, serial, panel [width, height] (both eyes), grid,
//   captures: sorted by pvrIpd, strictly ascending, at least one: {
//     pvrIpd (m, distance between PVR's eye positions, view_rotation_fix 0),
//     hidIpdRaw (uint16 the headset reports in HID feature report 0xF0, bytes 36-37 LE; null if unknown),
//     captureTime (unix seconds), captureTimeUtc,
//     viewRotationFix [0, 1]: { eyes [left, right]: {
//       fov {left, right, up, down}, hmdToEye {position, orientation}, textureSize [w, h], failedSamples,
//       uv: base64 of grid*grid*6 little-endian float32, laid out as in PvrCache } } }
// The UVs are stored as float32 in base64 (exact, ~540 KB per capture at grid 65) rather than as JSON
// numbers (~900 KB at 6 digits), which also keeps loading fast.
#include "PvrCache.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace PvrProfile {
	// bounds a profile must stay within (a damaged or hostile file is refused before it is used)
	constexpr uintmax_t kMaxFileBytes = 64u << 20;
	constexpr int kMaxCaptures = 64;
	// panel and texture sizes
	constexpr int kMaxSize = 16384;
	// metres
	constexpr float kMinPvrIpd = 0.040f, kMaxPvrIpd = 0.090f;

	struct Capture {
		// metres; the same as PvrIpd(data)
		float pvrIpd = 0;
		// raw IPD value from the headset's HID feature report 0xF0, or -1 if unknown
		int hidIpdRaw = -1;
		// productName, productId, serial, panel size and grid are the profile's (Normalize)
		PvrCache::Data data;
	};

	struct Profile {
		std::string productName;
		int productId = 0;
		std::string serial;
		// both eyes
		int panelWidth = 0, panelHeight = 0;
		int grid = 0;
		std::vector<Capture> captures;

		// metres; 0 without captures
		float MinIpd() const;
		float MaxIpd() const;
	};

	// file name (no folder) for a headset serial number; empty if the serial can't be used in one
	std::string FileName(std::string_view serial);

	// Sorts the captures by pvrIpd and copies the profile's product, serial, panel size and grid into them
	void Normalize(Profile& profile);
	// why the data's panel or texture sizes exceed kMaxSize, or empty (for single PvrCache captures too)
	std::string CheckBounds(const PvrCache::Data& data);
	// why the profile can't be used, or empty if it can: each capture as PvrCache::Validate and
	// CheckBounds, consistent with the profile, pvrIpd matching its eye poses and within
	// kMinPvrIpd-kMaxPvrIpd, sorted strictly ascending, 1 to kMaxCaptures captures
	std::string Validate(const Profile& profile);
	// why the profile doesn't belong to this headset, or empty if it does
	std::string CheckHeadset(const Profile& profile, int productId, std::string_view serial);

	// writes the file (through a temporary file); the profile must be valid
	bool Save(const std::string& path, const Profile& profile, std::string& error);
	// reads and validates the file (refused unread above kMaxFileBytes); on failure error says why
	// ("no file" if there is none)
	bool Load(const std::string& path, Profile& profile, std::string& error);

	// PVR's IPD in metres for a capture: distance between the eye positions (view_rotation_fix 0)
	float PvrIpd(const PvrCache::Data& data);

	// The data for an IPD (metres) from the two captures around it, linear in the IPD: UVs (all channels),
	// FOV tangents, eye positions and texture sizes (rounded); eye orientations are normalized-lerped.
	// Clamped to the captured range. The profile must be valid.
	PvrCache::Data Blend(const Profile& profile, float ipd);
	// the capture whose pvrIpd is nearest to ipd (metres); the profile must be valid
	const Capture& Nearest(const Profile& profile, float ipd);
	// the largest IPD gap between neighbouring captures (metres); 0 with one capture
	float LargestGap(const Profile& profile);

	// Monado's conversion of the raw HID IPD value to metres (separation of the lenses minus 26 mm)
	float HidFormulaIpd(uint16_t raw);
	// The profile's (HidFormulaIpd(raw), pvrIpd) pairs of the captures that have a raw value, sorted,
	// equal formula values averaged: made once, used for every reading
	struct HidCalibration {
		std::vector<std::pair<float, float>> points;
	};
	HidCalibration MakeHidCalibration(const Profile& profile);
	// The PVR IPD (metres) for a raw HID value: piecewise linear in HidFormulaIpd(raw) between the
	// calibration points, and outside them the nearest one's offset. False (ipd untouched) without points.
	bool CalibratedIpd(const HidCalibration& calibration, uint16_t raw, float& ipd);
	// the same, making the calibration for this call
	bool CalibratedIpd(const Profile& profile, uint16_t raw, float& ipd);

	// When to apply a new IPD, from readings several times a second. The median of the last 3 readings,
	// clamped to the profile's range, is followed while the dial turns: applied when it is 0.2 mm or more
	// from the applied IPD, at most every 0.5 s. Once it has stayed within 0.1 mm for 0.5 s it is applied
	// if it is 0.1 mm or more away, so the final IPD is exact. Jitter of a few raw counts never applies.
	class IpdFilter {
	public:
		// a reading (metres) at a time (seconds); true with the IPD to apply in result
		bool Update(float ipd, double seconds, float applied, float minIpd, float maxIpd, float& result);
		// no reading this time (the IPD is held): it has to be stable again before it settles
		void Hold();
		// forget the readings too (another source, or PVR took over)
		void Reset();
	private:
		std::vector<float> samples;
		bool tracking = false;
		float lastFiltered = 0;
		double stableSince = 0;
		bool appliedBefore = false;
		double lastApply = 0;
	};
}
