#pragma once
// Saved PVR data for Pimax P2 headsets (5K/8K series, Artisan), for running them on Lighthouse while
// pi_server is closed and PVR gives no field of view, eye poses or distortion. The driver saves it
// while PVR is connected, one file per headset serial number in the config folder.
//
// Self-contained (no driver globals, no PVR SDK) so a standalone tool can read and write it too.
// File (JSON, version 1):
//   format, version, productName, productId, serial, captureTime (unix seconds), captureTimeUtc,
//   panel [width, height] (both eyes), grid (samples per side),
//   viewRotationFix [0, 1]: { eyes [left, right]: {
//     fov {left, right, up, down} (tangents), hmdToEye {position [x,y,z] (m), orientation [x,y,z,w]},
//     textureSize [w, h] (pvr_getFovTextureSize, density 1), failedSamples,
//     uv: grid*grid samples of pvr_getHmdDistortedUV, row-major (v outer, u inner, both 0..1),
//         6 numbers each: red u, v, green u, v, blue u, v } }
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace PvrCache {
	constexpr int kVersion = 1;
	// the lens function is smooth; 65 samples per side keeps the file around 1 MB
	constexpr int kDefaultGrid = 65;

	struct Fov {
		// tangents of the half angles
		float left = 0, right = 0, up = 0, down = 0;
	};

	struct Eye {
		Fov fov;
		// PVR HmdToEyePose
		float position[3] = {};
		float orientation[4] = {0, 0, 0, 1}; // x y z w
		int textureWidth = 0, textureHeight = 0;
		// samples where pvr_getHmdDistortedUV failed; the data is only usable when 0
		int failedSamples = 0;
		// grid * grid * 6, see the file layout above
		std::vector<float> uv;
	};

	struct Data {
		std::string productName;
		int productId = 0;
		std::string serial;
		// unix time (seconds)
		int64_t captureTime = 0;
		// both eyes
		int panelWidth = 0, panelHeight = 0;
		int grid = 0;
		// [view_rotation_fix][eye]: view_rotation_fix 0 is canted, 1 parallel projection
		Eye eyes[2][2];

		// distance between the eyes in mm (view_rotation_fix 0)
		float Ipd() const;
		// half the angle between the eyes in degrees (view_rotation_fix 0)
		float CantingAngle() const;
		// bilinear lookup of pvr_getHmdDistortedUV(u, v) for one channel (0 red, 1 green, 2 blue).
		// u and v are clamped to 0..1 (NaN counts as 0). False if an index is out of range or there is no grid.
		bool DistortedUV(int viewRotationFix, int eye, int channel, float u, float v, float& outU, float& outV) const;
		// "YYYY-MM-DD HH:MM UTC"
		std::string CaptureTimeString() const;
		// whole days since the capture
		int AgeDays() const;
	};

	// file name (no folder) for a headset serial number; empty if the serial can't be used in one
	std::string FileName(std::string_view serial);

	// why the data can't be used, or empty if it can: panel and texture sizes non-zero, all values
	// finite, no failed samples, complete grids
	std::string Validate(const Data& data);
	// why the data doesn't belong to this headset, or empty if it does
	std::string CheckHeadset(const Data& data, int productId, std::string_view serial);

	// writes the file (through a temporary file, so a reader never sees half of it)
	bool Save(const std::string& path, const Data& data, std::string& error);
	// reads and validates the file; on failure error says why
	bool Load(const std::string& path, Data& data, std::string& error);
}
