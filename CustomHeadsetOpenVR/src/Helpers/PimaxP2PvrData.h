#pragma once
// Pimax P2 headsets (5K/8K series, Artisan) without PVR: when pi_server is closed (eg: a legacy launch on
// Lighthouse), the headset is identified over USB (PimaxP2::Identify) and PVR's data (FOV, distortion,
// render size) comes from the data saved while PVR was connected (PvrCache, <config>/pvr-data-<serial>.json).
// While PVR isn't connected to a headset identified this way, the Saved* functions answer instead of PVR.
// Other headsets are never identified here, so for them every function returns false at once.
#include "PimaxCommon.h"

#ifdef PVR_EXISTS
#include <openvr_driver.h>
#endif

namespace PimaxP2 {
	// For PimaxCommon::TryDirectConnection; does nothing if PimaxDirectUSB or PVR found a headset. Identifies
	// a P2 headset over USB and fills info (type, resolution, and IPD and canting from the saved data),
	// setting connected last. The headset is claimed only if its model is known (from the USB product
	// string, or from the saved data), its headset type is enabled and its panel size is known; otherwise
	// info is left untouched and the reason is logged once. Nothing is written if PVR connected meanwhile.
	bool IdentifyWithoutPvr(PimaxInfo& info);

#ifdef PVR_EXISTS
	// For the shim's RunFrame, with a headset identified by IdentifyWithoutPvr. PvrTookOver: true once each
	// time PVR connects (its values are written by then); from then on the Saved* functions return false
	// and PVR is used. PvrWentAway: true once each time PVR is lost again; the Saved* functions answer
	// again. The caller rebuilds what it built from the other source.
	bool PvrTookOver();
	bool PvrWentAway();

	// Instead of PVR while a headset identified without PVR has no PVR: the saved values for the
	// view_rotation_fix that parallelProjection selects. Without a saved file: tangent 1 (90 degrees),
	// identity UVs (no distortion) and the panel size. They return false (use PVR) otherwise.
	bool SavedEyeFov(vr::EVREye eye, pvrFovPort& fov);
	bool SavedDistortedUV(vr::EVREye eye, pvrVector2f uv, pvrVector2f outUV[3]);
	bool SavedFovTextureSize(pvrSizei& size);

	// For PimaxLighthouseShim::RunPvrBackground (PVR thread). P2 headsets only (returns at once for others),
	// if savePvrData is on: saves PVR's data (PvrCache) once per PVR connection, 15 s after it connects,
	// and again after an IPD change, unless the saved file is up to date. Reading it switches
	// view_rotation_fix in PVR for a moment, so that is done only while PVR doesn't render to the headset
	// (no_render on) and the Lighthouse shim drives it (lighthouseShimActive: only the active shim's
	// call saves). Saved data is used for the identified headset from then on. Returns true if
	// view_rotation_fix was changed (and set back): a distortion built from PVR meanwhile may have used
	// the other projection.
	bool RunPvrDataSaver(bool lighthouseShimActive);
#endif
}
