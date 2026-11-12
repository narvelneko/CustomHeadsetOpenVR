#include "PvrProfile.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include "nlohmann/json.hpp"

namespace PvrProfile {

using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;

static const char* kFormat = "chovr-pvr-profile-1";

float Profile::MinIpd() const{
	return captures.empty() ? 0.f : captures.front().pvrIpd;
}

float Profile::MaxIpd() const{
	return captures.empty() ? 0.f : captures.back().pvrIpd;
}

std::string FileName(std::string_view serial){
	// the same rules as PvrCache's file name
	const std::string dataName = PvrCache::FileName(serial);
	return dataName.empty() ? "" : "pvr-profile-" + std::string(serial) + ".json";
}

float PvrIpd(const PvrCache::Data& data){
	return data.Ipd() / 1000.f;
}

void Normalize(Profile& profile){
	std::stable_sort(profile.captures.begin(), profile.captures.end(), [](const Capture& a, const Capture& b){ return a.pvrIpd < b.pvrIpd; });
	for(Capture& capture : profile.captures){
		capture.data.productName = profile.productName;
		capture.data.productId = profile.productId;
		capture.data.serial = profile.serial;
		capture.data.panelWidth = profile.panelWidth;
		capture.data.panelHeight = profile.panelHeight;
		capture.data.grid = profile.grid;
	}
}

std::string CheckBounds(const PvrCache::Data& data){
	if(data.panelWidth > kMaxSize || data.panelHeight > kMaxSize){
		return "the panel size " + std::to_string(data.panelWidth) + "x" + std::to_string(data.panelHeight) + " is too large";
	}
	for(int vrf = 0; vrf < 2; vrf++){
		for(int e = 0; e < 2; e++){
			const PvrCache::Eye& eye = data.eyes[vrf][e];
			if(eye.textureWidth > kMaxSize || eye.textureHeight > kMaxSize){
				return "the texture size " + std::to_string(eye.textureWidth) + "x" + std::to_string(eye.textureHeight) + " is too large";
			}
		}
	}
	return "";
}

std::string Validate(const Profile& profile){
	if(profile.captures.empty()){
		return "it has no captures";
	}
	if(profile.captures.size() > (size_t)kMaxCaptures){
		return "it has " + std::to_string(profile.captures.size()) + " captures, more than " + std::to_string(kMaxCaptures);
	}
	float previousIpd = 0;
	for(size_t i = 0; i < profile.captures.size(); i++){
		const Capture& capture = profile.captures[i];
		const PvrCache::Data& data = capture.data;
		const std::string where = "capture " + std::to_string(i + 1);
		if(data.productName != profile.productName || data.productId != profile.productId || data.serial != profile.serial
			|| data.panelWidth != profile.panelWidth || data.panelHeight != profile.panelHeight || data.grid != profile.grid){
			return where + ": doesn't match the profile's headset, panel or grid";
		}
		std::string error = PvrCache::Validate(data);
		if(error.empty()){
			error = CheckBounds(data);
		}
		if(!error.empty()){
			return where + ": " + error;
		}
		if(!(capture.pvrIpd >= kMinPvrIpd && capture.pvrIpd <= kMaxPvrIpd)){
			return where + ": its pvrIpd is outside 40-90 mm";
		}
		if(std::fabs(capture.pvrIpd - PvrIpd(data)) > 1e-4f){
			return where + ": its pvrIpd doesn't match its eye poses";
		}
		if(capture.hidIpdRaw < -1 || capture.hidIpdRaw > 0xFFFF){
			return where + ": hidIpdRaw is out of range";
		}
		if(i > 0 && !(capture.pvrIpd > previousIpd)){
			return where + ": the captures aren't sorted by IPD (or two have the same IPD)";
		}
		previousIpd = capture.pvrIpd;
	}
	return "";
}

std::string CheckHeadset(const Profile& profile, int productId, std::string_view serial){
	if(profile.productId != productId){
		char text[96] = {};
		std::snprintf(text, sizeof(text), "it is for product id %04x, the headset is %04x", profile.productId, productId);
		return text;
	}
	if(serial.empty() || profile.serial != serial){
		return "it is for another serial number";
	}
	return "";
}

static const char* kBase64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// little-endian float32, base64
static std::string EncodeFloats(const std::vector<float>& values){
	std::string bytes(values.size() * 4, '\0');
	for(size_t i = 0; i < values.size(); i++){
		uint32_t bits = 0;
		std::memcpy(&bits, &values[i], 4);
		for(int b = 0; b < 4; b++){
			bytes[i * 4 + b] = (char)((bits >> (8 * b)) & 0xFF);
		}
	}
	std::string text;
	text.reserve((bytes.size() + 2) / 3 * 4);
	for(size_t i = 0; i < bytes.size(); i += 3){
		const size_t left = bytes.size() - i;
		const uint32_t chunk = ((uint32_t)(unsigned char)bytes[i] << 16)
			| (left > 1 ? (uint32_t)(unsigned char)bytes[i + 1] << 8 : 0)
			| (left > 2 ? (uint32_t)(unsigned char)bytes[i + 2] : 0);
		text += kBase64[(chunk >> 18) & 63];
		text += kBase64[(chunk >> 12) & 63];
		text += left > 1 ? kBase64[(chunk >> 6) & 63] : '=';
		text += left > 2 ? kBase64[chunk & 63] : '=';
	}
	return text;
}

static std::vector<float> DecodeFloats(const std::string& text){
	int value[256];
	for(int& v : value){
		v = -1;
	}
	for(int i = 0; i < 64; i++){
		value[(unsigned char)kBase64[i]] = i;
	}
	if(text.size() % 4 != 0){
		throw std::runtime_error("uv is not base64");
	}
	std::string bytes;
	bytes.reserve(text.size() / 4 * 3);
	for(size_t i = 0; i < text.size(); i += 4){
		uint32_t chunk = 0;
		int padding = 0;
		for(int k = 0; k < 4; k++){
			const char c = text[i + k];
			if(c == '=' && i + 4 == text.size() && k >= 2){
				padding++;
				chunk <<= 6;
				continue;
			}
			const int v = value[(unsigned char)c];
			if(v < 0 || padding > 0){
				throw std::runtime_error("uv is not base64");
			}
			chunk = (chunk << 6) | (uint32_t)v;
		}
		bytes += (char)((chunk >> 16) & 0xFF);
		if(padding < 2){
			bytes += (char)((chunk >> 8) & 0xFF);
		}
		if(padding < 1){
			bytes += (char)(chunk & 0xFF);
		}
	}
	if(bytes.size() % 4 != 0){
		throw std::runtime_error("uv is not a whole number of float32 values");
	}
	std::vector<float> values(bytes.size() / 4);
	for(size_t i = 0; i < values.size(); i++){
		uint32_t bits = 0;
		for(int b = 0; b < 4; b++){
			bits |= (uint32_t)(unsigned char)bytes[i * 4 + b] << (8 * b);
		}
		std::memcpy(&values[i], &bits, 4);
	}
	return values;
}

bool Save(const std::string& path, const Profile& profile, std::string& error){
	error = Validate(profile);
	if(!error.empty()){
		return false;
	}
	try{
		ordered_json captures = ordered_json::array();
		for(const Capture& capture : profile.captures){
			const PvrCache::Data& data = capture.data;
			ordered_json projections = ordered_json::array();
			for(int vrf = 0; vrf < 2; vrf++){
				ordered_json eyes = ordered_json::array();
				for(int e = 0; e < 2; e++){
					const PvrCache::Eye& eye = data.eyes[vrf][e];
					eyes.push_back({
						{"fov", {{"left", eye.fov.left}, {"right", eye.fov.right}, {"up", eye.fov.up}, {"down", eye.fov.down}}},
						{"hmdToEye", {
							{"position", {eye.position[0], eye.position[1], eye.position[2]}},
							{"orientation", {eye.orientation[0], eye.orientation[1], eye.orientation[2], eye.orientation[3]}},
						}},
						{"textureSize", {eye.textureWidth, eye.textureHeight}},
						{"failedSamples", eye.failedSamples},
						{"uv", EncodeFloats(eye.uv)},
					});
				}
				projections.push_back({{"eyes", eyes}});
			}
			captures.push_back({
				{"pvrIpd", capture.pvrIpd},
				{"hidIpdRaw", capture.hidIpdRaw >= 0 ? ordered_json(capture.hidIpdRaw) : ordered_json(nullptr)},
				{"captureTime", data.captureTime},
				{"captureTimeUtc", data.CaptureTimeString()},
				{"viewRotationFix", projections},
			});
		}
		const ordered_json file = {
			{"format", kFormat},
			{"productName", profile.productName},
			{"productId", profile.productId},
			{"serial", profile.serial},
			{"panel", {profile.panelWidth, profile.panelHeight}},
			{"grid", profile.grid},
			{"captures", captures},
		};
		const std::string temporaryPath = path + ".tmp";
		{
			std::ofstream out(temporaryPath, std::ios::binary | std::ios::trunc);
			if(!out){
				error = "cannot write " + temporaryPath;
				return false;
			}
			out << file.dump() << "\n";
			if(!out){
				error = "cannot write " + temporaryPath;
				return false;
			}
		}
		std::error_code renameError;
		std::filesystem::rename(temporaryPath, path, renameError);
		if(renameError){
			error = "cannot replace " + path + ": " + renameError.message();
			std::filesystem::remove(temporaryPath, renameError);
			return false;
		}
	}catch(const std::exception& e){
		error = e.what();
		return false;
	}
	return true;
}

static void ReadFloats(const json& array, float* out, size_t count){
	if(!array.is_array() || array.size() != count){
		throw std::runtime_error("expected " + std::to_string(count) + " numbers");
	}
	for(size_t i = 0; i < count; i++){
		out[i] = array.at(i).get<float>();
	}
}

bool Load(const std::string& path, Profile& profile, std::string& error){
	std::ifstream in(path, std::ios::binary);
	if(!in){
		error = "no file";
		return false;
	}
	std::error_code sizeError;
	const uintmax_t size = std::filesystem::file_size(path, sizeError);
	if(sizeError || size > kMaxFileBytes){
		error = sizeError ? "cannot read its size" : "the file is larger than 64 MB";
		return false;
	}
	Profile result;
	try{
		const json file = json::parse(in);
		if(file.value("format", "") != kFormat){
			error = "unknown format";
			return false;
		}
		result.productName = file.at("productName").get<std::string>();
		result.productId = file.at("productId").get<int>();
		result.serial = file.at("serial").get<std::string>();
		result.panelWidth = file.at("panel").at(0).get<int>();
		result.panelHeight = file.at("panel").at(1).get<int>();
		result.grid = file.at("grid").get<int>();
		const json& captures = file.at("captures");
		if(!captures.is_array() || captures.size() > (size_t)kMaxCaptures){
			error = "it has more than " + std::to_string(kMaxCaptures) + " captures";
			return false;
		}
		for(const json& source : captures){
			Capture capture;
			capture.pvrIpd = source.at("pvrIpd").get<float>();
			const json& raw = source.at("hidIpdRaw");
			capture.hidIpdRaw = raw.is_null() ? -1 : raw.get<int>();
			PvrCache::Data& data = capture.data;
			data.captureTime = source.at("captureTime").get<int64_t>();
			const json& projections = source.at("viewRotationFix");
			for(int vrf = 0; vrf < 2; vrf++){
				for(int e = 0; e < 2; e++){
					const json& eyeSource = projections.at(vrf).at("eyes").at(e);
					PvrCache::Eye& eye = data.eyes[vrf][e];
					const json& fov = eyeSource.at("fov");
					eye.fov.left = fov.at("left").get<float>();
					eye.fov.right = fov.at("right").get<float>();
					eye.fov.up = fov.at("up").get<float>();
					eye.fov.down = fov.at("down").get<float>();
					ReadFloats(eyeSource.at("hmdToEye").at("position"), eye.position, 3);
					ReadFloats(eyeSource.at("hmdToEye").at("orientation"), eye.orientation, 4);
					eye.textureWidth = eyeSource.at("textureSize").at(0).get<int>();
					eye.textureHeight = eyeSource.at("textureSize").at(1).get<int>();
					eye.failedSamples = eyeSource.at("failedSamples").get<int>();
					eye.uv = DecodeFloats(eyeSource.at("uv").get<std::string>());
				}
			}
			result.captures.push_back(std::move(capture));
		}
	}catch(const std::exception& e){
		error = std::string("cannot read it: ") + e.what();
		return false;
	}
	// fill in the profile's fields without reordering: an unsorted file is rejected
	for(Capture& capture : result.captures){
		capture.data.productName = result.productName;
		capture.data.productId = result.productId;
		capture.data.serial = result.serial;
		capture.data.panelWidth = result.panelWidth;
		capture.data.panelHeight = result.panelHeight;
		capture.data.grid = result.grid;
	}
	error = Validate(result);
	if(!error.empty()){
		return false;
	}
	profile = std::move(result);
	return true;
}

static float Lerp(float a, float b, float t){
	return a + (b - a) * t;
}

PvrCache::Data Blend(const Profile& profile, float ipd){
	const std::vector<Capture>& captures = profile.captures;
	if(captures.size() == 1 || !(ipd > captures.front().pvrIpd)){
		return captures.front().data;
	}
	if(ipd >= captures.back().pvrIpd){
		return captures.back().data;
	}
	size_t upper = 1;
	while(captures[upper].pvrIpd < ipd){
		upper++;
	}
	const Capture& a = captures[upper - 1];
	const Capture& b = captures[upper];
	const float t = (ipd - a.pvrIpd) / (b.pvrIpd - a.pvrIpd);
	PvrCache::Data result = a.data;
	result.captureTime = std::max(a.data.captureTime, b.data.captureTime);
	for(int vrf = 0; vrf < 2; vrf++){
		for(int e = 0; e < 2; e++){
			const PvrCache::Eye& ea = a.data.eyes[vrf][e];
			const PvrCache::Eye& eb = b.data.eyes[vrf][e];
			PvrCache::Eye& out = result.eyes[vrf][e];
			out.fov.left = Lerp(ea.fov.left, eb.fov.left, t);
			out.fov.right = Lerp(ea.fov.right, eb.fov.right, t);
			out.fov.up = Lerp(ea.fov.up, eb.fov.up, t);
			out.fov.down = Lerp(ea.fov.down, eb.fov.down, t);
			for(int i = 0; i < 3; i++){
				out.position[i] = Lerp(ea.position[i], eb.position[i], t);
			}
			// normalized lerp, on the same side of the quaternion double cover
			float dot = 0;
			for(int i = 0; i < 4; i++){
				dot += ea.orientation[i] * eb.orientation[i];
			}
			const float sign = dot < 0.f ? -1.f : 1.f;
			float length = 0;
			for(int i = 0; i < 4; i++){
				out.orientation[i] = Lerp(ea.orientation[i], sign * eb.orientation[i], t);
				length += out.orientation[i] * out.orientation[i];
			}
			length = std::sqrt(length);
			if(length > 0.f){
				for(int i = 0; i < 4; i++){
					out.orientation[i] /= length;
				}
			}
			out.textureWidth = (int)std::lround(Lerp((float)ea.textureWidth, (float)eb.textureWidth, t));
			out.textureHeight = (int)std::lround(Lerp((float)ea.textureHeight, (float)eb.textureHeight, t));
			out.failedSamples = 0;
			for(size_t i = 0; i < out.uv.size(); i++){
				out.uv[i] = Lerp(ea.uv[i], eb.uv[i], t);
			}
		}
	}
	return result;
}

const Capture& Nearest(const Profile& profile, float ipd){
	const Capture* nearest = &profile.captures.front();
	for(const Capture& capture : profile.captures){
		if(std::fabs(capture.pvrIpd - ipd) < std::fabs(nearest->pvrIpd - ipd)){
			nearest = &capture;
		}
	}
	return *nearest;
}

float LargestGap(const Profile& profile){
	float gap = 0;
	for(size_t i = 1; i < profile.captures.size(); i++){
		gap = std::max(gap, profile.captures[i].pvrIpd - profile.captures[i - 1].pvrIpd);
	}
	return gap;
}

float HidFormulaIpd(uint16_t raw){
	const float separation = ((raw & 0x8000) ? ((raw & 0x7fff) * -0.168f + 4960.f) : (float)raw) / 50000.f;
	return separation - 0.026f;
}

HidCalibration MakeHidCalibration(const Profile& profile){
	std::vector<std::pair<float, float>> points;
	for(const Capture& capture : profile.captures){
		if(capture.hidIpdRaw >= 0){
			points.emplace_back(HidFormulaIpd((uint16_t)capture.hidIpdRaw), capture.pvrIpd);
		}
	}
	std::sort(points.begin(), points.end());
	HidCalibration calibration;
	std::vector<std::pair<float, float>>& merged = calibration.points;
	for(size_t i = 0; i < points.size();){
		size_t j = i;
		float sum = 0;
		while(j < points.size() && points[j].first == points[i].first){
			sum += points[j].second;
			j++;
		}
		merged.emplace_back(points[i].first, sum / (float)(j - i));
		i = j;
	}
	return calibration;
}

bool CalibratedIpd(const HidCalibration& calibration, uint16_t raw, float& ipd){
	const std::vector<std::pair<float, float>>& merged = calibration.points;
	if(merged.empty()){
		return false;
	}
	const float x = HidFormulaIpd(raw);
	if(x <= merged.front().first){
		ipd = merged.front().second + (x - merged.front().first);
		return true;
	}
	if(x >= merged.back().first){
		ipd = merged.back().second + (x - merged.back().first);
		return true;
	}
	size_t upper = 1;
	while(merged[upper].first < x){
		upper++;
	}
	const auto& a = merged[upper - 1];
	const auto& b = merged[upper];
	ipd = Lerp(a.second, b.second, (x - a.first) / (b.first - a.first));
	return true;
}

bool CalibratedIpd(const Profile& profile, uint16_t raw, float& ipd){
	return CalibratedIpd(MakeHidCalibration(profile), raw, ipd);
}

bool IpdFilter::Update(float ipd, double seconds, float applied, float minIpd, float maxIpd, float& result){
	samples.push_back(ipd);
	if(samples.size() > 3){
		samples.erase(samples.begin());
	}
	std::vector<float> sorted = samples;
	std::sort(sorted.begin(), sorted.end());
	float filtered = sorted[sorted.size() / 2];
	filtered = filtered < minIpd ? minIpd : filtered > maxIpd ? maxIpd : filtered;
	if(std::fabs(filtered - applied) < 0.0002f){
		pending = false;
		return false;
	}
	if(!pending || std::fabs(filtered - pendingIpd) > 0.0001f){
		pending = true;
		pendingSince = seconds;
		pendingIpd = filtered;
		return false;
	}
	if(seconds - pendingSince < 0.9){
		return false;
	}
	pending = false;
	result = filtered;
	return true;
}

void IpdFilter::Hold(){
	pending = false;
}

void IpdFilter::Reset(){
	samples.clear();
	pending = false;
}

}
