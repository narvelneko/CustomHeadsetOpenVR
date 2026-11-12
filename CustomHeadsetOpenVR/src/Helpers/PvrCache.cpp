#include "PvrCache.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include "nlohmann/json.hpp"

namespace PvrCache {

using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;

static const char* kFormat = "CustomHeadsetOpenVR saved PVR data";

float Data::Ipd() const{
	const Eye& left = eyes[0][0];
	const Eye& right = eyes[0][1];
	const float dx = left.position[0] - right.position[0];
	const float dy = left.position[1] - right.position[1];
	const float dz = left.position[2] - right.position[2];
	return std::sqrt(dx * dx + dy * dy + dz * dz) * 1000.f;
}

float Data::CantingAngle() const{
	// the angle between two rotations is 2 * acos(|dot|); half of it per eye
	const float* a = eyes[0][0].orientation;
	const float* b = eyes[0][1].orientation;
	float dot = std::fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
	dot = dot > 1.f ? 1.f : dot;
	return std::acos(dot) * 180.f / 3.1415926f;
}

bool Data::DistortedUV(int viewRotationFix, int eye, int channel, float u, float v, float& outU, float& outV) const{
	if(viewRotationFix < 0 || viewRotationFix > 1 || eye < 0 || eye > 1 || channel < 0 || channel > 2 || grid < 2){
		return false;
	}
	const std::vector<float>& uv = eyes[viewRotationFix][eye].uv;
	const int n = grid;
	if(uv.size() != (size_t)n * n * 6){
		return false;
	}
	// written so that NaN ends up at 0 (a NaN converted to int is undefined)
	float fx = u * (n - 1);
	float fy = v * (n - 1);
	if(!(fx >= 0.f)){
		fx = 0.f;
	}
	if(!(fy >= 0.f)){
		fy = 0.f;
	}
	if(fx > (float)(n - 1)){
		fx = (float)(n - 1);
	}
	if(fy > (float)(n - 1)){
		fy = (float)(n - 1);
	}
	int x0 = (int)fx;
	int y0 = (int)fy;
	x0 = x0 > n - 2 ? n - 2 : x0;
	y0 = y0 > n - 2 ? n - 2 : y0;
	const float tx = fx - x0;
	const float ty = fy - y0;
	auto at = [&](int x, int y, int k){ return uv[((size_t)y * n + x) * 6 + channel * 2 + k]; };
	float result[2];
	for(int k = 0; k < 2; k++){
		const float top = at(x0, y0, k) * (1.f - tx) + at(x0 + 1, y0, k) * tx;
		const float bottom = at(x0, y0 + 1, k) * (1.f - tx) + at(x0 + 1, y0 + 1, k) * tx;
		result[k] = top * (1.f - ty) + bottom * ty;
	}
	outU = result[0];
	outV = result[1];
	return true;
}

std::string Data::CaptureTimeString() const{
	const std::time_t time = (std::time_t)captureTime;
	const std::tm* utc = std::gmtime(&time);
	if(!utc){
		return "unknown time";
	}
	char text[64] = {};
	std::strftime(text, sizeof(text), "%Y-%m-%d %H:%M UTC", utc);
	return text;
}

int Data::AgeDays() const{
	const int64_t now = (int64_t)std::time(nullptr);
	return now > captureTime ? (int)((now - captureTime) / 86400) : 0;
}

std::string FileName(std::string_view serial){
	if(serial.empty() || serial.size() > 64){
		return "";
	}
	for(char c : serial){
		const bool ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '-' || c == '_';
		if(!ok){
			return "";
		}
	}
	return "pvr-data-" + std::string(serial) + ".json";
}

static bool Finite(const float* values, int count){
	for(int i = 0; i < count; i++){
		if(!std::isfinite(values[i])){
			return false;
		}
	}
	return true;
}

std::string Validate(const Data& data){
	if(data.panelWidth <= 0 || data.panelHeight <= 0){
		return "the panel size is " + std::to_string(data.panelWidth) + "x" + std::to_string(data.panelHeight);
	}
	if(data.grid < 2 || data.grid > 1025){
		return "the grid size " + std::to_string(data.grid) + " is out of range";
	}
	const size_t samples = (size_t)data.grid * data.grid * 6;
	for(int vrf = 0; vrf < 2; vrf++){
		for(int e = 0; e < 2; e++){
			const Eye& eye = data.eyes[vrf][e];
			const std::string where = "view_rotation_fix " + std::to_string(vrf) + (e == 0 ? " left eye" : " right eye");
			const float fov[4] = {eye.fov.left, eye.fov.right, eye.fov.up, eye.fov.down};
			if(!Finite(fov, 4) || !(eye.fov.left + eye.fov.right > 0.f) || !(eye.fov.up + eye.fov.down > 0.f)){
				return where + ": the field of view is not usable";
			}
			if(!Finite(eye.position, 3) || !Finite(eye.orientation, 4)){
				return where + ": the eye pose is not finite";
			}
			if(eye.textureWidth <= 0 || eye.textureHeight <= 0){
				return where + ": the texture size is not usable";
			}
			if(eye.failedSamples != 0){
				return where + ": " + std::to_string(eye.failedSamples) + " distortion samples failed";
			}
			if(eye.uv.size() != samples){
				return where + ": " + std::to_string(eye.uv.size()) + " distortion values, expected " + std::to_string(samples);
			}
			if(!Finite(eye.uv.data(), (int)eye.uv.size())){
				return where + ": a distortion value is not finite";
			}
		}
	}
	return "";
}

std::string CheckHeadset(const Data& data, int productId, std::string_view serial){
	if(data.productId != productId){
		char text[96] = {};
		std::snprintf(text, sizeof(text), "it is for product id %04x, the headset is %04x", data.productId, productId);
		return text;
	}
	if(serial.empty() || data.serial != serial){
		return "it is for another serial number";
	}
	return "";
}

// ~6 significant digits; returned as the double nearest the shortened decimal so that the JSON
// writer, which prints the shortest text that reads back the same, prints it short
static double Round6(float value){
	char text[32] = {};
	std::snprintf(text, sizeof(text), "%.6g", value);
	return std::strtod(text, nullptr);
}

bool Save(const std::string& path, const Data& data, std::string& error){
	try{
		ordered_json projections = ordered_json::array();
		for(int vrf = 0; vrf < 2; vrf++){
			ordered_json eyes = ordered_json::array();
			for(int e = 0; e < 2; e++){
				const Eye& eye = data.eyes[vrf][e];
				std::vector<double> uv(eye.uv.size());
				for(size_t i = 0; i < uv.size(); i++){
					uv[i] = Round6(eye.uv[i]);
				}
				eyes.push_back({
					{"fov", {{"left", Round6(eye.fov.left)}, {"right", Round6(eye.fov.right)}, {"up", Round6(eye.fov.up)}, {"down", Round6(eye.fov.down)}}},
					{"hmdToEye", {
						{"position", {Round6(eye.position[0]), Round6(eye.position[1]), Round6(eye.position[2])}},
						{"orientation", {Round6(eye.orientation[0]), Round6(eye.orientation[1]), Round6(eye.orientation[2]), Round6(eye.orientation[3])}},
					}},
					{"textureSize", {eye.textureWidth, eye.textureHeight}},
					{"failedSamples", eye.failedSamples},
					{"uv", uv},
				});
			}
			projections.push_back({{"eyes", eyes}});
		}
		ordered_json file = {
			{"format", kFormat},
			{"version", kVersion},
			{"productName", data.productName},
			{"productId", data.productId},
			{"serial", data.serial},
			{"captureTime", data.captureTime},
			{"captureTimeUtc", data.CaptureTimeString()},
			{"panel", {data.panelWidth, data.panelHeight}},
			{"grid", data.grid},
			{"viewRotationFix", projections},
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

bool Load(const std::string& path, Data& data, std::string& error){
	std::ifstream in(path, std::ios::binary);
	if(!in){
		error = "no file";
		return false;
	}
	Data result;
	try{
		const json file = json::parse(in);
		if(file.value("format", "") != kFormat){
			error = "unknown format";
			return false;
		}
		if(file.value("version", 0) != kVersion){
			error = "unsupported version " + std::to_string(file.value("version", 0));
			return false;
		}
		result.productName = file.at("productName").get<std::string>();
		result.productId = file.at("productId").get<int>();
		result.serial = file.at("serial").get<std::string>();
		result.captureTime = file.at("captureTime").get<int64_t>();
		result.panelWidth = file.at("panel").at(0).get<int>();
		result.panelHeight = file.at("panel").at(1).get<int>();
		result.grid = file.at("grid").get<int>();
		const json& projections = file.at("viewRotationFix");
		for(int vrf = 0; vrf < 2; vrf++){
			for(int e = 0; e < 2; e++){
				const json& source = projections.at(vrf).at("eyes").at(e);
				Eye& eye = result.eyes[vrf][e];
				const json& fov = source.at("fov");
				eye.fov.left = fov.at("left").get<float>();
				eye.fov.right = fov.at("right").get<float>();
				eye.fov.up = fov.at("up").get<float>();
				eye.fov.down = fov.at("down").get<float>();
				ReadFloats(source.at("hmdToEye").at("position"), eye.position, 3);
				ReadFloats(source.at("hmdToEye").at("orientation"), eye.orientation, 4);
				eye.textureWidth = source.at("textureSize").at(0).get<int>();
				eye.textureHeight = source.at("textureSize").at(1).get<int>();
				eye.failedSamples = source.at("failedSamples").get<int>();
				const json& uv = source.at("uv");
				if(!uv.is_array()){
					throw std::runtime_error("uv is not an array");
				}
				eye.uv.resize(uv.size());
				for(size_t i = 0; i < uv.size(); i++){
					eye.uv[i] = uv[i].get<float>();
				}
			}
		}
	}catch(const std::exception& e){
		error = std::string("cannot read it: ") + e.what();
		return false;
	}
	error = Validate(result);
	if(!error.empty()){
		return false;
	}
	data = std::move(result);
	return true;
}

}
