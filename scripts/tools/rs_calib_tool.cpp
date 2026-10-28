// SPDX-License-Identifier: BSD-3-Clause
// Build: g++ -std=c++17 -O2 -I/opt/ros/humble/include -o rs_calib_tool rs_calib_tool.cpp \
//          -L/opt/ros/humble/lib/x86_64-linux-gnu -lrealsense2 -Wl,-rpath,/opt/ros/humble/lib/x86_64-linux-gnu
// (plan docs/ja/plans/2026-10-28-avoidance-baseline.md §4.4). rs-on-chip-calib is interactive and
// cannot save/restore the current table, which the plan requires before any EEPROM write.
//
// Back up / restore the D435 depth calibration table and run Intel's
// On-Chip (OCC) and Tare calibrations through the public librealsense API
// (rs2::auto_calibrated_device). Non-interactive so each step is logged.
//   info | backup FILE | restore FILE | occ OUT.bin [--write] | tare GT_MM OUT.bin [--write]
// OCC/Tare stream depth at 256x144@90, the mode Intel's self-calibration
// white paper and the rs-on-chip-calib example use.
#include <librealsense2/rs.hpp>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

static rs2::device first_device() {
  rs2::context ctx;
  auto list = ctx.query_devices();
  if (list.size() == 0) throw std::runtime_error("no RealSense device");
  return list.front();
}
static void save(const std::string& f, const std::vector<uint8_t>& t) {
  std::ofstream o(f, std::ios::binary);
  o.write(reinterpret_cast<const char*>(t.data()), t.size());
  if (!o) throw std::runtime_error("cannot write " + f);
}
static std::vector<uint8_t> load(const std::string& f) {
  std::ifstream i(f, std::ios::binary);
  if (!i) throw std::runtime_error("cannot read " + f);
  return {std::istreambuf_iterator<char>(i), {}};
}
static const char* verdict(float h) {
  h = h < 0 ? -h : h;
  return h < 0.25f ? "Good" : (h < 0.75f ? "Can be improved" : "Requires calibration");
}

int main(int argc, char** argv) try {
  if (argc < 2) { std::cerr << "usage: info|backup F|restore F|occ OUT [--write]|tare GT_MM OUT [--write]\n"; return 2; }
  std::string cmd = argv[1];
  auto dev = first_device();
  std::cout << dev.get_info(RS2_CAMERA_INFO_NAME) << " S/N " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
            << " FW " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << "\n";
  auto cal = dev.as<rs2::auto_calibrated_device>();
  if (!cal) throw std::runtime_error("device does not support auto calibration");
  if (cmd == "info") return 0;
  if (cmd == "backup" && argc == 3) {
    auto t = cal.get_calibration_table();
    save(argv[2], t);
    std::cout << "saved current table (" << t.size() << " bytes) -> " << argv[2] << "\n";
    return 0;
  }
  if (cmd == "restore" && argc == 3) {
    auto t = load(argv[2]);
    cal.set_calibration_table(t);
    cal.write_calibration();
    std::cout << "restored and wrote " << argv[2] << " to EEPROM\n";
    return 0;
  }
  bool is_tare = cmd == "tare";
  if ((cmd == "occ" && argc >= 3) || (is_tare && argc >= 4)) {
    float gt_mm = is_tare ? std::stof(argv[2]) : 0.f;
    std::string out = argv[is_tare ? 3 : 2];
    bool write = std::string(argv[argc - 1]) == "--write";
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    cfg.enable_stream(RS2_STREAM_DEPTH, 256, 144, RS2_FORMAT_Z16, 90);
    pipe.start(cfg);
    for (int i = 0; i < 30; ++i) pipe.wait_for_frames();  // let auto-exposure settle
    float health = 0.f;
    auto progress = [](float p) { std::cout << "\r  progress " << int(p) << "%" << std::flush; };
    rs2::calibration_table t = is_tare
        ? cal.run_tare_calibration(gt_mm, "{\"average step count\":20,\"step count\":20,\"accuracy\":2}", &health, progress, 30000)
        : cal.run_on_chip_calibration("{\"calib type\":0,\"speed\":3,\"scan parameter\":0}", &health, progress, 30000);
    pipe.stop();
    std::cout << "\n" << cmd << " health " << health << " (" << verdict(health) << ")\n";
    save(out, t);
    std::cout << "new table -> " << out << "\n";
    if (write) {
      cal.set_calibration_table(t);
      cal.write_calibration();
      std::cout << "applied and wrote to EEPROM\n";
    } else {
      std::cout << "not written (pass --write to apply)\n";
    }
    return 0;
  }
  std::cerr << "bad arguments\n";
  return 2;
} catch (const std::exception& e) {
  std::cerr << "ERROR: " << e.what() << "\n";
  return 1;
}
