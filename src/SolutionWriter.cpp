#include "SolutionWriter.hpp"
#include <string_view>
#include "structs.h.cl"

using namespace std::literals;

const std::string_view SolutionWriter::csv_header =
  "Start Normal X, Start Normal Y, Start Normal Z, "
  "Start Position X, Start Position Y, Start Position Z, "
  "Frame 1 Position X, Frame 1 Position Y, Frame 1 Position Z, "
  "1-up Platform Position X, 1-up Platform Position Y, 1-up Platform Position Z, "
  "Return Position X, Return Position Y, Return Position Z, "
  "Pre-10K Speed, Pre-10K X Velocity, Pre-10K Z Velocity, "
  "Return Speed, Return X Velocity, Return Z Velocity, "
  "Frame 2 Q-steps, "
  "10K Stick X, 10K Stick Y, "
  "10K Camera Yaw, "
  "Start Floor Normal X, Start Floor Normal Y, Start Floor Normal Z, "
  "Start Position Limit 1 X, Start Position Limit 1 Y, Start Position Limit 1 Z, "
  "Start Position Limit 2 X, Start Position Limit 2 Y, Start Position Limit 2 Z, "
  "Number of Tilt Frames, "
  "Post-Tilt Platform Normal X, Post-Tilt Platform Normal Y, Post-Tilt Platform Normal Z, "
  "Post-Tilt Position X, Post-Tilt Position Y, Post-Tilt Position Z, "
  "Upwarp PU X, Upwarp PU Z, "
  "Upwarp Slide Facing Angle, Upwarp Slide IntendedMag, Upwarp Slide IntendedDYaw\n"sv;

void SolutionWriter::write_soln(const TenKSolution &tenK, const PUSolution &pu, const PlatformSolution& plat) {
  
}