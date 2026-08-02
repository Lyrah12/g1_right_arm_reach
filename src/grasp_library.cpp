/**
 * @file grasp_library.cpp
 * @brief 15 pre-defined grasp patterns for Inspire RH56 hand (6 active joints).
 *
 * RH56 underactuation: 6 active motors -> 12 joints.
 *   Active (commanded):  thumb_yaw, thumb_pitch, index/middle/ring/pinky proximal
 *   Slave (equality-coupled, follow automatically):
 *     thumb_intermediate = 1.6×thumb_pitch, thumb_distal = 2.4×thumb_pitch
 *     index/middle/ring/pinky intermediate = 1.0×proximal
 *
 *   Joint ranges (DFQ right hand URDF):
 *     thumb_yaw   [-0.10, 1.30]  (abduction/opposition)
 *     thumb_pitch [-0.10, 0.60]  (flexion)
 *     fingers     [ 0.00, 1.70]  (flexion; 0 = extended)
 *
 * A "curl" on a finger only needs its proximal joint set — the intermediate
 * phalanx follows at 1:1 (the real hand's linkage). Grasp targets here are
 * POSITIONS for the finger impedance controller (τ = Kp·(q_des − q)).
 */

#include "g1_right_arm_reach/grasp_library.hpp"

namespace g1_right_arm_reach {

// ============================================================================
// Joint name helpers (6 active Inspire joints)
// ============================================================================

namespace {

constexpr const char* J_TY = "R_thumb_proximal_yaw_joint";
constexpr const char* J_TP = "R_thumb_proximal_pitch_joint";
constexpr const char* J_IX = "R_index_proximal_joint";
constexpr const char* J_MD = "R_middle_proximal_joint";
constexpr const char* J_RG = "R_ring_proximal_joint";
constexpr const char* J_PK = "R_pinky_proximal_joint";

/// All 6 joints default to 0 (extended), then override the named ones.
std::map<std::string, double> pose(
    double ty, double tp, double ix, double md, double rg, double pk)
{
    return {
        {J_TY, ty}, {J_TP, tp},
        {J_IX, ix}, {J_MD, md}, {J_RG, rg}, {J_PK, pk},
    };
}

/// Four fingers curled to the same value, thumb explicit.
std::map<std::string, double> curl(double fingers, double ty, double tp)
{
    return pose(ty, tp, fingers, fingers, fingers, fingers);
}

}  // namespace

// ============================================================================
// Individual pattern builders
// ============================================================================

static GraspPattern makePattern_open() {
    return {"open", "All fingers extended (rest)", pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
}

static GraspPattern makePattern_close() {
    // Full fist — 4 fingers fully curled, thumb abducted + flexed over them
    return {"close", "Full fist — thumb over curled fingers",
            curl(1.5, 0.35, 0.5)};
}

static GraspPattern makePattern_home() {
    // Natural semi-grasp (default rest)
    // thumb_pitch 0.35→0.25: 抬高拇指避免碰食指（与桥接 hand_ready_pose 一致）
    return {"home", "Neutral semi-grasp", curl(0.8, 0.25, 0.25)};
}

static GraspPattern makePattern_pinch() {
    // Precision pinch — thumb + index tip-to-tip (others open)
    return {"pinch", "Precision pinch — thumb + index (others open)",
            pose(0.55, 0.5, 1.0, 0.0, 0.0, 0.0)};
}

static GraspPattern makePattern_tripod() {
    // Thumb + index + middle
    return {"tripod", "Tripod — thumb + index + middle (others open)",
            pose(0.5, 0.5, 0.95, 0.95, 0.0, 0.0)};
}

static GraspPattern makePattern_three_finger() {
    return {"three_finger", "Three-finger — thumb + index + middle (others open)",
            pose(0.5, 0.45, 0.9, 0.9, 0.0, 0.0)};
}

static GraspPattern makePattern_power_grasp() {
    // Power wrap — all fingers curled, thumb wraps over
    return {"power_grasp", "Power grasp — thumb over wrapped fingers",
            curl(1.5, 0.4, 0.55)};
}

static GraspPattern makePattern_hook_grasp() {
    // Hook — moderate proximal curl (carry/hook); thumb slightly flexed
    return {"hook_grasp", "Hook grip — fingers curled for carrying",
            curl(1.0, 0.2, 0.4)};
}

static GraspPattern makePattern_spherical_grasp() {
    // Spherical — fingers spread via thumb opposition + uniform curl
    return {"spherical_grasp", "Spherical grasp — spread for round objects",
            curl(1.2, 0.5, 0.5)};
}

static GraspPattern makePattern_cylindrical_grasp() {
    // Cylindrical — uniform curl
    return {"cylindrical_grasp", "Cylindrical grasp — uniform curl",
            curl(1.2, 0.35, 0.5)};
}

static GraspPattern makePattern_lateral_pinch() {
    // Key pinch — thumb adducts to index side, index curled
    return {"lateral_pinch", "Lateral (key) pinch — thumb against index side",
            pose(0.9, 0.4, 0.8, 0.0, 0.0, 0.0)};
}

static GraspPattern makePattern_point() {
    // Pointing — index extended (0), others curled
    return {"point", "Pointing — index extended, others closed",
            pose(0.3, 0.5, 0.0, 1.4, 1.4, 1.4)};
}

static GraspPattern makePattern_thumbs_up() {
    // Thumb extended up, fingers closed
    return {"thumbs_up", "Thumbs up — thumb extended, fingers closed",
            pose(0.5, 0.0, 1.4, 1.4, 1.4, 1.4)};
}

static GraspPattern makePattern_peace_sign() {
    // Index + middle extended, ring + pinky curled
    return {"peace_sign", "Peace / V sign — index+middle extended",
            pose(0.3, 0.4, 0.0, 0.0, 1.4, 1.4)};
}

static GraspPattern makePattern_trigger() {
    // Index slightly curled (trigger), others closed
    return {"trigger", "Trigger finger — index slightly flexed",
            pose(0.3, 0.5, 0.5, 1.3, 1.3, 1.3)};
}

// ============================================================================
// GraspLibrary core methods
// ============================================================================

GraspLibrary::GraspLibrary() {
    initBuiltinPatterns();
}

void GraspLibrary::registerPattern(const GraspPattern& pattern) {
    patterns_[pattern.name] = pattern;
}

GraspPattern GraspLibrary::getPattern(const std::string& name) const {
    auto it = patterns_.find(name);
    if (it != patterns_.end()) return it->second;
    return GraspPattern{};  // empty name → "not found"
}

std::vector<std::string> GraspLibrary::listPatterns() const {
    std::vector<std::string> names;
    names.reserve(patterns_.size());
    for (const auto& kv : patterns_) names.push_back(kv.first);
    return names;
}

bool GraspLibrary::hasPattern(const std::string& name) const {
    return patterns_.count(name) > 0;
}

// ============================================================================
// Registry
// ============================================================================

void GraspLibrary::initBuiltinPatterns() {
    registerPattern(makePattern_open());
    registerPattern(makePattern_close());
    registerPattern(makePattern_home());
    registerPattern(makePattern_pinch());
    registerPattern(makePattern_tripod());
    registerPattern(makePattern_three_finger());
    registerPattern(makePattern_power_grasp());
    registerPattern(makePattern_hook_grasp());
    registerPattern(makePattern_spherical_grasp());
    registerPattern(makePattern_cylindrical_grasp());
    registerPattern(makePattern_lateral_pinch());
    registerPattern(makePattern_point());
    registerPattern(makePattern_thumbs_up());
    registerPattern(makePattern_peace_sign());
    registerPattern(makePattern_trigger());
}

}  // namespace g1_right_arm_reach
