/**
 * @file grasp_library.cpp
 * @brief 15 pre-defined grasp patterns for Wuji Hand 2 (20-DOF).
 *
 * Joint naming: right_fingerN_jointM (N=1..5, M=1..4)
 *
 *   Joint ranges (Wuji Hand right.xml):
 *     Thumb (finger1): j1 [ 0.05, 1.60]  j2 [−0.14, 0.93]  j3 [−0.46, 1.56]  j4 [−0.47, 1.56]
 *     Fingers 2–5:    j1 [−0.16, 1.56]  j2 [−0.37, 0.37]  j3 [−0.48, 1.55]  j4 [−0.47, 1.57]
 *
 *   ALL joints rotate around local Y axis (axis="0 1 0").
 *   Positive = flex (curl toward palm).  Negative = extend (straighten / hyperextend).
 *
 *   Thumb key insight:
 *     j2 > 0 → abducts thumb across palm toward other fingers.
 *     j2 > 0.5 → thumb penetrates THROUGH index finger (no contact sensing in sim).
 *     j3/j4 > 0 → curls thumb tip (wraps around object / over fingers).
 *     j3/j4 < 0 → keeps thumb straight (useful for open/pinch, NOT for fist).
 *     For a fist: moderate j2=0.45 + slight curl j3/j4≈0.3/0.2 → thumb wraps OVER index.
 *
 * Adding a new pattern: add a function makePattern_<name>() and
 * register it in initBuiltinPatterns().
 */

#include "g1_right_arm_reach/grasp_library.hpp"

#include <algorithm>

namespace g1_right_arm_reach {

// ============================================================================
// Helpers
// ============================================================================

namespace {

std::string fingerJoint(int finger, int joint) {
    return "right_finger" + std::to_string(finger) + "_joint" + std::to_string(joint);
}

/// Set all 20 finger joints to uniform values.
std::map<std::string, double> uniformGrasp(
    double mcp, double abd, double pip, double dip)
{
    std::map<std::string, double> j;
    for (int f = 1; f <= 5; ++f) {
        j[fingerJoint(f, 1)] = mcp;
        j[fingerJoint(f, 2)] = abd;
        j[fingerJoint(f, 3)] = pip;
        j[fingerJoint(f, 4)] = dip;
    }
    return j;
}

/// Set a single finger's 4 joints.
void setFinger(std::map<std::string, double>& j,
    int finger, double j1, double j2, double j3, double j4)
{
    j[fingerJoint(finger, 1)] = j1;
    j[fingerJoint(finger, 2)] = j2;
    j[fingerJoint(finger, 3)] = j3;
    j[fingerJoint(finger, 4)] = j4;
}

}  // namespace

// ============================================================================
// Individual pattern builders
// ============================================================================

static GraspPattern makePattern_open() {
    auto j = uniformGrasp(0.0, 0.0, 0.0, 0.0);
    j["right_finger1_joint1"] = 0.05;  // thumb j1 min range
    return {"open", "All fingers fully extended", j};
}

static GraspPattern makePattern_close() {
    std::map<std::string, double> j;
    // Thumb strategy for simulation (no contact sensing):
    //   j1=0.6: more base rotation → thumb approaches from front of palm
    //   j2=0.25: MINIMAL abduction — stays on its side, doesn't cross the index
    //   j3=0.7, j4=0.6: DEEP curl → wraps AROUND index dorsum, not through it
    //   This creates a "hook over" motion rather than "push through" motion.
    setFinger(j, 1, 0.6, 0.25, 0.7, 0.6);
    // Fingers 2–5 — full fist
    for (int f = 2; f <= 5; ++f)
        setFinger(j, f, 1.5, 0.0, 1.5, 1.5);
    return {"close", "Full fist — thumb curled over index finger", j};
}

static GraspPattern makePattern_home() {
    // Fingers 2–5: semi-grasp
    auto j = uniformGrasp(0.8, 0.0, 0.7, 0.6);
    // Thumb: slightly abducted (j2=0.3), relaxed curl
    setFinger(j, 1, 0.6, 0.3, 0.5, 0.4);
    return {"home", "Neutral semi-grasp (default rest pose)", j};
}

static GraspPattern makePattern_pinch() {
    // ALL fingers start OPEN; only thumb + index move
    auto j = uniformGrasp(0.0, 0.0, 0.0, 0.0);
    j["right_finger1_joint1"] = 0.05;
    // Thumb — abduct (j2=0.6) toward index, flex to meet tip
    setFinger(j, 1, 0.7, 0.6, 1.1, 0.9);
    // Index — curl to meet thumb tip
    setFinger(j, 2, 0.9, 0.0, 1.1, 0.9);
    // Middle, ring, pinky — remain at 0 (open) from uniformGrasp
    return {"pinch", "Precision pinch — thumb + index tip-to-tip (others open)", j};
}

static GraspPattern makePattern_tripod() {
    auto j = uniformGrasp(0.0, 0.0, 0.0, 0.0);
    j["right_finger1_joint1"] = 0.05;
    setFinger(j, 1, 0.7, 0.5, 1.1, 0.9);   // thumb
    setFinger(j, 2, 0.9, 0.0, 1.1, 0.9);   // index
    setFinger(j, 3, 0.9, 0.0, 1.1, 0.9);   // middle
    // Ring + pinky — open
    return {"tripod", "Tripod — thumb + index + middle (others open)", j};
}

static GraspPattern makePattern_three_finger() {
    auto j = uniformGrasp(0.0, 0.0, 0.0, 0.0);
    j["right_finger1_joint1"] = 0.05;
    setFinger(j, 1, 0.7, 0.5, 1.0, 0.8);   // thumb
    setFinger(j, 2, 0.9, 0.0, 1.0, 0.8);   // index
    setFinger(j, 3, 0.9, 0.0, 1.0, 0.8);   // middle
    // Ring + pinky — open
    return {"three_finger", "Three-finger — thumb + index + middle (others open)", j};
}

static GraspPattern makePattern_power_grasp() {
    std::map<std::string, double> j;
    // Thumb — minimal abduction + deep curl to wrap over fingers
    setFinger(j, 1, 0.6, 0.25, 0.7, 0.6);
    // Fingers 2–5 — full wrap
    for (int f = 2; f <= 5; ++f)
        setFinger(j, f, 1.4, 0.0, 1.4, 1.4);
    return {"power_grasp", "Power grasp — thumb curled over wrapped fingers", j};
}

static GraspPattern makePattern_hook_grasp() {
    std::map<std::string, double> j;
    setFinger(j, 1, 0.8, 0.0, 1.2, 1.2);
    // Fingers 2–5 — MCP extended, PIP+DIP flexed (hook)
    for (int f = 2; f <= 5; ++f)
        setFinger(j, f, 0.0, 0.0, 1.5, 1.5);
    return {"hook_grasp", "Hook grip — PIP+DIP flexed for pulling/carrying", j};
}

static GraspPattern makePattern_spherical_grasp() {
    std::map<std::string, double> j;
    setFinger(j, 1, 1.0, 0.3, 1.0, 1.0);
    setFinger(j, 2, 1.2,  0.3, 1.2, 1.0);
    setFinger(j, 3, 1.2,  0.1, 1.2, 1.0);
    setFinger(j, 4, 1.2, -0.1, 1.2, 1.0);
    setFinger(j, 5, 1.2, -0.3, 1.2, 1.0);
    return {"spherical_grasp", "Spherical grasp — fingers spread for round objects", j};
}

static GraspPattern makePattern_cylindrical_grasp() {
    std::map<std::string, double> j;
    setFinger(j, 1, 0.8, 0.5, 1.2, 1.0);
    for (int f = 2; f <= 5; ++f)
        setFinger(j, f, 1.2, 0.0, 1.2, 1.0);
    return {"cylindrical_grasp", "Cylindrical grasp — uniform curl for bottles/cups", j};
}

static GraspPattern makePattern_lateral_pinch() {
    auto j = uniformGrasp(0.0, 0.0, 0.0, 0.0);
    j["right_finger1_joint1"] = 0.05;
    setFinger(j, 1, 0.9, 0.8, 1.0, 0.8);   // thumb — adducted against index side
    setFinger(j, 2, 0.6, 0.0, 1.0, 1.0);   // index
    // Others — open
    return {"lateral_pinch", "Lateral (key) pinch — thumb against index side", j};
}

static GraspPattern makePattern_point() {
    std::map<std::string, double> j;
    setFinger(j, 1, 1.5, 0.0, 1.5, 1.5);
    setFinger(j, 2, 0.0, 0.0, 0.0, 0.0);   // index — extended
    for (int f = 3; f <= 5; ++f)
        setFinger(j, f, 1.5, 0.0, 1.5, 1.5);
    return {"point", "Pointing — index extended, others closed", j};
}

static GraspPattern makePattern_thumbs_up() {
    std::map<std::string, double> j;
    setFinger(j, 1, 0.3, 0.9, 0.0, 0.0);   // thumb — extended upward
    for (int f = 2; f <= 5; ++f)
        setFinger(j, f, 1.5, 0.0, 1.5, 1.5);
    return {"thumbs_up", "Thumbs up — thumb extended, fingers closed", j};
}

static GraspPattern makePattern_peace_sign() {
    std::map<std::string, double> j;
    setFinger(j, 1, 1.2, 0.0, 1.2, 1.2);
    setFinger(j, 2, 0.0, 0.2,  0.0, 0.0);   // index — extended + splayed
    setFinger(j, 3, 0.0, -0.2, 0.0, 0.0);   // middle — extended + splayed
    for (int f = 4; f <= 5; ++f)
        setFinger(j, f, 1.5, 0.0, 1.5, 1.5);
    return {"peace_sign", "Peace / V sign — index+middle extended", j};
}

static GraspPattern makePattern_trigger() {
    std::map<std::string, double> j;
    setFinger(j, 1, 1.2, 0.0, 1.2, 1.2);
    setFinger(j, 2, 0.0, 0.0, 0.0, 1.2);   // index — extended, only DIP flexed
    for (int f = 3; f <= 5; ++f)
        setFinger(j, f, 1.5, 0.0, 1.5, 1.5);
    return {"trigger", "Trigger finger — index DIP flexed for pressing", j};
}

// ============================================================================
// Constructor — register all built-in patterns
// ============================================================================

GraspLibrary::GraspLibrary()
{
    initBuiltinPatterns();
}

void GraspLibrary::initBuiltinPatterns()
{
    registerPattern(makePattern_open());
    registerPattern(makePattern_close());
    registerPattern(makePattern_home());
    registerPattern(makePattern_pinch());
    registerPattern(makePattern_tripod());
    registerPattern(makePattern_power_grasp());
    registerPattern(makePattern_hook_grasp());
    registerPattern(makePattern_spherical_grasp());
    registerPattern(makePattern_cylindrical_grasp());
    registerPattern(makePattern_lateral_pinch());
    registerPattern(makePattern_three_finger());
    registerPattern(makePattern_point());
    registerPattern(makePattern_thumbs_up());
    registerPattern(makePattern_peace_sign());
    registerPattern(makePattern_trigger());
}

// ============================================================================
// Public API
// ============================================================================

GraspPattern GraspLibrary::getPattern(const std::string& name) const
{
    auto it = patterns_.find(name);
    if (it != patterns_.end()) {
        return it->second;
    }
    return GraspPattern{};
}

std::vector<std::string> GraspLibrary::listPatterns() const
{
    std::vector<std::string> names;
    names.reserve(patterns_.size());
    for (const auto& [name, _] : patterns_) {
        names.push_back(name);
    }
    return names;
}

bool GraspLibrary::hasPattern(const std::string& name) const
{
    return patterns_.find(name) != patterns_.end();
}

void GraspLibrary::registerPattern(const GraspPattern& pattern)
{
    patterns_[pattern.name] = pattern;
}

}  // namespace g1_right_arm_reach
