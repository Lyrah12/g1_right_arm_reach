/**
 * @file grasp_library.hpp
 * @brief Dexterous hand grasp pattern library (ROS-independent).
 *
 * Defines 15+ pre-defined grasp patterns for Wuji Hand 2
 * (20-DOF: 5 fingers × 4 joints). Used by both hand_controller_node
 * and action_sequencer_node.
 *
 * Adding a new pattern: add a new entry in initBuiltinPatterns().
 * No other files need modification.
 */

#ifndef G1_RIGHT_ARM_REACH__GRASP_LIBRARY_HPP_
#define G1_RIGHT_ARM_REACH__GRASP_LIBRARY_HPP_

#include <map>
#include <string>
#include <vector>

namespace g1_right_arm_reach {

// ============================================================================
// GraspPattern — one named grasp with 20 joint targets
// ============================================================================

struct GraspPattern {
    std::string name;                        ///< Unique pattern identifier
    std::string description;                 ///< Human-readable description
    std::map<std::string, double> joints;   ///< {joint_name: angle_rad}
};

// ============================================================================
// GraspLibrary — registry of all built-in grasp patterns
// ============================================================================

class GraspLibrary {
public:
    GraspLibrary();

    /// Get a pattern by name. Returns pattern with empty name if not found.
    GraspPattern getPattern(const std::string& name) const;

    /// List all registered pattern names.
    std::vector<std::string> listPatterns() const;

    /// Check whether a named pattern exists.
    bool hasPattern(const std::string& name) const;

    /// Register a custom pattern at runtime.
    void registerPattern(const GraspPattern& pattern);

    /// Number of registered patterns.
    size_t size() const { return patterns_.size(); }

private:
    std::map<std::string, GraspPattern> patterns_;

    /// Populate all 15 pre-defined grasp patterns.
    void initBuiltinPatterns();
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__GRASP_LIBRARY_HPP_
