#include "g1_right_arm_reach/grasp_library.hpp"

namespace g1_right_arm_reach {
namespace {

constexpr const char* kThumbOpposition = "right_thumb_metacarpal_joint";
constexpr const char* kThumbFlexion = "right_thumb_proximal_joint";
constexpr const char* kIndex = "right_index_proximal_joint";
constexpr const char* kMiddle = "right_middle_proximal_joint";
constexpr const char* kRing = "right_ring_proximal_joint";
constexpr const char* kPinky = "right_pinky_proximal_joint";

std::map<std::string, double> pose(
    double opposition, double thumb, double index,
    double middle, double ring, double pinky)
{
    return {{kThumbOpposition, opposition}, {kThumbFlexion, thumb},
            {kIndex, index}, {kMiddle, middle}, {kRing, ring}, {kPinky, pinky}};
}

GraspPattern pattern(const std::string& name, const std::string& description,
                     double opposition, double thumb, double index,
                     double middle, double ring, double pinky)
{
    return {name, description,
            pose(opposition, thumb, index, middle, ring, pinky)};
}

}  // namespace

GraspLibrary::GraspLibrary() { initBuiltinPatterns(); }

void GraspLibrary::registerPattern(const GraspPattern& grasp)
{
    if (!grasp.name.empty()) patterns_[grasp.name] = grasp;
}

GraspPattern GraspLibrary::getPattern(const std::string& name) const
{
    const auto iterator = patterns_.find(name);
    return iterator == patterns_.end() ? GraspPattern{} : iterator->second;
}

std::vector<std::string> GraspLibrary::listPatterns() const
{
    std::vector<std::string> names;
    for (const auto& item : patterns_) names.push_back(item.first);
    return names;
}

bool GraspLibrary::hasPattern(const std::string& name) const
{
    return patterns_.count(name) != 0;
}

void GraspLibrary::initBuiltinPatterns()
{
    // Ranges: opposition 0..1.571, thumb flexion 0..1.030,
    // four-finger flexion 0..1.414 rad.
    registerPattern(pattern("open", "Fully open", 0, 0, 0, 0, 0, 0));
    registerPattern(pattern("home", "Safe semi-open pose", 0.45, 0.18,
                            0.18, 0.18, 0.18, 0.18));
    registerPattern(pattern("close", "Full power close", 1.18, 0.90,
                            1.25, 1.25, 1.25, 1.20));
    registerPattern(pattern("pinch", "Thumb-index precision pinch", 1.25, 0.82,
                            1.02, 0.10, 0.05, 0.05));
    registerPattern(pattern("tripod", "Thumb-index-middle tripod", 1.20, 0.78,
                            1.00, 0.92, 0.10, 0.08));
    registerPattern(pattern("cable_pinch", "Tactile cable pinch", 1.24, 0.78,
                            0.92, 0.62, 0.08, 0.05));
    registerPattern(pattern("cylindrical_grasp", "Cylindrical wrap", 1.05, 0.78,
                            1.12, 1.14, 1.12, 1.08));
    registerPattern(pattern("power_grasp", "Power wrap", 1.12, 0.88,
                            1.30, 1.30, 1.28, 1.22));
    registerPattern(pattern("point", "Index extended", 0.55, 0.55,
                            0.0, 1.15, 1.18, 1.15));
}

}  // namespace g1_right_arm_reach
