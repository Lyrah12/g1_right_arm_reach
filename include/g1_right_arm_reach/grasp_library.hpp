#ifndef G1_RIGHT_ARM_REACH__GRASP_LIBRARY_HPP_
#define G1_RIGHT_ARM_REACH__GRASP_LIBRARY_HPP_

#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace g1_right_arm_reach {

/** One named grasp with six active Revo2 joint targets. */
struct GraspPattern {
    std::string name;
    std::string description;
    std::map<std::string, double> joints;
};

/** Registry of built-in and user-registered Revo2 grasp patterns. */
class GraspLibrary {
public:
    GraspLibrary();

    GraspPattern getPattern(const std::string& name) const;
    std::vector<std::string> listPatterns() const;
    bool hasPattern(const std::string& name) const;
    void registerPattern(const GraspPattern& pattern);
    std::size_t size() const { return patterns_.size(); }

private:
    std::map<std::string, GraspPattern> patterns_;
    void initBuiltinPatterns();
};

}  // namespace g1_right_arm_reach

#endif
