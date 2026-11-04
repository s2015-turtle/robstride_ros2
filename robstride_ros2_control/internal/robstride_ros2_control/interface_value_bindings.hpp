#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace robstride_ros2_control
{

// Bind framework-owned interfaces to stable driver storage once during export.
// Cyclic transfers use cached handles, non-blocking access and no allocations.
template<typename Interface>
class InterfaceValueBindings
{
public:
  void clear() {bindings_.clear();}

  void add(std::shared_ptr<Interface> handle, double & value)
  {
    bindings_.push_back({std::move(handle), &value, 0.0});
  }

  bool publish() const
  {
    for (const auto & binding : bindings_) {
      if (!binding.handle->set_value(*binding.value, false)) {return false;}
    }
    return true;
  }

  // Only newly claimed commands are reset; unrelated controllers retain their values.
  bool publish_selected(const std::vector<std::string> & names) const
  {
    for (const auto & binding : bindings_) {
      if (std::find(names.begin(), names.end(), binding.handle->get_name()) != names.end() &&
        !binding.handle->set_value(*binding.value, false))
      {
        return false;
      }
    }
    return true;
  }

  bool consume()
  {
    // A failed read must not leave a partially updated multi-joint command batch.
    for (auto & binding : bindings_) {
      if (!binding.handle->get_value(binding.sample, false)) {return false;}
    }
    for (const auto & binding : bindings_) {*binding.value = binding.sample;}
    return true;
  }

private:
  struct Binding
  {
    std::shared_ptr<Interface> handle;
    double * value;
    double sample;
  };
  std::vector<Binding> bindings_;
};

}  // namespace robstride_ros2_control
