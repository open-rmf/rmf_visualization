#ifndef RMF_SCHEDULE_VISUALIZER__COMMONDATA_HPP
#define RMF_SCHEDULE_VISUALIZER__COMMONDATA_HPP

#include <rmf_traffic/Time.hpp>
#include <string>
#include <memory>

namespace rmf_schedule_visualizer {

struct RequestParam
{
  std::string map_name;
  rmf_traffic::Time start_time;
  rmf_traffic::Time finish_time;
};

using RequestParamPtr = std::shared_ptr<RequestParam>;

} // namespace rmf_schedule_visualizer

#endif //RMF_SCHEDULE_VISUALIZER__COMMONDATA_HPP
