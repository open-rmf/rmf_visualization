#ifndef RMF_VISUALIZATION_SCHEDULE__COMMONDATA_HPP
#define RMF_VISUALIZATION_SCHEDULE__COMMONDATA_HPP

#include <rmf_traffic/Time.hpp>
#include <string>
#include <memory>

namespace rmf_visualization_schedule {

struct RequestParam
{
  std::string map_name;
  rmf_traffic::Time start_time;
  rmf_traffic::Time finish_time;
};

using RequestParamPtr = std::shared_ptr<RequestParam>;

} // namespace rmf_visualization_schedule

#endif //RMF_VISUALIZATION_SCHEDULE__COMMONDATA_HPP
