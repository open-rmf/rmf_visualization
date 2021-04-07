#ifndef RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__NEGOTIATION_MODEL_HPP
#define RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__NEGOTIATION_MODEL_HPP

#include <vector>
#include <map>
#include <QTableWidget>

class NegotiationModel
{
public:
  NegotiationModel(QTableWidget* _negotiation_view);
  void add_negotiation(
    uint64_t negotiation_id,
    const std::vector<uint64_t>& participants);
  void remove(
    uint64_t negotiation_id);
  void get_selected_id(std::vector<uint64_t>& negotiations);
private:
  QTableWidget* _negotiation_view;
  //TODO(arjo): Implement custom class to track state/
  //Uses map to maintain sorted order.
  std::map<uint64_t, std::vector<uint64_t>> _model;
  void render();
  uint64_t get_negotiation_id(std::size_t row_number);
  QString render_participants(uint64_t conflict_version);
};

#endif // RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__NEGOTIATION_MODEL_HPP