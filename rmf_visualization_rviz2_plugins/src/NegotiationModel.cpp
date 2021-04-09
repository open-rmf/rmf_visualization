#include "NegotiationModel.hpp"

NegotiationModel::NegotiationModel(QTableWidget* widget)
{
  _negotiation_view = widget;
}

void NegotiationModel::add_negotiation(
  uint64_t negotiation_id,
  const std::vector<uint64_t>& participants)
{
  _model[negotiation_id] = participants;
  render();
}

void NegotiationModel::remove(uint64_t neg_id)
{
  _model.erase(neg_id);
  render();
}

void NegotiationModel::get_selected_id(std::vector<uint64_t>& negotiations)
{
  QItemSelectionModel* select = _negotiation_view->selectionModel();
  if (select->hasSelection())
  {
    auto indices = select->selectedRows();
    for (auto i: indices)
    {
      auto neg_id = get_negotiation_id(i.row());
      negotiations.push_back(neg_id);
    }
  }
}

void NegotiationModel::render()
{
  QStringList table_header;
  _negotiation_view->clearContents();
  _negotiation_view->setRowCount(_model.size());
  auto _current_row = _model.begin();
  for (std::size_t i = 0; i < _model.size(); i++)
  {
    QTableWidgetItem* item = new QTableWidgetItem;
    item->setText(QString::number(_current_row->first));
    _negotiation_view->setItem(i, 0, item);

    QTableWidgetItem* participants = new QTableWidgetItem;
    participants->setText(render_participants(_current_row->first));
    _negotiation_view->setItem(i, 1, participants);
    _current_row++;
  }
}

uint64_t NegotiationModel::get_negotiation_id(std::size_t n)
{
  auto _current_row = _model.begin();
  for (std::size_t i = 0; i < _model.size(); i++)
  {
    if (i == n)
    {
      return _current_row->first;
    }
    _current_row++;
  }
}

QString NegotiationModel::render_participants(uint64_t conflict_version)
{
  QString res("");
  for (auto& item: _model[conflict_version])
  {
    res += QString::number(item) + " ";
  }
  return res;
}
