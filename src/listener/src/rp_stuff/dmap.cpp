#include "dmap.h"

int DMap::compute(const std::vector<Vector2i>& obstacles, int dmax2_) {
  FrontierType frontier;
  dmax2=dmax2_;
  //populate the frontier
  for (const auto& o: obstacles) {
    if (! inside(o))
      continue;
    auto& c=(*this)(o);
    c.parent=&c;
    frontier.push_back(&c);
  }

  int num_ops=0;
  while(! frontier.empty()) {
    auto current=frontier.front();
    frontier.pop_front();
    ++num_ops;
    auto current_pos=ptr2idx(current);
    auto parent=current->parent;
    for (int dr=-1; dr<=1; ++dr)
      for (int dc=-1; dc<=1; ++dc) {
        if (!dr && !dc)
          continue;
        Vector2i neighbor_pos=current_pos+Vector2i(dc, dr);
        if (! inside(neighbor_pos))
          continue;
        auto& neighbor=(*this)(neighbor_pos);
        int neighbor_d2=dmax2;
        if (neighbor.parent) {
          neighbor_d2 = distance2(&neighbor, neighbor.parent);
        }
        int new_neighbor_d2=distance2(&neighbor, parent);
        if(new_neighbor_d2 < neighbor_d2) {
          neighbor.parent=parent;
          frontier.push_back(&neighbor);
        }
      }
  }
  return num_ops;
}
