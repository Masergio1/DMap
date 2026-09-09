#pragma once
#include "grid.h"
#include <deque>

struct DMapCell {
  DMapCell* parent=0;
};

struct DMap: public Grid_<DMapCell> {
  using BaseType = Grid_<DMapCell>;
  using FrontierType = std::deque<DMapCell*>;

  // cached max squared distance
  int dmax2;
  
  DMap(int r=0, int c=0):
    BaseType(r,c){
  }

  void clear() {
    for (auto& c: cells)
      c.parent=0;
  }

  int compute(const std::vector<Vector2i>& obstacles, int dmax2_);
  
  template <typename CellType>
  void copyTo(Grid_<CellType>& dest_grid, int dmax2_=0) const {
    if (dmax2_==0)
      dmax2_=this->dmax2;
    dest_grid.resize(rows, cols);
    for (size_t i=0; i<cells.size(); ++i) {
      const auto& src=cells[i];
      auto& dest=dest_grid.cells[i];
      int d2=dmax2_;
      if (src.parent)
        d2=distance2(&src, src.parent);
      dest=sqrt(d2);
    }
  }

};
