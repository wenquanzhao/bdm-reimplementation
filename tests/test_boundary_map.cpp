#include "bdm/boundary_map.hpp"
#include <cassert>
#include <iostream>
#include <unordered_map>
using namespace bdm;
int main() {
  BoundaryMap map;
  std::unordered_map<Voxel, State, VoxelHash> grid;
  for (int x=0;x<5;++x) for(int y=0;y<5;++y)
    for(int z=0;z<8;++z) grid[{x,y,z}] = State::Free;
  grid[{2,2,4}] = State::Occupied;
  map.rebuild(grid);
  assert(map.query({2,2,4}) == State::Occupied);
  assert(map.query({2,2,3}) == State::Free);
  assert(map.query({2,2,-2}) == State::Unknown);
  assert(map.query({2,2,9}) == State::Unknown);
  assert(map.query({1,1,2}) == State::Free);
  assert(map.boundaryCount()>0);
  map.rebuild({});
  assert(map.query({2,2,3}) == State::Unknown);
  std::cout << "Static boundary-map smoke tests passed\n";
}
