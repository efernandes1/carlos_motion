#ifndef STRUCTS_H
#define STRUCTS_H

// structs for Astar

struct TMap {
    double resolution;
    int width;
    int height;
    float origin_x;
    float origin_y;
    float origin_yaw;
};

struct TNeighbour {
    int x, y; // every neighbour has x,y coordinates and a distance (cost) to the cell
    int dist;
};

struct TGridCoord {
    int x, y, z; // coord for each grid cell
};

struct TAStarCell {
    int  G,H; // each cell has a G and H cost
    TGridCoord ParentPoint; //parent of this cell
    int HeapIdx;
    TGridCoord MyCoord; //coord of this cell
    int8_t State;
    int8_t Cost;
};

struct  TAStarProfiler {
  int RemovePointFromAStarList_count;
  int RemoveBestFromAStarList_count;
  int AddToAStarList_count;
  int HeapArrayTotal;
  int comparesTotal;
 // int iter;
  long int iter;
};

struct  TAStarHeapArray {
    TAStarCell **data;
    int count;
};

struct  TAStarMap {
    TGridCoord InitialPoint, TargetPoint, ActualTargetPoint;
    TAStarCell *Grid;
    TAStarHeapArray HeapArray;
    TAStarProfiler Profiler;
    int Penalty; //Penalize going backwards or forward if missing a laser
};

struct TWorldPose {
    double x;
    double y;
    double yaw;
};

#endif // STRUCTS_H
