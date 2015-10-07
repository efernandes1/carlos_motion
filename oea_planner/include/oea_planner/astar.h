#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>

#include <oea_planner/structs.h>
#include <oea_planner/planner_paramsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h> // map
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>


#define AStarVirgin 0
#define AStarObstacle 1
#define AStarClosed 2
#define AStarOpen 3
#define AStarInflated 4
#define AStarInvalid 9
#define AStarHeapArraySize 512 * 1024*10
#define AStarDirCount 16

//RAINBOW COLORS FOR LAYERS (0 (red) -> 7 (pink))
#define color_layer0 0xFF0000 //red
#define color_layer1 0xFFA500 //orangr
#define color_layer2 0xFFFF00 //yellow
#define color_layer3 0x00FF00 // green
#define color_layer4 0x00FFFF //cyan
#define color_layer5 0x0000FF //blue
#define color_layer6 0x7D26CD //purple
#define color_layer7 0xFF69B4 // pink

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

inline double to_degrees(double radians) {return radians*(180.0/M_PI);}

inline double to_radians(double degrees) {return degrees*(M_PI/180.0);}

class TAstar
{
public:
    explicit TAstar(std::string logger_name);
    ~TAstar();
    std::string logger_name_;
    TAStarMap AStarMap_;
    TMap world_map_;
    TWorldPose robot_init_world_pose_, goal_world_pose_; //robot_world_current_pose, goal_world_target;
    TGridCoord robot_init_grid_pose_, goal_grid_pose_; //robot_grid_current_pose_x, robot_grid_current_pose_y, robot_grid_current_z_layer;
    double robot_x_size_, robot_y_size_; //params
    int robot_footprint_center_, high_cost_footprint_center_;

    uint8_t *robot_oriented_padded_footprint_, *high_cost_oriented_padded_footprint_;
    cv::Mat robot_padded_footprint_mat_,  high_cost_padded_footprint_mat_;
    bool inflate_map_borders_, allow_unknown_, stop_at_exact_target_, publish_entire_pcd_; //params
    int n_inflated_cells_;
    nav_msgs::Path path_msg_;
    visualization_msgs::MarkerArray marker_array_arrows_, marker_array_cells_;
 	int last_path_number_of_points_;
    int level_closest, level_middle, level_farthest;
    bool use_frontal_laser, use_back_laser;
    long int cost_scale;
    bool penalize_heading_change;

    dynamic_reconfigure::Server<oea_planner::planner_paramsConfig> params_server;
    dynamic_reconfigure::Server<oea_planner::planner_paramsConfig>::CallbackType f;

    void allocate();
// set
    void SetGridPoint(TGridCoord& point, int x, int y, int z);
    void SetGridFromMap(const nav_msgs::OccupancyGrid::ConstPtr& map, sensor_msgs::PointCloud2& pcd);
    void SetGridCellState(int x, int y, int z, int newstate);
    void SetGridCellState(TGridCoord Pnt, int newstate);
    void SetGridCellCost(int x, int y, int z, int cost);
// get
    int GetGridCellState(int x, int y, int z);
    int GetGridCellState(TGridCoord point);
    int Get_index(int h, int w, std::string str);
    int Get_index(int l, int h, int w, std::string str);
    int Get_index(TGridCoord point, std::string str);
    bool index_2D_is_valid(int index);
    bool index_3D_is_valid(int index);
// conversions
    void ConvertMatrixCoordToWorl( int mx, int my, float &wx, float &wy);
    void ConvertMatrixCoordToWorl( int mx, int my, int mz, float &wx, float &wy, float &wz);
    void ConvertWorlCoordToMatrix(double wx, double wy, int &mx, int &my);
    void ConvertWorlCoordToMatrix(double wx, double wy, double wyaw, int &mx, int &my, int &z_layer);
    void ConvertWorlCoordToMatrix(TWorldPose pose_world, TGridCoord& pose_matrix);
    int n_layer_from_yaw(double yaw);
    double yaw_from_n_layer(int n_layer);
    void add_to_pointCloud(int cx, int cy, int layer, sensor_msgs::PointCloud2& pcd);
//Astar
    bool AStarGo(int maxIter, std::string &error_str);
    void AStarInit();
    int AStarStep();
    void AddToAStarList(TGridCoord Pnt);
    void AStarClear();

    void RemoveBestFromAStarList(TGridCoord &Pnt);

//grid
    void ClearGridState();

//footprint and inflation
    void get_robot_high_cost_footprint();
    void get_oriented_footprint(int layer);
    void get_robot_padded_footprint();
    void obstacle_inflation(int layer, int h, int w);
    void high_cost_inflation(int layer, int h, int w);
    uint get_robot_index(int h, int w, int robot_x_cells);
    uint get_pad_index(int h, int w);
    uint get_high_cost_index(int h, int w);

//checks
    bool is_valid_point(TGridCoord point, std::string &error);
    bool is_inside_map_boundries(int x , int y);

//heap
    void UpdateHeapPositionByPromotion(int idx);
    void UpdateHeapPositionByDemotion(int idx);
    void SwapHeapElements(int idx1, int idx2);
    int CalcHeapCost(int idx);

    int CalcH(TGridCoord Pi, TGridCoord Pf);
//other functions
    void getPath();
    void send_arrows_array(float wx, float wy, float wz);
    void add_cubes_array(int x, int y, int color);
    void delete_arrows_array(int number_of_arrows);

    void paramsCB(oea_planner::planner_paramsConfig &config, uint32_t level);
private:

    TNeighbour SixteenWayNeighbours_[16];
    int **CHCache_;
    uint biggest_dim_, high_cost_dim_;
    int VisitNeighbours[6]; //static?
    int VisitNeighboursSign[6];
    int marker_id_;
 	bool update_grid;

};



#endif // ASTAR_H
