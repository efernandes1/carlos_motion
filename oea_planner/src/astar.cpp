#include <ros/ros.h>
#include "oea_planner/astar.h"
#include <sstream>

int FixedPointConst = 65536; //e:(23-28) distance cost (dist)
int sqrt2=round(1.4142135623730950488016887242097 * FixedPointConst);

int fixp_1 = FixedPointConst;
int fixp_3_0 = 3 * fixp_1;
int fixp_3_1 = round(sqrt(3*3 + 1*1) * fixp_1);
int fixp_2_2 = round(sqrt(2*2 + 2*2) * fixp_1);

pcl::PointCloud<pcl::PointXYZRGB >::Ptr Cloud_Markers (new pcl::PointCloud<pcl::PointXYZRGB >);

// constructor
TAstar::TAstar(std::string logger_name)
{
    logger_name_ = logger_name;

    level_closest = 0;
    level_middle = 0;
    level_farthest = 0;
    cost_scale = 0;
    update_grid = false;

    SixteenWayNeighbours_[0].x = 3; SixteenWayNeighbours_[0].y = 0; SixteenWayNeighbours_[0].dist = fixp_3_0; //0
    SixteenWayNeighbours_[1].x = 3; SixteenWayNeighbours_[1].y = 1; SixteenWayNeighbours_[1].dist = fixp_3_1; //1
    SixteenWayNeighbours_[2].x = 2; SixteenWayNeighbours_[2].y = 2; SixteenWayNeighbours_[2].dist = fixp_2_2; //2
    SixteenWayNeighbours_[3].x = 1; SixteenWayNeighbours_[3].y = 3; SixteenWayNeighbours_[3].dist = fixp_3_1; //3
    SixteenWayNeighbours_[4].x = 0; SixteenWayNeighbours_[4].y = 3; SixteenWayNeighbours_[4].dist = fixp_3_0; //4
    SixteenWayNeighbours_[5].x = -1; SixteenWayNeighbours_[5].y = 3; SixteenWayNeighbours_[5].dist = fixp_3_1; //5
    SixteenWayNeighbours_[6].x = -2; SixteenWayNeighbours_[6].y = 2; SixteenWayNeighbours_[6].dist = fixp_2_2; //6
    SixteenWayNeighbours_[7].x = -3; SixteenWayNeighbours_[7].y = 1; SixteenWayNeighbours_[7].dist = fixp_3_1; //7
    SixteenWayNeighbours_[8].x = -3; SixteenWayNeighbours_[8].y = 0; SixteenWayNeighbours_[8].dist = fixp_3_0; //8
    SixteenWayNeighbours_[9].x = -3; SixteenWayNeighbours_[9].y = -1; SixteenWayNeighbours_[9].dist = fixp_3_1; //9
    SixteenWayNeighbours_[10].x = -2; SixteenWayNeighbours_[10].y = -2; SixteenWayNeighbours_[10].dist = fixp_2_2; //10
    SixteenWayNeighbours_[11].x = -1; SixteenWayNeighbours_[11].y = -3; SixteenWayNeighbours_[11].dist = fixp_3_1; //11
    SixteenWayNeighbours_[12].x = 0; SixteenWayNeighbours_[12].y = -3; SixteenWayNeighbours_[12].dist = fixp_3_0; //12
    SixteenWayNeighbours_[13].x = 1; SixteenWayNeighbours_[13].y = -3; SixteenWayNeighbours_[13].dist = fixp_3_1; //13
    SixteenWayNeighbours_[14].x = 2; SixteenWayNeighbours_[14].y = -2; SixteenWayNeighbours_[14].dist = fixp_2_2; //14
    SixteenWayNeighbours_[15].x = 3; SixteenWayNeighbours_[15].y = -1; SixteenWayNeighbours_[15].dist = fixp_3_1; //15
    //                        | 5| 4| 3|
    //                     | 6|		   | 2|
    //                  | 7|     	      | 1|
    //                  | 8|      x       | 0|
    //                  | 9|              |15|
    //                     |10|        |14|
    //                        |11|12|13|

    VisitNeighbours[0] = VisitNeighbours[3] = 0;
    VisitNeighbours[1] = VisitNeighbours[4] = 15;
    VisitNeighbours[2] = VisitNeighbours[5] = 1;
    VisitNeighboursSign[0] = VisitNeighboursSign[1] = VisitNeighboursSign[2] = 1;
    VisitNeighboursSign[3] = VisitNeighboursSign[4] = VisitNeighboursSign[5] = -1;

    marker_id_ = 0;
    last_path_number_of_points_ = 0;

    f = boost::bind(&TAstar::paramsCB, this,  _1, _2);
    params_server.setCallback(f);
}

// destructor
TAstar::~TAstar()
{
    std::cout << "Freeing memory before quiting " << std::endl;

    //Free allocated memory
    // free(robot_oriented_padded_footprint_);
    //free(robot_padded_footprint_);

    free(CHCache_);
    free(AStarMap_.HeapArray.data);
    free(AStarMap_.Grid);
    ros::shutdown();

}


void TAstar::allocate()
{
    //allocate Grid
    AStarMap_.Grid = (TAStarCell*)malloc(AStarDirCount*world_map_.height*world_map_.width*sizeof(TAStarCell));
    ROS_ASSERT_MSG(AStarMap_.Grid != NULL, "Failed to allocate Grid"); //if pointer is NULL, quit

    //allocate Heap
    AStarMap_.HeapArray.data =(TAStarCell**)malloc(AStarHeapArraySize*sizeof(TAStarCell));
    ROS_ASSERT_MSG(AStarMap_.HeapArray.data != NULL, "Failed to allocate Heap"); //if pointer is NULL, quit

    //allocate CHCache
    CHCache_ = (int **) malloc(world_map_.height * sizeof(int *));
    ROS_ASSERT_MSG(CHCache_ != NULL, "Failed to allocate CHCache"); //if pointer is NULL, quit

    for (int i=0; i<world_map_.height; i++)
    {
        CHCache_[i]=(int *)malloc(world_map_.width*sizeof(int));
        ROS_ASSERT_MSG(CHCache_[i] != NULL, "Failed to allocate CHCache"); //if pointer is NULL, quit

        for (int j=0; j<world_map_.width; j++)
        {
            CHCache_[i][j]=round(sqrt(j*j*1.0+i*i)* FixedPointConst);
        }
    }

    AStarMap_.HeapArray.count = 0;
    //ROS_DEBUG(" Allocation OK :)");
}


//***************************//
//      SET FUNCTIONS        //
//***************************//
// SET X, Y, Z in GridPoint
void TAstar::SetGridPoint(TGridCoord& point, int x, int y, int z)
{
    point.x = x;
    point.y = y;
    point.z = z;
}

//convert from ROS map (occupancy grid) to *GridState (16 layers of maps)
void TAstar::SetGridFromMap(const nav_msgs::OccupancyGrid::ConstPtr& map, sensor_msgs::PointCloud2& pcd)
{
    int ind = 0;

    int number_of_obstacles=0;
    ros::Time time_0 = ros::Time::now();
    // set state and coordinates
    for(int l=0 ; l<AStarDirCount; l++)
    {
        for(int h=0; h < world_map_.height; h++) //start w/ y=0 and go up
        {
            for(int w=0; w <  world_map_.width; w++)
            {
                int state = map->data[h*world_map_.width+w];
                switch (state)
                {
                case 100:
                    SetGridCellState(w,h,l,AStarObstacle);
                    ++number_of_obstacles; //global
                    break;
                case -1:
                    if (allow_unknown_)
                    {
                        SetGridCellState(w, h, l, AStarVirgin); //allow search into unknown....
                        SetGridCellCost(w, h, l, 0); // set all virgin cells with 0 cost
                    }
                    else
                        SetGridCellState(w,h,l,AStarObstacle); //this will make a virtual wall around unknown, but because we don't know what's there that's not too bad....
                    break;
                case 0:
                    SetGridCellState(w, h, l, AStarVirgin); // set state to virgin
                    SetGridCellCost(w, h, l, 0); // set all virgin cells with 0 cost
                    break;
                }
                SetGridPoint(AStarMap_.Grid[ind].MyCoord,w,h,l); // update Grid with point coordinates
                ind++;

                if (inflate_map_borders_)
                {
                    if (((h == 0) || (w == 0)) || ((h == world_map_.height-1) || (w == world_map_.width-1))) //map borders
                        SetGridCellState(w,h,l,AStarObstacle);
                }
            }
        }
    }


    ros::Duration d = ros::Time::now()-time_0;
    ROS_DEBUG_STREAM_NAMED(logger_name_, "Setting cells took: " << d.toSec() << " seconds.");

    bool enter = true;
    time_0 = ros::Time::now();

    //inflate obstacles only
    for(int l= 0 ; l<AStarDirCount/2; l++)
    {
        get_oriented_footprint(l);
        for(int h=0; h < world_map_.height; h++) //read from the bottom up...
        {
            for(int w=0; w < world_map_.width; w++) //from left to right
            {
                if ((GetGridCellState(w,h,l) == AStarObstacle) && (enter))
                {
                    high_cost_inflation(l,h,w);
                    obstacle_inflation(l,h,w);
                    // enter = false;
                }
            }
        }



    }
    d = ros::Time::now()-time_0;
    ROS_DEBUG_STREAM_NAMED(logger_name_, "Inflation took: " << d.toSec() << " seconds.");

    Cloud_Markers->reserve(n_inflated_cells_);

    time_0 = ros::Time::now();

    pcd.height = 1;
    pcd.width = n_inflated_cells_;

    enter = true;
    for(int l=0 ; l<AStarDirCount/2; l++)
    {
        for(int h=0; h < world_map_.height; h++) //start w/ y=0 and go up
        {
            for(int w=0; w <  world_map_.width; w++)
            {
                if (publish_entire_pcd_)
                {
                    enter = true;
                }
                else
                {
                    if ((l==0) || (l==4))
                        enter = true;
                    else
                        enter = false;
                }

                int state = GetGridCellState(w,h,l);

                if (state == AStarInflated && enter)
                {
                    add_to_pointCloud(w,h,l, pcd);
                }

                /*if ((AStarMap_.Grid[Get_index(l,h,w,"high_cost_inflation")].Cost > 0) && enter)
                {
                    add_to_pointCloud(w,h,4, pcd);
                }*/
            }
        }
    }


    d = ros::Time::now()-time_0;
    ROS_DEBUG_STREAM_NAMED(logger_name_, "Adding to PCD took: " << d.toSec() << " seconds.");

    pcl::toROSMsg(*Cloud_Markers, pcd); //convert pointCloudXYZ to PointCloud2
    update_grid = true; //after setting the grid we can update it when params change
}

//SET newstate to Grid Cell
void TAstar::SetGridCellState(int x, int y, int z, int newstate)
{
    int index = Get_index(z,y,x, "SetGridCellState(x,y,z)");

    //check if valid
    if (!index_3D_is_valid(index))
    {
        ROS_ERROR_STREAM_NAMED(logger_name_, "SetGridCellState - wrong index: (x,y,z) = (" << x << ", " << y << ", " << z << ")");
        return;
    }
    else
    {
        AStarMap_.Grid[index].State = newstate;
    }
}

void TAstar::SetGridCellCost(int x, int y, int z, int cost)
{
    int index = Get_index(z,y,x, "SetGridCellCost(x,y,z)");

    //check if valid
    if (!index_3D_is_valid(index))
    {
        ROS_ERROR_STREAM_NAMED(logger_name_, " SetGridCellCost - wrong index: (x,y,z) = (" << x << ", " << y << ", " << z << ")");
        return;
    }
    else
    {
        AStarMap_.Grid[index].Cost = cost;
    }
}

void TAstar::SetGridCellState(TGridCoord Pnt, int newstate)
{
    int x = Pnt.x;
    int y = Pnt.y;
    int z = Pnt.z;

   // SetGridCellState(x, y, z, newstate); int index = Get_index(z,y,x);
// to know (from the color msg) where this is happening (if with x,y,z or point
    int index = Get_index(z,y,x, "SetGridCellState(Point)");

    //check if valid
    if (!index_3D_is_valid(index))
    {
        ROS_ERROR_STREAM_NAMED(logger_name_, "SetGridCellState (Pnt) - wrong index = " << index << ": (x,y,z) = (" << x << ", " << y << ", " << z << ")");
        return;
    }
    else
    {
        AStarMap_.Grid[index].State = newstate;
    }
}


//***************************//
//      GET FUNCTIONS        //
//***************************//

// CELL/MAP INDEX:

// GET state from a cell
int TAstar::GetGridCellState(int x, int y, int z)
{


    if (!is_inside_map_boundries(x,y))// || ind > world_map_.height*world_map_.width*AStarDirCount)
    {
        ROS_ERROR_STREAM_NAMED(logger_name_, "Cell [" << z << "]["<< y << "][" << x <<"] is not within map boundries ");
        return AStarInvalid;
    }
    else
    {
        int ind = Get_index(z,y,x,"GetGridCellState(x,y,z)");
        return AStarMap_.Grid[ind].State;
    }

}

int TAstar::GetGridCellState(TGridCoord point)
{
    int x = point.x;
    int y = point.y;
    int z = point.z;

    int ind = Get_index(z,y,x,"GetGridCellState(point)");

    if (!is_inside_map_boundries(x,y) || ind > world_map_.height*world_map_.width*AStarDirCount)
    {
        ROS_ERROR_STREAM_NAMED(logger_name_, "Something went wrong with index ["<< z << "]["<< y << "][" << x <<"] (is not inside map boundries)");
        ROS_BREAK();
    }
    else
        return AStarMap_.Grid[ind].State;
}

//GET index from h, w
int TAstar::Get_index(int h, int w, std::string str)
{
    if(!is_inside_map_boundries(w,h))
    {
        ROS_ERROR_STREAM_NAMED(logger_name_, str + " > tried to get index(h,w):(" << h << ", " << w << ")");
    }
    else
        return h*world_map_.width+w;
}

//GET index from l, h, w
int TAstar::Get_index(int l, int h, int w, std::string str) //str to know where it failed
{
    int ind = l*world_map_.height*world_map_.width + h*world_map_.width + w;

    if(!is_inside_map_boundries(w,h))
    {
        ROS_ERROR_STREAM_NAMED(logger_name_, str + " > tried to get index(l,h,w):(" << l << ", " << h << ", " << w << ") = (index) " << ind);
    }
    return ind;
}

int TAstar::Get_index(TGridCoord point, std::string str)
{
    int l = point.z;
    int h = point.y;
    int w = point.x;

    return Get_index(l, h, w, str);
}

// ROBOT INDEX:

//TODO
void TAstar::get_oriented_footprint(int layer)
{

    float theta= -1*to_degrees(yaw_from_n_layer(layer)); // Non-standard axis in ocv

    cv::Mat oriented_footprint(robot_padded_footprint_mat_.rows, robot_padded_footprint_mat_.cols, CV_8UC1, cv::Scalar::all(0)); // blank image

    cv::Mat high_cost_oriented_footprint(high_cost_padded_footprint_mat_.rows, high_cost_padded_footprint_mat_.cols, CV_8UC1, cv::Scalar::all(0)); // blank image

    //rotate image theta degrees
    cv::Mat matRotation = cv::getRotationMatrix2D(cv::Point((int)robot_padded_footprint_mat_.cols/2, (int) robot_padded_footprint_mat_.rows/2), (theta - 180), 1 );
    cv::warpAffine( robot_padded_footprint_mat_, oriented_footprint, matRotation, robot_padded_footprint_mat_.size());

    robot_oriented_padded_footprint_ = (uint8_t*)oriented_footprint.data;

    // print robot_oriented_padded_footprint_ to console
/*     for (int i=0; i< biggest_dim_; i++)
    {
        for (int j=0; j< biggest_dim_; j++)
        {
            int ind = get_pad_index(i,j);
            if (robot_oriented_padded_footprint_[ind] == 0)
                std::cout << WHITE << static_cast<unsigned>(robot_oriented_padded_footprint_[ind]) << " " << RESET;
            else
                std::cout << GREEN << static_cast<unsigned>(robot_oriented_padded_footprint_[ind]) << " " << RESET;

        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
*/
    //rotate image theta degrees
    cv::Mat matRotation2 = cv::getRotationMatrix2D(cv::Point((int)high_cost_padded_footprint_mat_.cols/2, (int) high_cost_padded_footprint_mat_.rows/2), (theta - 180), 1 );
    cv::warpAffine( high_cost_padded_footprint_mat_, high_cost_oriented_footprint, matRotation2, high_cost_padded_footprint_mat_.size());

    high_cost_oriented_padded_footprint_ = (uint8_t*)high_cost_oriented_footprint.data;

    // print robot_oriented_padded_footprint_ to console
/*     for (int i=0; i< high_cost_dim_; i++)
    {
        for (int j=0; j< high_cost_dim_; j++)
        {
            int ind = get_high_cost_index(i,j);
            if (high_cost_oriented_padded_footprint_[ind] == 0)
                std::cout << WHITE << static_cast<unsigned>(high_cost_oriented_padded_footprint_[ind]) << " " << RESET;
            else
                std::cout << RED << static_cast<unsigned>(high_cost_oriented_padded_footprint_[ind]) << " " << RESET;

        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
*/
}

void TAstar::get_robot_high_cost_footprint()
{

    //robot dimensions in pixels
    int robot_x_cells = (int)(ceil) (robot_x_size_/world_map_.resolution);
    int robot_y_cells = (int)(ceil) (robot_y_size_/world_map_.resolution);

    // odd number of cells - this way calculations are easier
    if ((robot_x_cells%2) == 0)
        robot_x_cells++;

    if ((robot_y_cells%2) == 0)
        robot_y_cells++;

    cv::Size robot_dimensions(robot_x_cells, robot_y_cells); //robot dimensions in pixels
    int robot_size_4_square = std::max(robot_dimensions.width, robot_dimensions.height); // square from the bigger dimension

    int h  =  (int) ceil(sqrt(robot_x_cells*robot_x_cells + robot_y_cells*robot_y_cells)); // robot diagonal

    // create robot footprint image
    cv::Mat robot_image = cv::Mat(robot_y_cells, robot_x_cells, CV_8UC1, cv::Scalar::all(7));
    //  std::cout << BOLDRED << "\n robot_image.rows: " << robot_image.rows<< " , robot_image.cols: " << robot_image.cols << RESET<< std::endl;
    //  std::cout << YELLOW <<"robot_image:\n \r"<<robot_image << RESET << std::endl;


    /**/  cv::Mat robot_image_pad_high;
    cv::copyMakeBorder(robot_image, robot_image_pad_high,1,1,1,1, cv::BORDER_CONSTANT, level_closest);
    cv::copyMakeBorder(robot_image_pad_high, robot_image_pad_high, 1, 1, 1, 1, cv::BORDER_CONSTANT, level_middle);
    cv::copyMakeBorder(robot_image_pad_high, robot_image_pad_high, 1, 1, 1, 1, cv::BORDER_CONSTANT, level_farthest);

    // std::cout << MAGENTA <<"robot_image pad:\n \r"<<robot_image_pad_high << RESET << std::endl;
    robot_size_4_square =  robot_size_4_square +3+3; //because of the high cost padding

    robot_x_cells = robot_x_cells+6;
    robot_y_cells = robot_y_cells+6;

    //  std::cout << CYAN << "robot_size_4_square: " << robot_size_4_square << RESET << std::endl;
    // calculate offset for padding
    int r = (robot_size_4_square-robot_y_cells)/2;
    int c = (robot_size_4_square-robot_x_cells)/2; // how much to pad to be squared

    //  std::cout << MAGENTA << " ** r: " << r << " , c: " << c << RESET << std::endl;

    //initial padding of the robot (converts to square pad with additional zeros);
    cv::Mat robot_image_pad_square(robot_size_4_square, robot_size_4_square, CV_8UC1, cv::Scalar::all(0)); //square matrix w/ all zeros

    //     std::cout << CYAN <<"robot_image pad:\n \r"<<robot_image_pad_square << RESET << std::endl;

    //Not square
    if( robot_dimensions.width != robot_dimensions.height)
    {
        // std::cout << BOLDRED <<"is not squared" << RESET << std::endl;
        ///cv::Rect roi_sub (cv::Point2i(c, r), cv::Size(robot_image.cols, robot_image.rows)); // region of interest starting from (c,r) and size (cols, rows)
        ///cv::Mat sub_img = robot_image_pad_square(roi_sub); //sub image from that roi
        ///sub_img         += robot_image_pad_high; // update this part w/ robot values (7)
        // now the image is all zeros except for the part where the robot is
        // equivalent to:
        copyMakeBorder(robot_image_pad_high, robot_image_pad_square, r, r, c, c, cv::BORDER_CONSTANT, 0); // add a border (zero padding) to image

    }
    else //If square
    {
        //  std::cout << BOLDGREEN <<"is squared" << RESET << std::endl;

        robot_image_pad_square = robot_image_pad_high; // no need to pad
    }

    h  =  (int) ceil(sqrt(robot_x_cells*robot_x_cells + robot_y_cells*robot_y_cells)); // robot diagonal

    // std::cout << BOLDRED << "\n robot_image_pad_square.rows: " << robot_image_pad_square.rows<< " , robot_image_pad_square.cols: " << robot_image_pad_square.cols << RESET<< std::endl;
    //    std::cout<<"robot_image_pad_square:\n \r"<<robot_image_pad_square<<std::endl;

    // we still need a bit more padding for rotations
    cv::Mat robot_full_padding = robot_image_pad_square;

    if ((h%2) == 0) // if h is even
        h++; // we want odd h

    int border = (h-robot_size_4_square)/2;


    //std::cout << BOLDGREEN <<  "new border: " << border << RESET << std::endl;
    copyMakeBorder(robot_image_pad_square, robot_full_padding, border, border, border, border, cv::BORDER_CONSTANT, 0); // add a border (zero padding) to image

    //  std::cout << BOLDGREEN <<  "robot_full_padding_2 size: " << robot_full_padding.rows << RESET << std::endl;
    //  std::cout<<"robot_full_padding_2:\n \r"<<robot_full_padding<<std::endl;
    //  std::cout << std::endl;std::cout << std::endl;

    // robot_padded_footprint_mat_ = robot_full_padding;

    high_cost_padded_footprint_mat_ = robot_full_padding;

    high_cost_dim_ = robot_full_padding.rows;
    high_cost_footprint_center_ = (int)(high_cost_dim_/2); //center cell

    robot_oriented_padded_footprint_ = (uint8_t*)malloc(biggest_dim_*biggest_dim_*sizeof(uint8_t));
    high_cost_oriented_padded_footprint_ = (uint8_t*)malloc(high_cost_dim_*high_cost_dim_*sizeof(uint8_t));

    ROS_ASSERT_MSG(high_cost_oriented_padded_footprint_ != NULL, "Failed to allocate high_cost_oriented_padded_footprint_"); //if pointer is NULL, quit
    /**************************************/

}

//TODO
void TAstar::get_robot_padded_footprint()
{
    //robot dimensions in pixels
    int robot_x_cells = (int)(ceil) (robot_x_size_/world_map_.resolution);
    int robot_y_cells = (int)(ceil) (robot_y_size_/world_map_.resolution);

    // odd number of cells - this way calculations are easier
    if ((robot_x_cells%2) == 0)
        robot_x_cells++;

    if ((robot_y_cells%2) == 0)
        robot_y_cells++;

    cv::Size robot_dimensions(robot_x_cells, robot_y_cells); //robot dimensions in pixels
    int robot_size_4_square = std::max(robot_dimensions.width, robot_dimensions.height); // square from the bigger dimension

    int h  =  (int) ceil(sqrt(robot_x_cells*robot_x_cells + robot_y_cells*robot_y_cells)); // robot diagonal


    // create robot footprint image
    cv::Mat robot_image = cv::Mat(robot_y_cells, robot_x_cells, CV_8UC1, cv::Scalar::all(7));
    /*std::cout << BOLDRED << "\n robot_image.rows: " << robot_image.rows<< " , robot_image.cols: " << robot_image.cols << RESET<< std::endl;
    std::cout<<"robot_image:\n \r"<<robot_image<<std::endl;*/

    // calculate offset for padding
    int r = (robot_size_4_square-robot_y_cells)/2;
    int c = (robot_size_4_square-robot_x_cells)/2; // how much to pad to be squared

    //std::cout << MAGENTA << " ** r: " << r << " , c: " << c << RESET << std::endl;

    //initial padding of the robot (converts to square pad with additional zeros);
    cv::Mat robot_image_pad_square(robot_size_4_square, robot_size_4_square, CV_8UC1, cv::Scalar::all(0)); //square matrix w/ all zeros
    //Not square
    if( robot_dimensions.width != robot_dimensions.height)
    {
        cv::Rect roi_sub (cv::Point2i(c, r), cv::Size(robot_image.cols, robot_image.rows)); // region of interest starting from (c,r) and size (cols, rows)
        cv::Mat sub_img = robot_image_pad_square(roi_sub); //sub image from that roi
        sub_img         += robot_image; // update this part w/ robot values (7)
        // now the image is all zeros except for the part where the robot is
        // equivalent to:
        //copyMakeBorder(robot_image, robot_image_pad_square, r, r, c, c, cv::BORDER_CONSTANT, 0); // add a border (zero padding) to image

    }
    else //If square
        robot_image_pad_square = robot_image; // no need to pad


    /*  std::cout << BOLDRED << "\n robot_image_pad_square.rows: " << robot_image_pad_square.rows<< " , robot_image_pad_square.cols: " << robot_image_pad_square.cols << RESET<< std::endl;
    std::cout<<"robot_image_pad_square:\n \r"<<robot_image_pad_square<<std::endl;
*/
    // we still need a bit more padding for rotations
    cv::Mat robot_full_padding = robot_image_pad_square;

    if ((h%2) == 0) // if h is even
        h++; // we want odd h

    int border = (h-robot_size_4_square)/2;

    //std::cout << BOLDGREEN <<  "new border: " << border << RESET << std::endl;
    copyMakeBorder(robot_image_pad_square, robot_full_padding, border, border, border, border, cv::BORDER_CONSTANT, 0); // add a border (zero padding) to image

    // std::cout << BOLDGREEN <<  "robot_full_padding_2 size: " << robot_full_padding.rows << RESET << std::endl;
    //std::cout<<"robot_full_padding_2:\n \r"<<robot_full_padding<<std::endl;
    //std::cout << std::endl;std::cout << std::endl;

    robot_padded_footprint_mat_ = robot_full_padding;

    biggest_dim_ = robot_full_padding.rows;
    robot_footprint_center_ = (int)(biggest_dim_/2); //center cell of the robot

    robot_oriented_padded_footprint_ = (uint8_t*)malloc(biggest_dim_*biggest_dim_*sizeof(uint8_t));
    ROS_ASSERT_MSG(robot_oriented_padded_footprint_ != NULL, "Failed to allocate robot_oriented_padded_footprint_"); //if pointer is NULL, quit
}

uint TAstar::get_robot_index(int h, int w, int robot_x_cells)
{
    if (robot_x_cells >0)
        return h*robot_x_cells+w;
    else
    {
        ROS_ERROR_NAMED(logger_name_, "robot_x_cells not set yet");
        ROS_BREAK();

    }
}

uint TAstar::get_pad_index(int h, int w)
{
    return h*biggest_dim_+w;
}

uint TAstar::get_high_cost_index(int h, int w)
{
    return h*high_cost_dim_+w;
}



//*****************************//
//   VALIDITY CHECK FUNCTIONS  //
//*****************************//
// if index is inside the array
bool TAstar::index_2D_is_valid(int index)
{
    if ((index < 0) || (index >= world_map_.height*world_map_.width))
        return false;
    else
        return true;
}

// if index is inside the array
bool TAstar::index_3D_is_valid(int index)
{
    if ((index < 0) || (index >= world_map_.height*world_map_.width*AStarDirCount))
        return false;
    else
        return true;
}

bool TAstar::is_inside_map_boundries(int x, int y)
{
    if (((x < 0) || (x >= world_map_.width)) || ((y < 0) || (y >= world_map_.height)))
        return false;
    else
        return true;
}

//check if point is not obstacle or inflated
bool TAstar::is_valid_point(TGridCoord point, std::string &error)
{
    if (is_inside_map_boundries(point.x, point.y))
    {
        if (GetGridCellState(point) == AStarObstacle)
        {
            error = "Point is Invalid (obstacle cell) ";
            return false;
        }
        else
        {
            if  (GetGridCellState(point) == AStarInflated)
            {
                std::string layer;
                std::ostringstream convert;
                convert << point.z;
                layer = convert.str();
                error = "Point is Inflated Obstacle in layer " + layer;
                return false;
            }
            else
            {
                return true;
            }
        }
    }
    else
    {
        error = "Point is Outside the map";
        return false;
    }
}


//*****************************//
//       CONVERSIONS           //
//*****************************//
// World -> Matrix conversion 2D:
void TAstar::ConvertWorlCoordToMatrix(double wx, double wy, int &mx, int &my)
{
    mx = (int) floor( ((wx - ((double) world_map_.origin_x))/(double)world_map_.resolution));
    my = (int) floor( ((wy - ((double) world_map_.origin_y))/(double)world_map_.resolution));
}

// World -> Matrix conversion 3D:
void TAstar::ConvertWorlCoordToMatrix(double wx, double wy, double wyaw, int &mx, int &my, int &z_layer)
{
    ConvertWorlCoordToMatrix(wx, wy, mx, my);

    if (wyaw <0)
        wyaw = wyaw + 2*M_PI; //we want values between 0-2*PI

    z_layer = n_layer_from_yaw(wyaw);
}

void TAstar::ConvertWorlCoordToMatrix(TWorldPose pose_world, TGridCoord& pose_matrix)
{
    float wx = pose_world.x;
    float wy = pose_world.y;
    float wyaw = pose_world.yaw;

    int mx, my, mz;

    ConvertWorlCoordToMatrix(wx, wy, wyaw, mx, my, mz);

    SetGridPoint(pose_matrix, mx, my, mz);
}

// Matrix -> World conversion 2D:
void TAstar::ConvertMatrixCoordToWorl( int mx, int my, float &wx, float &wy) //returns the center point of the cell - 2D
{
    wx = ((mx+0.5)*world_map_.resolution) + world_map_.origin_x;
    wy = ((my+0.5)*world_map_.resolution) + world_map_.origin_y;
}

// Matrix -> World conversion 3D:
void TAstar::ConvertMatrixCoordToWorl( int mx, int my, int mz, float &wx, float &wy, float &wz) //returns the center point of the cell - 3D
{
    ConvertMatrixCoordToWorl(mx, my, wx, wy);
    wz = yaw_from_n_layer(mz);
}

// Yaw -> z
int TAstar::n_layer_from_yaw(double yaw)
{
    if (yaw < 0)
    {
        int n = yaw/(2*M_PI);
        yaw = yaw -(n-1)*(2*M_PI);
    }

    if (yaw >= 2*M_PI) // > 360 deg
    {
        int n = yaw/(2*M_PI);
        yaw = yaw-n*(2*M_PI); 
    }

    //z_layer = wyaw*8/M_PI; //(z = yaw/360 *16)  // -> 0 : [0-22.5]
    int z_layer = (int) ((yaw + (M_PI/16))*8/M_PI); //(z = (yaw+11.25º)/(360º/16)    // -> 0 : [-11.25 - 11.25]

    if(z_layer > 15)
        z_layer = z_layer-16;

    return z_layer;
}

// z -> yaw
double TAstar::yaw_from_n_layer(int n_layer)
{
    double yaw;

    if (n_layer < 0)
    {
        ROS_ERROR_STREAM_NAMED(logger_name_, "Layer " <<  n_layer << " < 0 -> Quiting");
        ROS_BREAK();
    }

    if(n_layer > 15) // z E [0-15]
    {
        int n = n_layer/16;
        n_layer = n_layer-n*16;
    }

    yaw = M_PI*n_layer/8; //mesma expressao -> assim o valor de z é o centro da "fatia"
    return yaw;
}


//*****************************//
//           ASTAR             //
//*****************************//

//TODO
void TAstar::AStarInit()
{
    AStarMap_.Profiler = TAStarProfiler(); //clear profiler;

   AddToAStarList(AStarMap_.InitialPoint);

   int ind = Get_index(AStarMap_.InitialPoint.z,AStarMap_.InitialPoint.y,AStarMap_.InitialPoint.x, "AStarInit()");
   AStarMap_.Grid[ind].H = CalcH(AStarMap_.InitialPoint, AStarMap_.TargetPoint);

} // AStarInit

bool TAstar::AStarGo(int maxIter, std::string &error_str)
{

    bool success = false;

    ROS_DEBUG_NAMED(logger_name_, "Planner is trying to find a path");


   // int ind = Get_index(AStarMap_.TargetPoint.z,AStarMap_.TargetPoint.y,AStarMap_.TargetPoint.x);

    if (GetGridCellState(AStarMap_.TargetPoint) == AStarClosed)
        ROS_ERROR_NAMED(logger_name_, "Closed point - path will not be valid");

    AStarInit();

    while (true)
    {
        if ((maxIter > 0) && (AStarMap_.Profiler.iter > maxIter))
        {
            std::string iterations;
            std::ostringstream convert;
            convert << maxIter;
            iterations = convert.str() + " iterations";

            error_str =  "Couldn't find path in " + iterations;
            ROS_WARN_STREAM_NAMED(logger_name_, error_str);
            break;
        }

        if (AStarStep() > 2*fixp_1)
        {
            error_str =  "Couldn't find path :(";
            ROS_WARN_STREAM_NAMED(logger_name_, error_str);
            break;
        }

        //check for neighbours of target Point!!!!!
        if(GetGridCellState(AStarMap_.ActualTargetPoint) == AStarClosed)
        {
          /*  if (AStarMap_.Profiler.iter <= 1) //understand why this is happening
            {
                ROS_WARN_STREAM_NAMED(logger_name_, "Couldn't find path :( (only " << AStarMap_.Profiler.iter <<" iteration(s))");
            }
            else
            {*/
                getPath();
                ROS_INFO_STREAM_NAMED(logger_name_, "Path found :) (" << AStarMap_.Profiler.iter << " iterations)");
                success = true;
                error_str = "NO ERRORS! Path found :)";
           // }
            break;
        }

        if (AStarMap_.HeapArray.count == 0)
        {
            error_str =  "Couldn't find path :(";
            ROS_ERROR_STREAM_NAMED(logger_name_, error_str + " - Map.HeapArray.count == 0");
            break;
        }
    }
    AStarMap_.Profiler.iter = 0;

    return success;
}//A_starGo

int TAstar::AStarStep()
{

    TGridCoord curPnt, NewPnt;
    int i, ith, Visits, newG, result, Penalty;

    result = 0;

    AStarMap_.Profiler.iter++;


    AStarMap_.Profiler.HeapArrayTotal += AStarMap_.HeapArray.count;

    //e: node examined. Remove from open, set it to closed
    RemoveBestFromAStarList(curPnt);


    int ind = Get_index(curPnt, "AStarStep(curPnt)");
    SetGridCellState(curPnt, AStarClosed); //(CLOSED = 2)
    //point is added to CLOSED LIST


    //is the current point the goal (d==0) or close (d==1)?
    int delta_x, delta_y, delta_z;
    delta_x = abs(curPnt.x - AStarMap_.TargetPoint.x);
    delta_y = abs(curPnt.y - AStarMap_.TargetPoint.y);
    delta_z = abs(curPnt.z - AStarMap_.TargetPoint.z);

    int d_x = 0, d_y = 0;

    if (!stop_at_exact_target_) // can stop at neighbors
    {
        if ((AStarMap_.TargetPoint.z == 0) || (AStarMap_.TargetPoint.z == 8))
            d_x = 2;
        else if ((AStarMap_.TargetPoint.z == 4) || (AStarMap_.TargetPoint.z == 12))
            d_y = 2;
    }

    //Finished
    if(((delta_x <= d_x) && (delta_y <= d_y)) && (curPnt.z == AStarMap_.TargetPoint.z))
    {
        //ROS_DEBUG_STREAM_NAMED(logger_name_, " > delta_x: "<< delta_x <<" | d_x: "<< d_x);
        //ROS_DEBUG_STREAM_NAMED(logger_name_, " > delta_y: "<< delta_y <<" | d_y: "<< d_y);

        ind = Get_index(curPnt,  "AStarStep(curPnt)2");
        result = AStarMap_.Grid[ind].H;
        AStarMap_.ActualTargetPoint = curPnt;
        return result;
    }

    Visits = sizeof(VisitNeighbours)/sizeof(VisitNeighbours[0]); //6 visits

    //because we don't want to move at all in the direction where there's no laser -> limit the search
    //if we can move a litlle bit even without laser, use penalty instead, without limiting
    int in=0;
    if ((!use_frontal_laser) && (use_back_laser))
    {
        Visits=6;
        in=3;
    }

    if ((!use_back_laser) && (use_frontal_laser))
    {
        Visits=3;
        in=0;
    }

    if ((!use_back_laser) && (!use_frontal_laser))
    {
        Visits=0;
        in=0;
    }

    int heading_change_cost; //adaptative cost to penalize changing orientation
    for (int i=in; i<Visits;i++ )
    {
        // i=5 (-1) |1 |         | 1| i=2 (+1)
        // i=3 (-1) |0 |   |x|   | 0| i=0 (+1)
        // i=4 (-1) |15|         |15| i=1 (+1)
        // meaning:
        // i=0 // go forward on current layer
        // i=1 // rotate forward clockwise (previous layer)
        // i=2 // rotate forward counter-clockwise (next layer)
        // i=3 // go backwards on current layer
        // i=4 // rotate backwards counter-clockwise (next layer)
        // i=5 // rotate backwards clockwise (previous layer)

        if (i==0 || i==3) // if maintain same orientation
            heading_change_cost = 1;
        else // if there's a change of heading
            heading_change_cost = 2; // penalize it


        ith = ((curPnt.z + VisitNeighbours[i]) & (AStarDirCount-1)); //index of neighbour to visit

        NewPnt.x = curPnt.x + SixteenWayNeighbours_[ith].x * VisitNeighboursSign[i];
        NewPnt.y = curPnt.y + SixteenWayNeighbours_[ith].y * VisitNeighboursSign[i];
        NewPnt.z = ith;

        //std::cout<<BOLDGREEN <<" 6.6 > = visit neighbour: ["<< NewPnt.z <<"]["<< NewPnt.y <<"]["<< NewPnt.x <<"] " <<RESET <<std::endl;

        //check if new neighbour is valid
        std::string str = "AStep: NewPnt validity";
        if(is_valid_point(NewPnt, str))
        {
            ind = Get_index(NewPnt,"AStarStep(NewPnt)");

            //if we can move a litlle bit even without laser, use penalty instead
            /* if ((!use_frontal_laser && (VisitNeighboursSign[i] > 0)) || (!use_back_laser && (VisitNeighboursSign[i] < 0)))
        {
            Penalty = AStarMap_.Penalty;
          //  std::cout << BOLDRED << "Penalizing: " << Penalty << RESET << std::endl;
        }
        else*/
            Penalty = 1;


            switch (AStarMap_.Grid[ind].State)
            {
            case AStarClosed: // do nothing on a closed cell
                break;
            case AStarVirgin:
                AStarMap_.Grid[ind].ParentPoint = curPnt; //parent of newPnt is curPnt
                AStarMap_.Grid[ind].G = AStarMap_.Grid[Get_index(curPnt,"AStarStep(currPointVirgin)")].G +  Penalty * SixteenWayNeighbours_[ith].dist*heading_change_cost + AStarMap_.Grid[ind].Cost*cost_scale;
                AStarMap_.Grid[ind].H = CalcH(NewPnt, AStarMap_.TargetPoint); //+cell_cost[cell];
                AddToAStarList(NewPnt); // -> set new neighbour to OPEN
                break;
            case AStarObstacle: // do nothing on an obstacle cell
                break;
            case AStarOpen:
                newG = AStarMap_.Grid[Get_index(curPnt ,"AStarStep(curPnt Open)")].G + Penalty * SixteenWayNeighbours_[ith].dist*heading_change_cost + AStarMap_.Grid[ind].Cost*cost_scale;
                if (newG < AStarMap_.Grid[ind].G) //e: if new G is lower than previous G
                {
                    AStarMap_.Grid[ind].G = newG;//e: replace with smaller G
                    AStarMap_.Grid[ind].ParentPoint = curPnt;//e: and update its parent
                    UpdateHeapPositionByPromotion(AStarMap_.Grid[ind].HeapIdx);
                }
                break;
            }
        }

    }
    return result;
}

void TAstar::AStarClear()
{
    ClearGridState();
    AStarMap_.HeapArray.count = 0;
    AStarMap_.Profiler.iter = 0;
    AStarMap_.HeapArray.count = 0;
}

void TAstar::AddToAStarList(TGridCoord Pnt)
{

    if (AStarMap_.HeapArray.count >= AStarHeapArraySize)
    {   
        //ROS_DEBUG_STREAM_NAMED(logger_name_, "array is full" << AStarMap_.HeapArray.count);
        return;
    }

    AStarMap_.Profiler.AddToAStarList_count++;
 // update the grid state
    SetGridCellState(Pnt, AStarOpen);

    // insert at the bottom of the heap
    int idx = AStarMap_.HeapArray.count; //number of nodes in the heap
    int ind = Get_index(Pnt.z, Pnt.y, Pnt.x, "AddToAStarList()");
    AStarMap_.HeapArray.data[idx] = &AStarMap_.Grid[ind];

    AStarMap_.Grid[ind].HeapIdx = idx;
    AStarMap_.HeapArray.count++;


    UpdateHeapPositionByPromotion(idx);

} //AddToAStarList

void TAstar::RemoveBestFromAStarList(TGridCoord &Pnt )
{

    AStarMap_.Profiler.RemoveBestFromAStarList_count++;

    // return the first node
    Pnt =  AStarMap_.HeapArray.data[0]->MyCoord; //coord of the lowest node

    // move the last node into the first position //e: why?
    AStarMap_.HeapArray.data[AStarMap_.HeapArray.count-1]->HeapIdx = 0;
    AStarMap_.HeapArray.data[0] = AStarMap_.HeapArray.data[AStarMap_.HeapArray.count-1];
    // update the array size
    AStarMap_.HeapArray.count--;

    // PrintHeapArray(Map);
    // re-sort that "first" node
    UpdateHeapPositionByDemotion(0);

}

void TAstar::getPath()
{
    // DELETE
    if ((AStarMap_.ActualTargetPoint.x == AStarMap_.TargetPoint.x) && (AStarMap_.ActualTargetPoint.y == AStarMap_.TargetPoint.y))
        ROS_DEBUG_NAMED(logger_name_, "Target is Exact Point");
    else
        ROS_DEBUG_NAMED(logger_name_, "Target is NOT Exact Point");

    TGridCoord Pnt, nxt_Pnt;
    Pnt = AStarMap_.ActualTargetPoint;
    TGridCoord grid_points[150000]; //iter+1 // grid points to path points?

    nxt_Pnt = Pnt;

    int x, y, z; // IS THIS USED???
    x = Pnt.x;
    y = Pnt.y;
    z = Pnt.z;

    int i=0;
    //  grid_points[0] = Pnt; // use ActualTargetPoint; if stop_at_exact_target TargetPoint= Actual
    grid_points[0] = AStarMap_.TargetPoint; // use TargetPoint; if stop_at_exact_target TargetPoint= Actual

    //sometimes code gets stuck inside this while!
    // while not the initial point
    while(!(((Pnt.x == AStarMap_.InitialPoint.x) && (Pnt.y == AStarMap_.InitialPoint.y)) && (Pnt.z == AStarMap_.InitialPoint.z)) )
    {
        int ind = Get_index(Pnt, "getPath()");
        Pnt = AStarMap_.Grid[ind].ParentPoint;

        i++;
        grid_points[i] = Pnt;

        // CHECK THIS!!!!!
        if (i>150000) //to prevent getting stuck in infinite loop
        {
            ROS_ERROR_NAMED(logger_name_, "Path is not valid! :(");
            return;
        }

    }

    int n_points = i+1;
    float wx,wy,wz;

    ROS_DEBUG_STREAM_NAMED(logger_name_, "Path has " << n_points <<" points");
    path_msg_.header.frame_id = "map";
    path_msg_.poses.resize(n_points);

    last_path_number_of_points_ = n_points;

    for (int j = 0; j<= i; j++)
    {
        if (j==0) // we want the first point to be the robot current position //NOW WE DON'T HAVE THIS UPDATED!!!!
        {
            wx = robot_init_world_pose_.x;
            wy = robot_init_world_pose_.y;
            wz = robot_init_world_pose_.yaw;
        }
        else
        {
            ConvertMatrixCoordToWorl(grid_points[i-j].x, grid_points[i-j].y, grid_points[i-j].z, wx, wy, wz);
        }

        tf::Quaternion quat = tf::createQuaternionFromYaw(wz);
        path_msg_.poses[j].pose.position.x = wx;
        path_msg_.poses[j].pose.position.y = wy;
        path_msg_.poses[j].pose.position.z = 0;
        path_msg_.poses[j].pose.orientation.x = quat.x();
        path_msg_.poses[j].pose.orientation.y = quat.y();
        path_msg_.poses[j].pose.orientation.z = quat.z();
        path_msg_.poses[j].pose.orientation.w = quat.w();

        send_arrows_array(wx, wy, wz);
    }
    return;

}


//*****************************//
//           HEAP              //
//*****************************//

void TAstar::UpdateHeapPositionByPromotion(int idx)
{
    int parent_idx, node_cost;

    if (idx == 0) // first node - no promotion!
        return;

    // calc node cost
    node_cost = CalcHeapCost(idx); //node_cost = G+H;

    // repeat until we can promote no longer
    while(true)
    {
        // if we are on the first node, there is no way to promote
        if (idx == 0)
            return;

        parent_idx = (idx-1)/2;

        // if the parent is better than we are, there will be no promotion
        if  (CalcHeapCost(parent_idx) < node_cost)
            return;

        // if not, just promote it
        SwapHeapElements(idx, parent_idx);
        idx = parent_idx;
    }
}

void TAstar::UpdateHeapPositionByDemotion(int idx)
{
     int cost_child1, cost_child2, cost_parent;
     int idx_child, new_idx;

     cost_parent = CalcHeapCost(idx);

     while(true)
     {
        idx_child = idx*2+1;

        // if the node has no childs, there is no way to demote
        if (idx_child >= AStarMap_.HeapArray.count)
            return;

        //calc our cost and the first node cost
        cost_child1 = CalcHeapCost(idx_child);

        // if there is only one child, just compare with this one
        if ((idx_child + 1)  >= AStarMap_.HeapArray.count)
        {
            if (cost_parent < cost_child1) // if we are better than this child, then no demotion
                return;
            else // demote parent node
            {
                SwapHeapElements(idx, idx_child);
                return;
            }
        }
        else //two childs
        {
            // calc the second node cost
            cost_child2 = CalcHeapCost(idx_child + 1);

            // select the best node to demote to
            new_idx = idx;

            if (cost_child2 < cost_parent)
            {
                if (cost_child1 < cost_child2)
                    new_idx = idx_child; //first child is best node
                else
                    new_idx = idx_child +1; //second child is best node
            }
            else
            {
                if (cost_child1 < cost_parent)
                     new_idx = idx_child; //first child is best node
            }

            // if there is no better child, just return
            if (new_idx == idx) // parent is best node
                return;

            // if not, just promote it
            SwapHeapElements(idx, new_idx);
            idx = new_idx; //update node until there's no childs to demote
        }
        //return;
     }
}

void TAstar::SwapHeapElements(int idx1, int idx2)
{
    TAStarCell *ptr1, *ptr2;

    ptr1 = AStarMap_.HeapArray.data[idx1];
    ptr2 = AStarMap_.HeapArray.data[idx2];

    AStarMap_.HeapArray.data[idx1] = ptr2;
    AStarMap_.HeapArray.data[idx2] = ptr1;
}

int TAstar::CalcHeapCost(int idx)
{
    int F = AStarMap_.HeapArray.data[idx]->G + AStarMap_.HeapArray.data[idx]->H;
    return F;
}


//*****************************//
//            OTHER            //
//*****************************//

void TAstar::ClearGridState()
{

    if ((world_map_.height != 0) && (world_map_.height != 0))
    {
        int ind;
        // clear GridState
        for(int l= 0 ; l < AStarDirCount; l++)
        {
            for(int h=0; h<world_map_.height; h++) //read from the bottom up...
            {
                for(int w=0; w<world_map_.width; w++) //from left to right
                {
                    ind = Get_index(l,h,w, "ClearGridState()");

                    if ((AStarMap_.Grid[ind].State) == AStarClosed ||(AStarMap_.Grid[ind].State) == AStarOpen )
                        AStarMap_.Grid[ind].State = AStarVirgin; // clear closed and open cells

                }
            }
        }
        ROS_DEBUG_NAMED(logger_name_, "Planner is ready to receive a new goal");
        // cleared = true;
    }
    else
        ROS_ERROR_NAMED(logger_name_, "Couldn't clear GridState because map size is 0");
}

int TAstar::CalcH(TGridCoord Pi, TGridCoord Pf)
{
    int x, y ;
    x = abs(Pi.x - Pf.x);
    y = abs(Pi.y - Pf.y);

    //return CalcHCache[abs(Pf.y-Pi.y)][abs(Pi.x - Pf.x)];
    //return CalcHCache[y][x];
    return CHCache_[y][x];
    //return round(sqrt(sqr(Pi.x-Pf.x) + sqr(Pi.y-Pf.y)) * Map.EucliDistK * FixedPointConst);

} // CalcH

//TODO
void TAstar::obstacle_inflation(int layer, int h, int w)
{

    for (int ih = -robot_footprint_center_; ih <= robot_footprint_center_; ih++)
    {
        for (int iw = -robot_footprint_center_; iw <= robot_footprint_center_; iw++)
        {
            if (robot_oriented_padded_footprint_[get_pad_index(ih+robot_footprint_center_, iw+robot_footprint_center_)] != 0)
            {

                if (((w+iw)>=0) && ((w+iw)<world_map_.width) && ((h+ih)>=0) && ((h+ih)<world_map_.height))//(index_3D_is_valid(ind))
                {
                    int ind = Get_index(layer, h+ih, w+iw, "obstacle_inflation");

                    if ((AStarMap_.Grid[ind].State != AStarObstacle) && (AStarMap_.Grid[ind].State != AStarInflated))
                    {
                        SetGridCellState(w+iw, h+ih, layer, AStarInflated);
                        SetGridCellState(w+iw, h+ih, layer+8, AStarInflated);
                        n_inflated_cells_++;
                    }
                }
            }
        }
    }
}

void TAstar::high_cost_inflation(int layer, int h, int w)
{

    for (int ih = -high_cost_footprint_center_; ih <= high_cost_footprint_center_; ih++)
    {
        for (int iw = -high_cost_footprint_center_; iw <= high_cost_footprint_center_; iw++)
        {
            if (high_cost_oriented_padded_footprint_[get_high_cost_index(ih+high_cost_footprint_center_, iw+high_cost_footprint_center_)] != 0)
            {

                if (((w+iw)>=0) && ((w+iw)<world_map_.width) && ((h+ih)>=0) && ((h+ih)<world_map_.height))//(index_3D_is_valid(ind))
                {
                    int ind = Get_index(layer, h+ih, w+iw, "high_cost_inflation");

                    if ((AStarMap_.Grid[ind].State != AStarObstacle))// && (AStarMap_.Grid[ind].State != AStarInflated))
                    {
                        SetGridCellCost(w+iw, h+ih, layer, high_cost_oriented_padded_footprint_[get_high_cost_index(ih+high_cost_footprint_center_, iw+high_cost_footprint_center_)]);
                        SetGridCellCost(w+iw, h+ih, layer+8, high_cost_oriented_padded_footprint_[get_high_cost_index(ih+high_cost_footprint_center_, iw+high_cost_footprint_center_)]);
                   }
                }
            }
        }
    }
}

void TAstar::send_arrows_array(float wx, float wy, float wz)
{
    //temporary marker
    visualization_msgs::Marker marker_arrow;

    marker_arrow.header.frame_id = "map";
    marker_arrow.header.stamp = ros::Time().now();

    marker_arrow.ns = "arrows_namespace";
    marker_arrow.id = ++marker_id_;
    marker_arrow.type = visualization_msgs::Marker::ARROW; // shape;
    marker_arrow.action = visualization_msgs::Marker::ADD;

    marker_arrow.scale.x = world_map_.resolution; //shaft  // 0.1;
    marker_arrow.scale.y = world_map_.resolution;  //head //0.5;
    marker_arrow.scale.z = world_map_.resolution;  //head; //0.25;

    int l = n_layer_from_yaw(wz);

    int color;
    switch (l)
    {
    case 0:
        color = color_layer0;
        break;
    case 1:
        color = color_layer1;
        break;
    case 2:
        color = color_layer2;
        break;
    case 3:
        color = color_layer3;
        break;
    case 4:
        color = color_layer4;
        break;
    case 5:
        color = color_layer5;
        break;
    case 6:
        color = color_layer6;
        break;
    case 7:
        color = color_layer7;
        break;
    case 8:
        color = color_layer0;
        break;
    case 9:
        color = color_layer1;
        break;
    case 10:
        color = color_layer2;
        break;
    case 11:
        color = color_layer3;
        break;
    case 12:
        color = color_layer4;
        break;
    case 13:
        color = color_layer5;
        break;
    case 14:
        color = color_layer6;
        break;
    case 15:
        color = color_layer7;
        break;
    }

    float r,g,b;
    r = ((color >> 16) & 0xFF) / 255.0;
    g = ((color >> 8) & 0xFF) / 255.0;;
    b = ((color) & 0xFF) / 255.0;

    marker_arrow.color.r = r;
    marker_arrow.color.g = g;
    marker_arrow.color.b = b;
    marker_arrow.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = wx;
    p.y = wy;
    p.z = 0;
    marker_arrow.points.push_back(p); //start point

    p.x = wx+world_map_.resolution*cos(wz);
    p.y = wy+world_map_.resolution*sin(wz);
    p.z = 0;
    marker_arrow.points.push_back(p); //end point

    //marker_arrow.lifetime = ros::Duration(0.1);
    marker_array_arrows_.markers.push_back(marker_arrow);
}

void TAstar::delete_arrows_array(int number_of_arrows)
{

     marker_array_arrows_.markers.clear();

    //temporary marker
    visualization_msgs::Marker marker_arrow;

    marker_arrow.header.frame_id = "map";
    marker_arrow.header.stamp = ros::Time().now();

    marker_arrow.ns = "arrows_namespace";
    marker_arrow.type = visualization_msgs::Marker::ARROW; // shape;

    for (int i=0; i< number_of_arrows; i++)
    {
        marker_arrow.id = i+1; //markers start with 1;
        marker_arrow.action = visualization_msgs::Marker::DELETE;
        marker_array_arrows_.markers.push_back(marker_arrow);
    }

    marker_id_ = 0;
}

//pcl::PointCloud<pcl::PointXYZ >::Ptr Cloud_Markers (new pcl::PointCloud<pcl::PointXYZ >);
void  TAstar::add_to_pointCloud(int cx, int cy, int layer, sensor_msgs::PointCloud2& pcd)
{

    //std::cout << BOLDRED << "Pointcloud!!" << RESET << std::endl;
    float wx, wy;
    ConvertMatrixCoordToWorl(cx, cy, wx, wy);

    // cloudMarkers is a "temporary" pointCloudXYZ
    Cloud_Markers->header.frame_id = "map";

    pcl::PointXYZRGB point_pcl;
    point_pcl.x = wx;
    point_pcl.y = wy;
    point_pcl.z = layer*0.1;

    switch(layer)
    {
    case 0:
        point_pcl.r = 0xff;
        point_pcl.g = 0x00;
        point_pcl.b = 0x00;
        break;
    case 4:
        point_pcl.r = 0x00;
        point_pcl.g = 0xff;
        point_pcl.b = 0xff;
        break;
    }

    Cloud_Markers->points.push_back(point_pcl);
    Cloud_Markers->is_dense =false;
    Cloud_Markers->width = Cloud_Markers->points.size();
    Cloud_Markers->height = 1;

}

void TAstar::paramsCB(oea_planner::planner_paramsConfig &config, uint32_t level)
{
    ROS_INFO_NAMED(logger_name_, "Changed Planner parameters!");

    level_closest = config.level_closest;
    level_middle = config.level_middle;
    level_farthest = config.level_farthest;

    cost_scale = config.cost_scale;

    if ( update_grid)
    {
        for(int l=0; l < AStarDirCount/2; l++) //read from the bottom up...
        {
            for(int h=0; h < world_map_.height; h++) //read from the bottom up...
            {
                for(int w=0; w < world_map_.width; w++) //from left to right
                {
                    if ((GetGridCellState(w,h,l) == AStarObstacle))
                    {
                        high_cost_inflation(l,h,w);
                    }
                }
            }
        }
    }
}

