//
// Created by yangt on 1/31/18.
//
#include "simple_car/simple_car_node.hpp"
SimpleCar::SimpleCar()
        : gridmap_(grid_map::GridMap({"obstacle"})) {
    this->enable_auto_drive_ =
            this->nh_.param("/simple_car_node/enable_auto_drive", true);
    this->laser_scan_sub_ = this->nh_.subscribe("/scan",
                                                10,
                                                &SimpleCar::laserScanCb,
                                                this);
    this->cmd_vel_sub_ = this->nh_.subscribe("/cmd_vel",
                                             10,
                                             &SimpleCar::keyCommandCb,
                                             this);
    this->timer_ =
            this->nh_.createTimer(ros::Duration(0.1),
                                  boost::bind(&SimpleCar::timerCb, this));
    this->map_pub_ =
            this->nh_.advertise<nav_msgs::OccupancyGrid>("grid_map",
                                                         1,
                                                         true);
    this->poly_pub_ =
            this->nh_.advertise<geometry_msgs::PolygonStamped>(
                    "front_polygon",
                    1,
                    true);
    this->drive_mode_.steering = 0;
    this->drive_mode_.left_mode = STOP;
    this->drive_mode_.right_mode = STOP;
    ///init grid map
    grid_map::Length map_length(20, 20);
    grid_map::Position pos(0, 0);
    this->gridmap_.setFrameId("laser");
    this->gridmap_.setGeometry(map_length, 0.05);
    this->gridmap_.setPosition(pos);
    this->gridmap_.move(pos);
    this->back_count_ = 0;
    this->go_count_ = 0;
}
SimpleCar::~SimpleCar() {
    this->my_serial_.close();
}

bool SimpleCar::setSerial(std::string port, uint32_t baundRate) {
    if(this->my_serial_.isOpen()) {
        std::string port = this->my_serial_.getPort();
        std::cout << "port" + port <<"is already opened!"<< std::endl;
    } else {
        try {
            this->my_serial_.setPort(port);
            this->my_serial_.setBaudrate( baundRate);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            this->my_serial_.setTimeout(timeout);
            this->my_serial_.open();
        }catch (serial::IOException& e) {
            std::cout << "Unable to open port " << std::endl;
        }
        if(this->my_serial_.isOpen()) {
            std::cout << port << " is opened!"<< std::endl;
        } else {
            std::cout << port << " open failed!"<< std::endl;
            return false;
        }
    }
    return true;
}

void SimpleCar::sendCommand(const DriveMode &cmd) {
    if(this->my_serial_.isOpen()) {
        uint8_t buffer[9] = {0xA5, 0x5A, 0x07, 0xA1, 0, 0, 0, 0, 0xAA};
        buffer[4] = cmd.steering;
        buffer[5] = cmd.right_mode;
        buffer[6] = cmd.left_mode;
        buffer[7] = 0x07 + 0xA1 + buffer[4] + buffer[5] + buffer[6];
        this->my_serial_.write(buffer, 9);
    } else{
        std::cout << "serial port is not opened!" << std::endl;
    }
}

void SimpleCar::keyCommandCb(const geometry_msgs::Twist &vel_msgs) {
    if(vel_msgs.linear.x > 0.1) {
        this->drive_mode_.right_mode = GO;
        this->drive_mode_.left_mode = GO;
    } else if(vel_msgs.linear.x < -0.1) {
        this->drive_mode_.left_mode = BACK;
        this->drive_mode_.right_mode = BACK;
    } else {
        this->drive_mode_.left_mode = STOP;
        this->drive_mode_.right_mode = STOP;
    }
    if(vel_msgs.angular.z > 0.1) {
        this->drive_mode_.steering = TURN_RIGHT;
    } else if(vel_msgs.angular.z < -0.1) {
        this->drive_mode_.steering = TURN_LEFT;
    } else {
        this->drive_mode_.steering = GO_STRIGHT;
    }
}

void SimpleCar::laserScanCb(const sensor_msgs::LaserScan &laser_msgs) {
    this->laser_msg_ = laser_msgs;
}
bool SimpleCar::updateGridMap() {
    if(this->laser_msg_.ranges.empty()) {
        return false;
    } else {
        this->gridmap_.get("obstacle").setConstant(255);
        for(size_t i=0; i<this->laser_msg_.ranges.size(); i++) {
            double angle_increment = this->laser_msg_.angle_increment;
//            double angle = i * 0.00436 - (135.0/180.0)*M_PI;//M_PI_2 - (i - 540) * 0.00436;
            double angle = i * this->laser_msg_.angle_increment - M_PI;
            double dist = this->laser_msg_.ranges.at(i);
            double x = dist * cos(angle);
            double y = dist * sin(angle);
            grid_map::Position pos(x, y);
            if(this->gridmap_.isInside(pos)) {
                this->gridmap_.atPosition("obstacle", pos) = 0;
            }
        }
//        cv::Mat map_cv = hmpl::eigen2cv(this->gridmap_.get("obstacle"));
//        cv::GaussianBlur(map_cv, map_cv, cv::Size(7,7), 0, 0);
        /// rviz display
        nav_msgs::OccupancyGrid map_msg;
        ros::Time time = ros::Time::now();
        this->gridmap_.setTimestamp(time.toNSec());
        grid_map::GridMapRosConverter::toOccupancyGrid(this->gridmap_,
                                                       "obstacle",
                                                       255,
                                                       0,
                                                       map_msg);
        this->map_pub_.publish(map_msg);
    }
}
void SimpleCar::calSteering() {
    if(this->laser_msg_.ranges.empty())
        return;
    if(back_count_ > 1) {
        this->back_count_ --;
        return;
    }
    if (this->go_count_ > 1){
        this->go_count_ --;
        return;
    }
    int obs_pixel = 0;
    this->drive_mode_old_ = this->drive_mode_;
    double resolution = this->gridmap_.getResolution();
    int step_x = (0.6 + 0.6) / (resolution);
    int step_y = (1.2 - 0.0) / (resolution);
    double init_y = 0;
    double init_x = -0.6;
    double obs_y = 0.;
    for(int i = 0; i<step_x; i++) {
        double x = init_x + i * resolution;
        for(int j = 0; j<step_y; j++) {
            double y = init_y + j * resolution;
            grid_map::Position pt(x, y);
            if(this->gridmap_.atPosition("obstacle", pt) < 50) {
                obs_pixel ++;
                obs_y = y;
                if(obs_pixel > 2)
                break;
            }
        }
        if(obs_pixel > 2)
            break;
    }
    if(obs_pixel > 2) { //need to back
        if(obs_y > 0)
            this->drive_mode_.steering = TURN_RIGHT;
        else
            this->drive_mode_.steering = TURN_LEFT;
        this->drive_mode_.right_mode = BACK;
        this->drive_mode_.left_mode = BACK;
        this->back_count_ = 50;
    } else {
        step_x =(0.6 + 0.6) / resolution;
        step_y = (2 - 1.2) / resolution;
        init_y = 1.2;
        init_x = -0.6;
        for(int i = 0; i<step_x; i++) {
            double x = init_x + i * resolution;
            for(int j = 0; j<step_y; j++) {
                double y = init_y + j * resolution;
                grid_map::Position pt(x, y);
                if(this->gridmap_.atPosition("obstacle", pt) < 50) {
                    obs_pixel ++;
                    obs_y = y;
                    break;
                }
            }
            if(obs_pixel > 0)
                break;
        }
        if(obs_pixel > 0) {// need to turn
            obs_pixel = 0;
            step_x =(1.5 - 0.6) / resolution;
            step_y = (2- 0) / resolution;
            init_y = 0;
            init_x = -1.5;
            for(int i = 0; i<step_x; i++) {
                double x = init_x + i * resolution;
                for(int j = 0; j<step_y; j++) {
                    double y = init_y + j * resolution;
                    grid_map::Position pt(x, y);
                    if(this->gridmap_.atPosition("obstacle", pt) < 50) {
                        obs_pixel ++;
                        obs_y = y;
                        break;
                    }
                }
                if(obs_pixel > 0)
                    break;
            }
            if(obs_pixel > 0) {
                obs_pixel = 0;
                step_x =(1.5 - 0.6) / resolution;
                step_y = (2- 0) / resolution;
                init_y = 0;
                init_x = 0.6;
                for(int i = 0; i<step_x; i++) {
                    double x = init_x + i * resolution;
                    for(int j = 0; j<step_y; j++) {
                        double y = init_y + j * resolution;
                        grid_map::Position pt(x, y);
                        if(this->gridmap_.atPosition("obstacle", pt) < 50) {
                            obs_pixel ++;
                            obs_y = y;
                            break;
                        }
                    }
                    if(obs_pixel > 0)
                        break;
                }
                if(obs_pixel > 0) {
                    if(obs_y > 0)
                        this->drive_mode_.steering = TURN_RIGHT;
                    else
                        this->drive_mode_.steering = TURN_LEFT;
                    this->drive_mode_.right_mode = BACK;
                    this->drive_mode_.left_mode = BACK;
                    this->back_count_ = 50;
                } else { // turn left
                    this->drive_mode_.steering = TURN_LEFT;
                    this->drive_mode_.right_mode = GO;
                    this->drive_mode_.left_mode = GO;
                }
            } else { // turn right
                this->drive_mode_.steering = TURN_RIGHT;
                this->drive_mode_.right_mode = GO;
                this->drive_mode_.left_mode = GO;
            }
        } else { // go straight
            this->drive_mode_.steering = GO_STRIGHT;
            this->drive_mode_.right_mode = GO;
            this->drive_mode_.left_mode = GO;
        }
    }
    if(this->drive_mode_old_.right_mode == BACK &&
            this->drive_mode_.right_mode == GO &&
            (this->drive_mode_.steering == this->drive_mode_old_.steering ||
                    this->drive_mode_.steering == GO_STRIGHT)) {
        if(this->drive_mode_old_.steering == TURN_RIGHT)
            this->drive_mode_.steering = TURN_LEFT;
        else {
            this->drive_mode_.steering = TURN_RIGHT;
        }
        this->go_count_ = 20;
    }
//    grid_map::Polygon polygon;
//    polygon.setFrameId(this->gridmap_.getFrameId());
//    geometry_msgs::PolygonStamped p_message;
//    p_message.header.stamp = ros::Time::now();
//    grid_map::PolygonRosConverter::toMessage(polygon, p_message);
//    this->poly_pub_.publish(p_message);
}

void SimpleCar::timerCb() {
    if(this->enable_auto_drive_) {
        if(this->updateGridMap()) {
            this->calSteering();
            std::cout << "steer : "
                      << (int)this->drive_mode_.steering
                      << "speed: "
                      << (int)this->drive_mode_.right_mode
                      << std::endl;
        }
        this->sendCommand(this->drive_mode_);
    } else {
        this->sendCommand(this->drive_mode_);
        this->drive_mode_.right_mode = STOP;
        this->drive_mode_.left_mode = STOP;
        this->drive_mode_.steering = GO_STRIGHT;
    }
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "simple_car_node");
    SimpleCar simple_car;
    simple_car.setSerial();
    ros::spin();
}