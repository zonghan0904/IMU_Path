# include "ros/ros.h"
# include "sensor_msgs/Imu.h"
# include "geometry_msgs/Vector3.h"
# include "geometry_msgs/Point.h"
# include "visualization_msgs/Marker.h"
# include "Eigen/Dense"

geometry_msgs::Vector3 _gyr, _acc, gyr, acc;
Eigen::Vector3d mid_gyr, mid_acc, acc_g, gyr_g;
Eigen::Vector3d g_g(0, 0, 9.81), s_g(0, 0, 0), v_g(0, 0, 0);
Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
double past = 0, now = 0;
double dt = 0;
bool sampled = false;

void callback(const sensor_msgs::Imu::ConstPtr& msg){
    now = msg -> header.stamp.sec + double(msg -> header.stamp.nsec) * 1e-9;
    gyr = msg -> angular_velocity;
    acc = msg -> linear_acceleration;

    if (past != 0){
	dt = now - past;
	mid_gyr << (_gyr.x + gyr.x) / 2, (_gyr.y + gyr.y) / 2, (_gyr.z + gyr.z) / 2;
	mid_acc << (_acc.x + acc.x) / 2, (_acc.y + acc.y) / 2, (_acc.z + acc.z) / 2;
    }
    else{
	dt = 0;
    }

    _gyr = gyr;
    _acc = acc;
    past = now;
    sampled = true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "visualize_path_node");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Subscriber sub = n.subscribe("imu/data", 1000, callback);
    ros::Rate rate(200);
    double start_time = ros::Time::now().toSec(), now_time;
    bool end = false;
    visualization_msgs::Marker line_strip;

    while (ros::ok()){
    	line_strip.header.frame_id = "/map";
    	line_strip.header.stamp = ros::Time::now();
    	line_strip.ns = "points_and_lines";
    	line_strip.action = visualization_msgs::Marker::ADD;
    	line_strip.pose.orientation.w = 1.0;
    	line_strip.id = 1;
    	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    	line_strip.scale.x = 0.1;
    	line_strip.color.b = 1.0;
    	line_strip.color.a = 1.0;

	if (dt != 0 && sampled == true){
	    Eigen::Matrix3d B;
	    B <<                 0, -mid_gyr.z() * dt,  mid_gyr.y() * dt,
	          mid_gyr.z() * dt,                 0, -mid_gyr.x() * dt,
	         -mid_gyr.y() * dt, -mid_gyr.x() * dt,                 0;

	    double sigma = (mid_gyr * dt).norm();
	    C = C * (I + (sin(sigma) / sigma) * B + ((1 - cos(sigma))/ pow(sigma, 2.0)) * B * B);
	    acc_g = C * mid_acc;
	    v_g = v_g + dt * (acc_g - g_g);
	    s_g = s_g + dt * v_g;

	    geometry_msgs::Point p;
	    p.x = s_g.x();
	    p.y = s_g.y();
	    p.z = s_g.z();

	    line_strip.points.push_back(p);
	    sampled = false;
	    marker_pub.publish(line_strip);
	}
	ros::spinOnce();
	rate.sleep();
    }
    return 0;
}
