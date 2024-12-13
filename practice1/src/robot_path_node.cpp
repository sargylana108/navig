#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

nav_msgs::Odometry odom;

// Функция для приёма одометрии и сохранения её в переменную odom
void odomCallback(const nav_msgs::Odometry &msg) {
    odom = msg;
}


int main(int argc, char** argv) {

    // Инициализация нового узла для публикации пути
    ros::init(argc, argv, "odom_path");
    ros::NodeHandle nh("~");

    // Инициализация "подписчика" на одометрию
    ros::Subscriber subOdom = nh.subscribe("/odom", 10, odomCallback);
    // Инициализация "издателя" пути
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 10);

    nav_msgs::Path odom_path;

    ros::Rate r(10);

    while (ros::ok()) {
        // Ожидаем первой одометрии, до этого момента спим
        if (odom.header.frame_id.empty()) {
            ros::spinOnce();
            r.sleep();
            continue;
        }

        // Публикуем путь
        odom_path.header = odom.header;
        geometry_msgs::PoseStamped pose;
        pose.header = odom_path.header;
        pose.pose = odom.pose.pose;
        odom_path.poses.push_back(pose);
        pubPath.publish(odom_path);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}