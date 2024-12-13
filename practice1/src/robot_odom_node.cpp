#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <vector>

using namespace std;

#define PI 3.14159265

const float WHEEL_RADIUS = 0.05; // Радиус колёс
const float WHEEL_DISTANCE = 0.35; // Расстояние между колесами робота


// Глобальные переменные
ros::Time prev_update_time;  // Время предыдущего обновления
geometry_msgs::Twist vel;     // Текущая скорость робота
nav_msgs::Odometry odom;      // Текущая одометрия робота
vector<float> prev_pose = {0, 0, 0};  // Положение робота - {x,y, theta}

ros::Publisher pubOdom;     // Структура "издателя" (publisher-а) информации об одометрии
ros::Publisher pubVel;  // Структура "издателя" (publisher-а) скоростей

ros::Subscriber subVel; // Структура "подписчика" (subscriber-а) скоростей

bool use_keyboard_control = false;

// Объявление функций
void velCallback(const geometry_msgs::Twist &msg);
void updateOmnidirectionalRobotState(tf::TransformBroadcaster &tf_broadcaster);
void updateOdometry(ros::Duration diff_time, ros::Time current_time, geometry_msgs::Twist control_velocity, vector<float> previous_pose);
void controlLogic(float &tick);


int main(int argc, char **argv)
{
    // Инициализация узла ROS
    ros::init(argc, argv, "odom_fake_node");
    ros::NodeHandle nh("~");

    // Инициализация "подписчика" на скорость робота
    subVel = nh.subscribe("/cmd_vel", 10, velCallback);

    // Инициализация "издателя" информации о положении робота
    pubOdom = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    // Инициализация "издателя" информации о скорости робота
    pubVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Структура для публикации информации между матрицами перемещения робота (odom -> base_link) 
    tf::TransformBroadcaster tf_broadcaster;

    // Инициализация начальной позиции робота из параметров ROS
    ros::param::get("init_pose_x", prev_pose[0]);
    ros::param::get("init_pose_y", prev_pose[1]);
    ros::param::get("init_pose_theta", prev_pose[2]);

    // Параметр, который показывает используем ли мы клавиатуру робота или нет
    ros::param::get("use_keyboard_control", use_keyboard_control);

    // Установка частоты цикла управления
    ros::Rate r(1000);

    // Инициализация счетчика тактов
    float tick = 0;

    // Основной цикл управления
    while (nh.ok())
    {
        // Обновление состояния робота и логики управления
        updateOmnidirectionalRobotState(tf_broadcaster);

        if (!use_keyboard_control){
            controlLogic(tick);
            tick++;
        }

        // Обработка обратных вызовов ROS и ожидание следующей итерации
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}


/**
 * @brief Функция логики управления. Здесь мы отправляем информацию о желаемой скорости 
 * в конкретный момент времени на топик /cmd_vel. Данную часть кода менять не нужно.
 *
 * @param tf_broadcaster структура TransformBroadcaster для публикации перемещения систем координат
 */
void controlLogic(float &i)
{
    geometry_msgs::Twist cmd;

    if (i > 0 && i < 4000)
    {
        cmd.linear.x = 0.1;
        cmd.linear.y = 0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = 0.1;
    }
    else if (i >= 4000 && i < 5000)
    {
        cmd.linear.x = 0.1;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = 0.0;
    }
    else if (i >= 5000 && i < 9710)
    {
        cmd.linear.x = 0.1;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = 0.1;
    }
    else if (i >= 9710 && i < 10710)
    {
        cmd.linear.x = 0.1;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = 0.0;
    }
    else if (i >= 10710 && i < 14700)
    {
        cmd.linear.x = 0.1;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = 0.1;
    }
    else if (i >= 14690 && i < 16690)
    {
        cmd.linear.x = 0.1;
        cmd.linear.y = 0.3;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = 0.3;
    }
    else if (i == 16690)
    {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = 0;
    }
    pubVel.publish(cmd);
}


/**
 * @brief Рассчитывает прямую кинематику двухколёсного робота с дифф. приводом.
 *
 * Эта функция вычисляет следующее состояние робота на основе его текущего состояния и скоростей колес.
 * Если скорости левого и правого колес равны, предполагается чистое линейное движение.
 *
 * @param x Текущая x-координата робота в глобальной системе координат.
 * @param y Текущая y-координата робота в глобальной системе координат.
 * @param theta Текущая ориентация (угол поворота) робота в глобальной системе координат.
 * @param dt Шаг времени для обновления кинематики.
 * @param vl Скорость левого колеса.
 * @param vr Скорость правого колеса.
 *
 * @return Вектор, содержащий обновленные значения x, y и theta (т.е. новое положение робота).
 */
vector<float> forwardKinematics(float x, float y, float theta, double dt, float vl, float vr)
{
    
    /************************************/
    /****ПОСЧИТАЙТЕ ПРЯМУЮ КИНЕМАТИКУ****/
    /************************************/

    // Линейная скорость
    float v_l = vl * WHEEL_RADIUS;
    float v_r = vr * WHEEL_RADIUS;
    float v = (v_r + v_l) / 2.0;
    float omega = (v_r - v_l) / WHEEL_DISTANCE;

    // Обновляем координаты
    float dx = x + v * cos(theta) * dt;
    float dy = y + v * sin(theta) * dt;
    float dtheta = theta + omega * dt;

    vector<float> next_state {dx, dy, dtheta};
    return next_state;
}

// Сохраняем текущую скорость робота в переменную vel
void velCallback(const geometry_msgs::Twist &msg)
{
    vel = msg;
}

// Обновление состояния робота, одометрии и публикация информации
void updateOmnidirectionalRobotState(tf::TransformBroadcaster &tf_broadcaster)
{
    
    // Получение текущего времени относительно ROS
    ros::Time current_time = ros::Time::now();

    // Рассчет разницы во времени между прошлым и текущим временем обновления
    ros::Duration elapsed_time = current_time - prev_update_time;
    prev_update_time = current_time;

    // Обновление одометрии робота
    updateOdometry(elapsed_time, current_time, vel, prev_pose);

    // Создание и публикация перемещения положения робота относительно глобальных координат
    geometry_msgs::TransformStamped current_pose;
    current_pose.header = odom.header;
    current_pose.transform.translation.x = odom.pose.pose.position.x;
    current_pose.transform.translation.y = odom.pose.pose.position.y;
    current_pose.transform.translation.z = odom.pose.pose.position.z;
    current_pose.transform.rotation = odom.pose.pose.orientation;
    current_pose.child_frame_id = odom.child_frame_id;

    tf_broadcaster.sendTransform(current_pose);
}

/**
 * @brief Обновляет данные одометрии робота на основе управляющей скорости и предыдущего положения.
 *
 * Эта функция рассчитывает новое положение и ориентацию робота на основе управляющей скорости и
 * предыдущего положения. После вычисления новых значений, они присваиваются соответствующим полям
 * сообщения об одометрии (odom), которое затем публикуется.
 *
 * @param diff_time Разница времени между текущим и предыдущим обновлением одометрии.
 * @param current_time Текущее время.
 * @param control_velocity Требуемая скорость робота (control input).
 * @param previous_pose Предыдущее положение робота в виде вектора [x, y, theta].
 */

void updateOdometry(ros::Duration diff_time, ros::Time current_time, geometry_msgs::Twist control_velocity, vector<float> previous_pose)
{
    // Частота
    static const double MIN_DT = 1.0 / 50.0;
    
    // Корректируем значение времени
    double raw_dt = diff_time.toSec();
    double dt = std::max(raw_dt, MIN_DT);
    
    /***************************************************/
    /****ПОСЧИТАЙТЕ СКОРОСТЬ ЛЕВОГО И ПРАВОГО КОЛЕСА****/
    /***************************************************/

    // Линейная и угловая скорости робота
    float v = control_velocity.linear.x;
    float w = control_velocity.angular.z;

    // Скорости колес (угловые скорости, делённые на радиус)
    float vel_l = (v - (w * WHEEL_DISTANCE / 2.0)) / WHEEL_RADIUS;
    float vel_r = (v + (w * WHEEL_DISTANCE / 2.0)) / WHEEL_RADIUS;

    vector<float> new_pose;

    new_pose = forwardKinematics(previous_pose[0], previous_pose[1], previous_pose[2], dt, vel_l, vel_r);

    /*************************************/
    /****ЗАПОЛНИТЕ СООБЩЕНИЕ ОДОМЕТРИИ****/
    /*************************************/

    // Показываем что одометрия отправляется между odom - глобальной системой координат, и base_link - локальной
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // Обновление позиции
    odom.pose.pose.position.x = new_pose[0];
    odom.pose.pose.position.y = new_pose[1];
    odom.pose.pose.position.z = 0.0;

    // Обновление ориентации
    tf::Quaternion q;
    q.setRPY(0, 0, new_pose[2]);  // Угол theta из new_pose
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // Скорости в одометрии
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;

    // Публикация обновленной одометрии
    pubOdom.publish(odom);

    // Обновляем положение - теперь наше текущее становится прошлым
    prev_pose = new_pose;
}