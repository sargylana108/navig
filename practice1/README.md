# Одометрия мобильного робота

## Цель работы
- Реализовать расчет одометрии мобильного робота с дифференциальным приводом.
- Обеспечить прохождение роботом заданной траектории.

## Что было сделано
1. Функция `forwardKinematics`:
2. Функция `updateOdometry`:

## Полученный результат
Робот успешно проходит заданную траекторию, видна корректная работа одометрии.

![Trajectory](assets/trajectory_rviz.png)

## Запуск проекта

Для запуска данного проекта требуется установленный ROS Noetic и пакет rviz для визуализации траектории робота.

1. Проект должен  расположен в папке `catkin_ws/src/practice1`.

2. Перейдите в рабочую папку ROS, склонируйте репозиторий и соберите пакет:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/sargylana108/navig.git
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

2. Запустите симуляцию:
    ```bash
    roslaunch practice1 robot_kinematics.launch
    ```
