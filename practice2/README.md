# Приращение ковариации и расширенный фильтр Калмана для мобильного робота

## Файлы в директории

1. **Task1.ipynb**  
   - Реализуована функция `differential_drive_motion` для вычисления нового положения робота с дифференциальным приводом.
   - Реализована функция `propagate_covariance` для приращения ковариации этого движения.
   - Визуализировано изменение положения робота и матрицы ковариации на промежутке от 0 секунд до 100 с шагом одна секунда. 

2. **Task2.ipynb**  
   - Реализован расширенный фильтр Калмана для мобильного робота. Робот начинает движение в стартовой точке (0, 0) и двигается по окружности постоянного радиуса с постоянной скоростью до промежуточной точки, а затем продолжает движение по другой окружности, пока не достигнет финиша.
