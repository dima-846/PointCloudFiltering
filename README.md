# PointCloudFiltering
Дано облако точек с лидара. Требуется отфильтровать землю и из 
оставшихся точек сформировать BEV-изображение (Bird-Eye-View). Для 
формирования BEV использовать диапазон точек x∈[−100 ;100], y∈[−100 ;100] , 
шаг сетки - 0.5 м. На выходе должно получиться одноканальное изображение 
размером 500x500, в котором интенсивность пикселя пропорциональна 
количеству точек, попадающих в него. Облако точек в формате .pcd лежит в файлах. 
