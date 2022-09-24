#!/usr/bin/env python3
'''
    Cкрипт принимает x1, x2, y_upped, y_bottom, x_start, y_start, x_end, y_end
    предполагается, что линия движения лежит вдоль парковок
    предполагается, что парковочные места друг с другом соеденены

    Скрипт будет возвращать текстовый файл parking_points.txt с кординатами 
    необходимыми для парковок со структурой:

    start_x
    start_y
    end_y
    end_x
    center_x
    centery

'''

class parking_place:
    def __init__(self,id):
        self.id = id 


if __name__ == "__main__":
    x1 = 388956.89
    x2 = 388961.89
    y_upper = 221204.03
    y_bottom = 221198.03
    num_of_palces = 18
    diff = x2 - x1

    x_start = 388949.36
    y_start = 221207.86

    x_end = 388968.36
    y_end = 221207.86
 

    #------------------------PLACES - список парковок(кординат их 4х точек)------------
    places = []
    for num in range(num_of_palces):
        place = parking_place(num + 1)
        place.x1 = place.x4 = x1 + diff * num
        place.x2 = place.x3 = x1 + diff * (num + 1)
        place.y1 = place.y2 = y_upper
        place.y3 = place.y4 = y_bottom

        place.center_x = place.x1 + (place.x2 - place.x1) / 2
        place.center_y = place.y4 + (place.y1 - place.y4) / 2

        place.x_start = x_start + num * diff
        place.y_start = y_start
        place.x_end = x_end + num * diff
        place.y_end = y_end

        places.append(place)

    with open("parking_points.txt", 'w') as file:
        for i in range(len(places)):
            place = places[i]
            #file.write(f"{place.x_start}\n")
            file.write(f"{388932.98}\n") #Сделали так что у каждой комбинации 3х точек первая точка левее автомобиля
            file.write(f"{place.y_start}\n")
            file.write(f"{place.x_end}\n")
            file.write(f"{place.y_end}\n")
            file.write(f"{place.center_x}\n")
            file.write(f"{place.center_y}\n")
            