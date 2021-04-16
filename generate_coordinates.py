nums = [1,2,3,4,5,6,7,8,9,10,11,12,12,11,10,9,8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8,9,10,11,12,12,11,10,9,8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8,9,10,11,12,12,11,10,9,8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8,9,10,11,12,12,11,10,9,8,7,6,5,4,3,2,1]
letters = ["A", "B", "C","D","E","F","G","H"]
x_0 = 4200
y_0 = 12000
dx_well = 7300
dy_well = 7300
dx_plate = 83500
dy_plate = 56000

with open("platecoordinates.dat", "w") as f:
    for i in range(12):
        plate_x = x_0 + (i % 8) * dx_plate
        plate_y = y_0 + (i / 8) * dy_plate
        for j in range(96):
            well_y = plate_y + (j / 12) * dy_well
            well_x = plate_x + (nums[j] - 1) * dx_well

            well = letters[j / 12]

            out = str(i + 1) + well + str(nums[j]) + ","
            out += str(well_x) + "," + str(well_y) + "\n"

            f.write(out)

