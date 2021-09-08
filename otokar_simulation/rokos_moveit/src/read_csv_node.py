#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import csv


def csv_read():
    try:
        right_rokos = "---\n\nRight_Rokos:\n\n"
        left_rokos = "---\n\nLeft_Rokos:\n\n"
        count = 0
        with open('/home/ros/Desktop/VALU3S/Otokar/Yorungeler/13M38_Yorunge.csv', 'r') as csvfile:
            read_file = csv.reader(csvfile, delimiter=',')

            for row in read_file:
                count += 1
                robot_no = row[2]
                if robot_no == "Robot_NO":
                    print("DONE")
                    continue

                else:
                    x_deger = float(row[19])

                    if x_deger < -7.8:
                        x_deger = -7.8

                    y_deger = float(row[20])
                    z_deger = float(row[21])

                    if int(robot_no) == 2:
                        c_ax_deger = float(row[9])

                    else:
                        c_ax_deger = float(float(row[9]) * -1)

                    c_az_deger = float(float(row[11]) * -1)
                    x_sira = int(row[8])
                    y_sira = int(row[4])
                    z_sira = int(row[6])
                    c_ax_sira = int(row[10])
                    c_az_sira = int(row[12])
                    task_id = int(str(row[0]).strip())
                    arac_kodu = str(row[1]).strip()
                    durum = int(row[13])
                    etiket = str(row[15]).strip()

                    schema = "  - Task:\n      Task_ID: %d\n\n      Vehicle_CODE: '%s'\n\n      Mode: %d\n\n      Tag: '%s'\n\n      Position:\n        X: %.3f\n        Y: %.3f\n        Z: %.3f\n        C_AX: %.3f\n        C_AZ: %.3f\n\n      Queue:\n        X: %d\n        Y: %d\n        Z: %d\n        C_AX: %d\n        C_AZ: %d\n\n" % (task_id, arac_kodu, durum, etiket, x_deger, y_deger, z_deger, c_ax_deger, c_az_deger, x_sira, y_sira, z_sira, c_ax_sira, c_az_sira)

                    if int(robot_no) == 1:
                        right_rokos += str(schema)

                    else:
                        left_rokos += str(schema)

        return right_rokos, left_rokos

    except Exception as err:
        print(err)

def write_file_func(file_name, write_data):
    try:
        with open('/home/ros/Desktop/VALU3S/Otokar/Yorungeler/' + str(file_name) + '_task.yaml', 'w+') as write_file:
            write_file.write(write_data)

        write_file.close()

    except Exception as err:
        print(err)

def read_func():
    right_rokos, left_rokos = csv_read()

    write_file_func('right_rokos', right_rokos)
    write_file_func('left_rokos', left_rokos)


if __name__ == '__main__':
    rospy.init_node('read_csv_node')
    read_func()
    