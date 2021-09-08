#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
from datetime import datetime

class TaskLogClass(object):
    def log_file_func(self, rokos_type, start_time, finish_time, diff_time, task_list):
        current_time = self.datenow_func(finish_time)
        file_name = str(current_time + "_" + str(rokos_type))

        write_data = str()
        write_data += "\n%s Log File" % (rokos_type)
        write_data += "\nStart Time = %s\nFinish Time = %s\n" % (str(start_time), str(finish_time))
        write_data += "\nTime Difference = %s\n"  % (str(diff_time))
        write_data += "\n------------------------------------------------\n\n"

        write_data += self.convert_task_to_string(task_list)

        self.write_file_func(file_name, write_data)

    @classmethod
    def convert_task_to_string(cls, task_list):
        str_task_list = str(task_list).replace(", [[", ",\n [[")

        return str_task_list

    @classmethod
    def write_file_func(cls, file_name, write_data):
        import os
        try:
            get_file_path = os.path.realpath(os.path.dirname(__file__))
            full_path = os.path.join(get_file_path, "../task_log_file")
            with open(str(full_path) + "/" + str(file_name) + '.txt', 'w+') as write_file:
                write_file.write(write_data)

            write_file.close()

        except Exception as err:
            print(err)

    @classmethod
    def datenow_func(cls, now_time):
        dt_string = now_time.strftime("%Y_%m_%d_-_%H_%M_%S")

        return str(dt_string)

