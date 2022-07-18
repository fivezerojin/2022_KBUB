#!/usr/bin/env python
import sys, os
import rospy

def main():
    user_id = os.getlogin()
    paths_dir = "/home/"+user_id+"/catkin_ws/src/kbub_pkg/src/map/"
    files_in_dir = os.listdir(paths_dir)

    txts = [file for file in files_in_dir if file.endswith('.txt')]

    print("------PLEASE DON'T EXIT THE PROGRAM WHILE PROCESSING------")

    for txt in txts:
        with open(paths_dir + txt, 'r') as r:
            temp_lines = list(enumerate(r.readlines()))
            lines_cnt = len(temp_lines)+1
        with open(paths_dir + txt, 'w') as w:
            for coords_idx, coords in temp_lines:
                if coords_idx % 4 == 0:
                    w.write(coords)
    
    print("------DONE------")




if __name__ == "__main__":
    main()