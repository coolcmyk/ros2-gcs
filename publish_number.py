#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def publish_angka():
    # Inisialisasi node ROS
    rospy.init_node('publisher_angka', anonymous=True)
    
    # Buat publisher untuk topic '/angka'
    pub = rospy.Publisher('/angka', Int32, queue_size=10)

    while not rospy.is_shutdown():
        try:
            # Meminta input angka dari pengguna
            angka = int(input("Masukkan angka untuk dipublish (Ctrl+C untuk berhenti): "))
            
            # Publish angka
            rospy.loginfo(f'Publish angka: {angka}')
            pub.publish(angka)
            
        except ValueError:
            print("Input harus berupa angka bulat!")

if __name__ == '__main__':
    try:
        publish_angka()
    except rospy.ROSInterruptException:
        pass
