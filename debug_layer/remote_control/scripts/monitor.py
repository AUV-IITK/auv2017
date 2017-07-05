#!/usr/bin/python
from Tkinter import *
import rospy
from std_msgs.msg import Int32

if __name__ == '__main__':
    try:

        def change_pwm_motion_1(data):
            w1.set(data.data)

        def change_pwm_motion_2(data):
            w2.set(data.data)

        def change_pwm_motion_3(data):
            w3.set(data.data)

        def change_pwm_motion_4(data):
            w4.set(data.data)

        def change_pressure(data):
            w5.set(data.data)

        def change_yaw(data):
            w6.set(data.data)

        rospy.init_node('pwm_monitor', anonymous=True)
        pressure = rospy.Subscriber(
            "/varun/sensors/pressure_sensor/depth", Int32, change_pressure)
        yaw = rospy.Subscriber("/varun/sensors/imu/yaw", Int32, change_yaw)
        # topic for motion_1
        rospy.Subscriber("/pwm/forward", Int32, change_pwm_motion_1)
        # topic for motion_2
        rospy.Subscriber("/pwm/turn", Int32, change_pwm_motion_2)
        rospy.Subscriber("/pwm/sideward", Int32,
                         change_pwm_motion_3)  # topic for motion_3
        # topic for motion_4
        rospy.Subscriber("/pwm/upward", Int32, change_pwm_motion_4)

        window = Tk()
        label = Label(window, bd=5, font=("Helvetica", 16),
                      fg="blue", anchor=N, text="AUV IITK")
        label.pack()
        ca = Canvas(window, height=50)
        ca.pack()
        main = PanedWindow()
        main.pack()
        m1 = PanedWindow(main, orient=VERTICAL)
        main.add(m1)
        w1 = Scale(m1, length=150, troughcolor="orange", highlightbackground="grey", label="PWM",
                   fg="darkviolet", from_=-255, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        l1 = Label(m1, fg="green", font=("Helvetica", 12), text="front")
        m1.add(l1)
        m1.add(w1)

        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m2 = PanedWindow(main, orient=VERTICAL)
        main.add(m2)
        w2 = Scale(m2, length=150, troughcolor="orange", highlightbackground="grey", label="PWM",
                   fg="darkviolet", from_=-255, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        l2 = Label(m2, fg="green", font=("Helvetica", 12), text="turn")
        m2.add(l2)
        m2.add(w2)

        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m3 = PanedWindow(main, orient=VERTICAL)
        main.add(m3)
        w3 = Scale(m3, length=150, troughcolor="orange", highlightbackground="grey", label="PWM",
                   fg="darkviolet", from_=-255, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        l3 = Label(m3, fg="green", font=("Helvetica", 12), text="sideward")
        m3.add(l3)
        m3.add(w3)

        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m4 = PanedWindow(main, orient=VERTICAL)
        main.add(m4)
        w4 = Scale(m4, length=150, troughcolor="orange", highlightbackground="grey", label="PWM",
                   fg="darkviolet", from_=-255, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        l4 = Label(m4, fg="green", font=("Helvetica", 12), text="upward")
        m4.add(l4)
        m4.add(w4)

        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m5 = PanedWindow(main, orient=VERTICAL)
        main.add(m5)
        w5 = Scale(m5, length=150, troughcolor="orange", highlightbackground="grey", label="Depth",
                   fg="darkviolet", from_=-500, to=100, orient=HORIZONTAL, activebackground="lightgreen")
        l5 = Label(m5, fg="green", font=(
            "Helvetica", 12), text="pressure sensor")
        m5.add(l5)
        m5.add(w5)

        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m6 = PanedWindow(main, orient=VERTICAL)
        main.add(m6)
        w6 = Scale(m6, length=150, troughcolor="orange", highlightbackground="grey", label="YAW",
                   fg="darkviolet", from_=-180, to=180, orient=HORIZONTAL, activebackground="lightgreen")
        l6 = Label(m6, fg="green", font=("Helvetica", 12), text="yaw")
        m6.add(l6)
        m6.add(w6)

        window.mainloop()

    except rospy.ROSInterruptException:
        pass
