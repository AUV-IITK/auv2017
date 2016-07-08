#!/usr/bin/python
from Tkinter import *
import rospy
"""
\file
\brief super short description

Long decription
"""
from std_msgs.msg import Int32

if __name__ == '__main__':
    try:
        forward_motion = rospy.Publisher('/pwm/forward', Int32)
        Turn_motion_sway = rospy.Publisher('/pwm/turnsway', Int32)
        Turn_motion_front = rospy.Publisher('/pwm/turn', Int32)
        Sideward_motion = rospy.Publisher('/pwm/sideward', Int32)
        Upward_motion = rospy.Publisher('/pwm/upward', Int32)
        rospy.init_node('remote_gui', anonymous=True)

        def forwardClicked():
            """Documentation for a function"""
            rospy.loginfo("forward is clicked with pwm = %d", w1.get())
            forward_motion.publish(w1.get())

        def backwardClicked():
            rospy.loginfo("backward is clicked with pwm = %d", -w1.get())
            forward_motion.publish(-w1.get())

        def stop_front():
            rospy.loginfo("Stop is clicked")
            forward_motion.publish(0)
            w1.set(0)

        def stop_vertical():
            rospy.loginfo("Stop is clicked")
            Upward_motion.publish(0)
            w2.set(0)

        def stop_sway():
            rospy.loginfo("Stop is clicked")
            Sideward_motion.publish(0)
            w3.set(0)

        def stop_turn():
            rospy.loginfo("Stop is clicked")
            Turn_motion_sway.publish(0)
            Turn_motion_front.publish(0)
            w4.set(0)

        def upClicked():
            rospy.loginfo("up is clicked with pwm = %d", w2.get())
            Upward_motion.publish(w2.get())

        def downClicked():
            rospy.loginfo("dowm is clicked with pwm = %d", -w2.get())
            Upward_motion.publish(-w2.get())

        def leftClicked():
            rospy.loginfo("sway left is clicked with pwm = %d", -w3.get())
            Sideward_motion.publish(-w3.get())

        def rightClicked():
            rospy.loginfo("Sway right is clicked with pwm = %d", w3.get())
            Sideward_motion.publish(w3.get())

        def clockSway():
            rospy.loginfo(
                "clockwise (sway) is clicked with pwm = %d", w4.get())
            Turn_motion_sway.publish(w4.get())

        def anticlockSway():
            rospy.loginfo(
                "anticlockwise (sway) is clicked with pwm = %d", -w4.get())
            Turn_motion_sway.publish(-w4.get())

        def clockFront():
            rospy.loginfo(
                "clockwise (front) is clicked with pwm = %d", w4.get())
            Turn_motion_front.publish(w4.get())

        def anticlockFront():
            rospy.loginfo(
                "anticlockwise (front) is clicked with pwm = %d", -w4.get())
            Turn_motion_front.publish(-w4.get())

        def reset():
            rospy.loginfo("reset is called, everything is set to 0")
            w1.set(0)
            w2.set(0)
            w3.set(0)
            w4.set(0)
            Turn_motion_sway.publish(0)
            Turn_motion_front.publish(0)
            Sideward_motion.publish(0)
            Upward_motion.publish(0)
            forward_motion.publish(0)

        window = Tk()
        label = Label(window, bd=5, font=("Helvetica", 16),
                      fg="blue", anchor=N, text="AUV IITK")
        label.pack()
        Button(window, bg="darkmagenta", font=("Helvetica", 14), fg="white",
               height=3, width=10, text="Reset", command=reset).pack()
        ca = Canvas(window, height=50)
        ca.pack()
        main = PanedWindow()
        main.pack()
        m1 = PanedWindow(main, orient=VERTICAL)
        main.add(m1)
        w1 = Scale(m1, length=150, troughcolor="orange", highlightbackground="grey", label="PWM",
                   fg="darkviolet", from_=0, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        forward = Button(m1, text="forward", height=2, command=forwardClicked)
        l1 = Label(m1, fg="green", font=("Helvetica", 12), text="front")
        backward = Button(m1, text="backward", height=2,
                          command=backwardClicked)
        Stop_front = Button(m1, text="stop", fg="yellow",
                            bg="red", command=stop_front)
        m1.add(l1)
        m1.add(w1)
        m1.add(forward)
        m1.add(backward)
        m1.add(Stop_front)
        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m2 = PanedWindow(main, orient=VERTICAL)
        main.add(m2)
        w2 = Scale(m2, length=150, troughcolor="orange", highlightbackground="grey", label="PWM",
                   fg="darkviolet", from_=0, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        l2 = Label(m2, fg="green", font=("Helvetica", 12), text="vertical")
        up = Button(m2, text="up", height=2, command=upClicked)
        down = Button(m2, text="down", height=2, command=downClicked)
        Stop_vertical = Button(m2, fg="yellow", bg="red",
                               text="Stop", command=stop_vertical)
        m2.add(l2)
        m2.add(w2)
        m2.add(up)
        m2.add(down)
        m2.add(Stop_vertical)
        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m3 = PanedWindow(main, orient=VERTICAL)
        main.add(m3)
        w3 = Scale(m3, length=150, troughcolor="orange", highlightbackground="grey", label="PWM",
                   fg="darkviolet", from_=0, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        l3 = Label(m3, fg="green", font=("Helvetica", 12), text="sway")
        left = Button(m3, text="Sway left", height=2, command=leftClicked)
        right = Button(m3, text="Sway right", height=2, command=rightClicked)
        Stop_sway = Button(m3, fg="yellow", bg="red",
                           text="stop", command=stop_sway)
        m3.add(l3)
        m3.add(w3)
        m3.add(left)
        m3.add(right)
        m3.add(Stop_sway)
        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m4 = PanedWindow(main, orient=VERTICAL)
        main.add(m4)
        w4 = Scale(m4, length=150, troughcolor="orange", highlightbackground="grey", label="PWM",
                   fg="darkviolet", from_=0, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        l4 = Label(m4, fg="green", font=("Helvetica", 12), text="turn")
        clockwiseSway = Button(m4, text="clockwise (sway)", command=clockSway)
        anticlockwiseSway = Button(
            m4, text="anticlockwise (sway)", command=anticlockSway)
        clockwiseFront = Button(
            m4, text="clockwise (front)", command=clockFront)
        anticlockwiseFront = Button(
            m4, text="anticlockwise (front)", command=anticlockFront)
        Stop_turn = Button(m4, bg="red", fg="yellow",
                           text="stop", command=stop_turn)
        m4.add(l4)
        m4.add(w4)
        m4.add(clockwiseSway)
        m4.add(anticlockwiseSway)
        m4.add(clockwiseFront)
        m4.add(anticlockwiseFront)
        m4.add(Stop_turn)
        window.mainloop()

    except rospy.ROSInterruptException:
        pass
