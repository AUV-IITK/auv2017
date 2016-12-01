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
        window = Tk()
        side_stab_var = IntVar()
        vert_stab_var = IntVar()
        forward_motion = rospy.Publisher('/pwm/forward', Int32)
        Turn_motion = rospy.Publisher('/pwm/turn', Int32)
        Sideward_motion = rospy.Publisher('/pwm/sideward', Int32)
        Upward_motion = rospy.Publisher('/pwm/upward', Int32)
        rospy.init_node('remote_gui', anonymous=True)

        def change_pwm_motion_1(data):
            w01.set(data.data)

        def change_pwm_motion_2(data):
            w02.set(data.data)

        def change_pwm_motion_3(data):
            w03.set(data.data)

        def change_pwm_motion_4(data):
            w04.set(data.data)
        # topic for motion_1
        rospy.Subscriber("/pwm/forward", Int32, change_pwm_motion_1)
        # topic for motion_2
        rospy.Subscriber("/pwm/turn", Int32, change_pwm_motion_2)
        # topic for motion_3
        rospy.Subscriber("/pwm/sideward", Int32, change_pwm_motion_3)
        # topic for motion_4
        rospy.Subscriber("/pwm/upward", Int32, change_pwm_motion_4)
# #################################################################
        ca = Canvas(window, height=50)
        ca.pack()
        main = PanedWindow()
        main.pack()
        m01 = PanedWindow(main, orient=VERTICAL)
        main.add(m01)
        w01 = Scale(m01, length=150, troughcolor="orange", highlightbackground="grey", label="PWM", fg="darkviolet",
                    from_=-255, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        l01 = Label(m01, fg="green", font=("Helvetica", 12), text="front_current")
        m01.add(l01)
        m01.add(w01)
        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m04 = PanedWindow(main, orient=VERTICAL)
        main.add(m04)
        w04 = Scale(m04, length=150, troughcolor="orange", highlightbackground="grey", label="PWM", fg="darkviolet",
                    from_=-255, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        l04 = Label(m04, fg="green", font=("Helvetica", 12), text="vertical_current")
        m04.add(l04)
        m04.add(w04)
        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m03 = PanedWindow(main, orient=VERTICAL)
        main.add(m03)
        w03 = Scale(m03, length=150, troughcolor="orange", highlightbackground="grey", label="PWM", fg="darkviolet",
                    from_=-255, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        l03 = Label(m03, fg="green", font=("Helvetica", 12), text="sway_current")
        m03.add(l03)
        m03.add(w03)
        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m02 = PanedWindow(main, orient=VERTICAL)
        main.add(m02)
        w02 = Scale(m02, length=150, troughcolor="orange", highlightbackground="grey", label="PWM", fg="darkviolet",
                    from_=-255, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        l02 = Label(m02, fg="green", font=("Helvetica", 12), text="turn_current")
        m02.add(l02)
        m02.add(w02)

# #################################################################
# FORWARD_RELATED########################################################

        def forwardClicked(event):
            """Documentation for a function"""
            w1.set(w1.get() + 10)
            rospy.loginfo("forward is clicked with pwm = %d", w1.get())
            forward_motion.publish(w1.get())

        def backwardClicked(event):
            w1.set(w1.get() - 10)
            rospy.loginfo("backward is clicked with pwm = %d", w1.get())
            forward_motion.publish(w1.get())

        def stop_front(event):
            rospy.loginfo("Stop is clicked")
            forward_motion.publish(0)
            w1.set(0)
# #######################################################################
# SIDEWARD_RELATED#######################################################

        def stop_sway(event):
            if side_stab_var.get() == 0:
                rospy.loginfo("Stop is clicked")
                Sideward_motion.publish(0)
                w3.set(0)
            else:
                rospy.loginfo("side stability is on")

        def leftClicked(event):
            if side_stab_var.get() == 0:
                w3.set(w3.get() - 10)
                rospy.loginfo("sway left is clicked with pwm = %d", w3.get())
                Sideward_motion.publish(w3.get())
            else:
                rospy.loginfo("side stability is on")

        def rightClicked(event):
            if side_stab_var.get() == 0:
                w3.set(w3.get() + 10)
                rospy.loginfo("Sway right is clicked with pwm = %d", w3.get())
                Sideward_motion.publish(w3.get())
            else:
                rospy.loginfo("side stability is on")
# #######################################################################
# VERTICAL_RELATED#######################################################

        def stop_vertical(event):
            if vert_stab_var.get() == 0:
                rospy.loginfo("Stop is clicked")
                Upward_motion.publish(0)
                w2.set(0)
            else:
                rospy.loginfo("vertical stability is on")

        def upClicked(event):
            if vert_stab_var.get() == 0:
                w2.set(w2.get() + 10)
                rospy.loginfo("up is clicked with pwm = %d", w2.get())
                Upward_motion.publish(w2.get())
            else:
                rospy.loginfo("vertical stability is on")

        def downClicked(event):
            if vert_stab_var.get() == 0:
                w2.set(w2.get() - 10)
                rospy.loginfo("dowm is clicked with pwm = %d", w2.get())
                Upward_motion.publish(w2.get())
            else:
                rospy.loginfo("vertical stability is on")
# #######################################################################
# TURN_RELATED###########################################################

        def stop_turn(event):
            if side_stab_var.get() == 0:
                rospy.loginfo("Stop is clicked")
                Turn_motion.publish(0)
                w4.set(0)
            else:
                rospy.loginfo("side stability is on")

        def clock(event):
            if side_stab_var.get() == 0:
                w4.set(w4.get() + 10)
                rospy.loginfo("clockwise is clicked with pwm = %d", w4.get())
                Turn_motion.publish(w4.get())
            else:
                rospy.loginfo("side stability is on")

        def anticlock(event):
            if side_stab_var.get() == 0:
                w4.set(w4.get() - 10)
                rospy.loginfo("anticlockwise is clicked with pwm = %d", w4.get())
                Turn_motion.publish(w4.get())
            else:
                rospy.loginfo("side stability is on")
# #######################################################################
# TOTAL_RESET############################################################

        def reset(event):
            rospy.loginfo("reset is called, everything is set to 0")
            w1.set(0)
            w2.set(0)
            w3.set(0)
            w4.set(0)
            Turn_motion.publish(0)
            Sideward_motion.publish(0)
            Upward_motion.publish(0)
            forward_motion.publish(0)
# #######################################################################

        def verticalStabilize():
            if vert_stab_var.get() == 1:
                print("now on vert")
            else:
                print("now off vert")

        def sidewardStabilize():
            if side_stab_var.get() == 1:
                print("now on side")
            else:
                print("now off side")

# LABEL TOP##################################################################
        label = Label(window, bd=5, font=("Helvetica", 16),
                      fg="blue", anchor=N, text="AUV IITK")
        label.pack()
# ###########################################################################
# TOTAL RESET################################################################
        Button(window, bg="darkmagenta", font=("Helvetica", 14), fg="white",
               height=3, width=10, text="Reset", command=reset).pack()
        window.bind('r', reset)
# ###########################################################################
        t1 = Checkbutton(window, text="Vert. Stab.", command=verticalStabilize, variable=vert_stab_var, onvalue=1, offvalue=0, height=5, width=20)
        t2 = Checkbutton(window, text="Side Stab.", command=sidewardStabilize, variable=side_stab_var, onvalue=1, offvalue=0, height=5, width=20)
        t1.pack()
        t2.pack()
# ###########################################################################
        ca = Canvas(window, height=50)
        ca.pack()
        main = PanedWindow()
        main.pack()
        m1 = PanedWindow(main, orient=VERTICAL)
        main.add(m1)
        w1 = Scale(m1, length=150, troughcolor="orange", highlightbackground="grey", label="PWM",
                   fg="darkviolet", from_=-255, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        window.bind('w', forwardClicked)
        window.bind('s', backwardClicked)
        l1 = Label(m1, fg="green", font=("Helvetica", 12), text="front")
        Stop_front = Button(m1, text="stop", fg="yellow", bg="red", command=stop_front)
        window.bind('u', stop_front)
        m1.add(l1)
        m1.add(w1)
        m1.add(Stop_front)
# ###########################################################################
        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m2 = PanedWindow(main, orient=VERTICAL)
        main.add(m2)
        w2 = Scale(m2, length=150, troughcolor="orange", highlightbackground="grey", label="PWM",
                   fg="darkviolet", from_=-255, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        window.bind('z', upClicked)
        window.bind('x', downClicked)
        l2 = Label(m2, fg="green", font=("Helvetica", 12), text="vertical")
        Stop_vertical = Button(m2, fg="yellow", bg="red",
                               text="Stop", command=stop_vertical)
        window.bind('i', stop_front)
        m2.add(l2)
        m2.add(w2)
        m2.add(Stop_vertical)
# #######################################################################
        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m3 = PanedWindow(main, orient=VERTICAL)
        main.add(m3)
        w3 = Scale(m3, length=150, troughcolor="orange", highlightbackground="grey", label="PWM",
                   fg="darkviolet", from_=-255, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        l3 = Label(m3, fg="green", font=("Helvetica", 12), text="sway")
        window.bind('a', leftClicked)
        window.bind('d', rightClicked)
        left = Button(m3, text="Sway left", height=2, command=leftClicked)
        right = Button(m3, text="Sway right", height=2, command=rightClicked)
        Stop_sway = Button(m3, fg="yellow", bg="red",
                           text="stop", command=stop_sway)
        window.bind('o', stop_sway)
        m3.add(l3)
        m3.add(w3)
        m3.add(Stop_sway)
# #######################################################################
        ca = Canvas(main, width=50, height=100)
        main.add(ca)
        m4 = PanedWindow(main, orient=VERTICAL)
        main.add(m4)
        w4 = Scale(m4, length=150, troughcolor="orange", highlightbackground="grey", label="PWM",
                   fg="darkviolet", from_=-255, to=255, orient=HORIZONTAL, activebackground="lightgreen")
        l4 = Label(m4, fg="green", font=("Helvetica", 12), text="turn")
        window.bind('c', anticlock)
        window.bind('v', clock)
        Stop_turn = Button(m4, bg="red", fg="yellow",
                           text="stop", command=stop_turn)
        window.bind('p', stop_turn)
        m4.add(l4)
        m4.add(w4)
        m4.add(Stop_turn)

        window.mainloop()

    except rospy.ROSInterruptException:
        pass
