#!/usr/bin/python
from Tkinter import *
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import motion_commons.msg
import actionlib
import roslib
roslib.load_manifest('motion_commons')
"""
\file
\brief super short description

Long decription
"""

if __name__ == '__main__':
    try:
        window = Tk()
        window.wm_title("AUV Remote Controller")
        side_stab_var = IntVar()
        vert_stab_var = IntVar()
        forward_motion = rospy.Publisher(
            '/pwm/forward', Int32, queue_size=1000)
        Turn_motion = rospy.Publisher('/pwm/turn', Int32, queue_size=1000)
        Sideward_motion = rospy.Publisher(
            '/pwm/sideward', Int32, queue_size=1000)
        Upward_motion = rospy.Publisher('/pwm/upward', Int32, queue_size=1000)
        present_depth_data = rospy.Publisher(
            '/varun/motion/z_distance', Float64, queue_size=1000)
        present_angle_data = rospy.Publisher(
            '/varun/motion/yaw', Float64, queue_size=1000)
        rospy.init_node('remote_gui', anonymous=True)

        def pressure_sensor_data(data):
            global present_depth
            present_depth = data.data
            present_depth_data.publish(data.data)

        def imu_sensor_data(data):
            global present_angle
            present_angle = data.data
            present_angle_data.publish(data.data)

        # topic for pressure sensor data
        rospy.Subscriber("/varun/sensors/pressure_sensor/depth",
                         Float64, pressure_sensor_data)
        rospy.Subscriber("/varun/sensors/imu/yaw",
                         Float64, imu_sensor_data)
# #################################################################
# FORWARD_RELATED########################################################

        def forwardClicked(event):
            """Documentation for a function"""
            w1.set(w1.get() + 10)
            rospy.loginfo("forward is clicked with pwm = %d", w1.get())
            forward_motion.publish(w1.get())
            forward_motion.publish(w1.get())
            forward_motion.publish(w1.get())

        def backwardClicked(event):
            w1.set(w1.get() - 10)
            rospy.loginfo("backward is clicked with pwm = %d", w1.get())
            forward_motion.publish(w1.get())
            forward_motion.publish(w1.get())
            forward_motion.publish(w1.get())

        def stop_front(event):
            rospy.loginfo("Stop is clicked")
            forward_motion.publish(0)
            forward_motion.publish(0)
            forward_motion.publish(0)
            w1.set(0)
# #######################################################################
# SIDEWARD_RELATED#######################################################

        def stop_sway(event):
            rospy.loginfo("Stop is clicked")
            Sideward_motion.publish(0)
            Sideward_motion.publish(0)
            Sideward_motion.publish(0)
            w3.set(0)

        def leftClicked(event):
            w3.set(w3.get() - 30)
            rospy.loginfo("sway left is clicked with pwm = %d", w3.get())
            Sideward_motion.publish(w3.get())
            Sideward_motion.publish(w3.get())
            Sideward_motion.publish(w3.get())

        def rightClicked(event):
            w3.set(w3.get() + 30)
            rospy.loginfo("Sway right is clicked with pwm = %d", w3.get())
            Sideward_motion.publish(w3.get())
            Sideward_motion.publish(w3.get())
            Sideward_motion.publish(w3.get())
# #######################################################################
# VERTICAL_RELATED#######################################################

        def stop_vertical(event):
            if vert_stab_var.get() == 0:
                rospy.loginfo("Stop is clicked")
                Upward_motion.publish(0)
                Upward_motion.publish(0)
                Upward_motion.publish(0)
                w2.set(0)
            else:
                rospy.loginfo("vertical stability is on")

        def upClicked(event):
            if vert_stab_var.get() == 0:
                w2.set(w2.get() + 5)
                rospy.loginfo("up is clicked with pwm = %d", w2.get())
                Upward_motion.publish(w2.get())
                Upward_motion.publish(w2.get())
                Upward_motion.publish(w2.get())
            else:
                rospy.loginfo("vertical stability is on")

        def downClicked(event):
            if vert_stab_var.get() == 0:
                w2.set(w2.get() - 5)
                rospy.loginfo("dowm is clicked with pwm = %d", w2.get())
                Upward_motion.publish(w2.get())
                Upward_motion.publish(w2.get())
                Upward_motion.publish(w2.get())
            else:
                rospy.loginfo("vertical stability is on")
# #######################################################################
# TURN_RELATED###########################################################

        def stop_turn(event):
            if side_stab_var.get() == 0:
                rospy.loginfo("Stop is clicked")
                Turn_motion.publish(0)
                Turn_motion.publish(0)
                Turn_motion.publish(0)
                w4.set(0)
            else:
                rospy.loginfo("side stability is on")

        def clock(event):
            if side_stab_var.get() == 0:
                w4.set(w4.get() + 5)
                rospy.loginfo("clockwise is clicked with pwm = %d", w4.get())
                Turn_motion.publish(w4.get())
                Turn_motion.publish(w4.get())
                Turn_motion.publish(w4.get())
            else:
                rospy.loginfo("side stability is on")

        def anticlock(event):
            if side_stab_var.get() == 0:
                w4.set(w4.get() - 5)
                rospy.loginfo(
                    "anticlockwise is clicked with pwm = %d", w4.get())
                Turn_motion.publish(w4.get())
                Turn_motion.publish(w4.get())
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
            vert_stab_var.set(0)
            verticalStabilize()
            side_stab_var.set(0)
            turnStabilize()
            Turn_motion.publish(0)
            Turn_motion.publish(0)
            Turn_motion.publish(0)
            Sideward_motion.publish(0)
            Sideward_motion.publish(0)
            Sideward_motion.publish(0)
            Upward_motion.publish(0)
            Upward_motion.publish(0)
            Upward_motion.publish(0)
            forward_motion.publish(0)
            forward_motion.publish(0)
            forward_motion.publish(0)
# #######################################################################

        entry_vertical = Entry(window, width=10)

        def verticalDataStabilize():
            client_vertical = actionlib.SimpleActionClient(
                'upward', motion_commons.msg.UpwardAction)
            client_vertical.wait_for_server()
            goal_upward = motion_commons.msg.UpwardGoal(
                Goal=float(entry_vertical.get()), loop=100000)
            if vert_stab_var.get() == 1:
                client_vertical.send_goal(goal_upward)
            else:
                goal_upward = motion_commons.msg.UpwardGoal(
                    Goal=present_depth, loop=0)
                client_vertical.send_goal(goal_upward)

        def verticalStabilize():
            client_vertical = actionlib.SimpleActionClient(
                'upward', motion_commons.msg.UpwardAction)
            client_vertical.wait_for_server()
            goal_upward = motion_commons.msg.UpwardGoal(
                Goal=present_depth, loop=100000)
            if vert_stab_var.get() == 1:
                client_vertical.send_goal(goal_upward)
            else:
                goal_upward = motion_commons.msg.UpwardGoal(
                    Goal=present_depth, loop=0)
                client_vertical.send_goal(goal_upward)

        entry_turn = Entry(window, width=10)

        def turnDataStabilize():
            client_turn = actionlib.SimpleActionClient(
                'turningXY', motion_commons.msg.TurnAction)
            client_turn.wait_for_server()
            goal_turn = motion_commons.msg.TurnGoal(
                AngleToTurn=float(entry_turn.get()), loop=100000)
            if side_stab_var.get() == 1:
                client_turn.send_goal(goal_turn)
            else:
                goal_turn = motion_commons.msg.TurnGoal(
                    AngleToTurn=0.0, loop=0)
                client_turn.send_goal(goal_turn)

        def turnStabilize():
            client_turn = actionlib.SimpleActionClient(
                'turningXY', motion_commons.msg.TurnAction)
            client_turn.wait_for_server()
            goal_turn = motion_commons.msg.TurnGoal(
                AngleToTurn=0.0, loop=100000)
            if side_stab_var.get() == 1:
                client_turn.send_goal(goal_turn)
            else:
                goal_turn = motion_commons.msg.TurnGoal(
                    AngleToTurn=0.0, loop=0)
                client_turn.send_goal(goal_turn)

        def verticalStabilizeCaller(event):
            if vert_stab_var.get() == 1:
                vert_stab_var.set(0)
                verticalStabilize()
            else:
                vert_stab_var.set(1)
                verticalStabilize()

        def turnStabilizeCaller(event):
            if side_stab_var.get() == 1:
                side_stab_var.set(0)
                turnStabilize()
            else:
                side_stab_var.set(1)
                turnStabilize()

# LABEL TOP##################################################################
        label = Label(window, bd=5, font=("Helvetica", 16),
                      fg="blue", anchor=N, text="AUV IITK")
        label.pack()
# ###########################################################################
# TOTAL RESET################################################################
        Button(window, bg="darkmagenta", font=("Helvetica", 14), fg="white",
               height=3, width=10, text="Reset", command=reset).pack()
        window.bind("<space>", reset)
# ###########################################################################
        t1 = Checkbutton(window, text="Vert. Present Stab.", command=verticalStabilize,
                         variable=vert_stab_var, onvalue=1, offvalue=0, height=3, width=20)
        t3 = Checkbutton(window, text="Vert. Data Stab.", command=verticalDataStabilize,
                         variable=vert_stab_var, onvalue=1, offvalue=0, height=3, width=20)
        t2 = Checkbutton(window, text="Turn Present Stab.", command=turnStabilize,
                         variable=side_stab_var, onvalue=1, offvalue=0, height=3, width=20)
        t4 = Checkbutton(window, text="Turn Data Stab.", command=turnDataStabilize,
                         variable=side_stab_var, onvalue=1, offvalue=0, height=3, width=20)
        window.bind("q", verticalStabilizeCaller)
        window.bind("e", turnStabilizeCaller)
        entry_vertical.pack(side=TOP, padx=10, pady=5)
        t3.pack()
        t1.pack()
        entry_turn.pack(side=TOP, padx=10, pady=5)
        t4.pack()
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
        Stop_front = Button(m1, text="stop", fg="yellow",
                            bg="red", command=stop_front)
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
        window.bind('i', stop_vertical)
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
