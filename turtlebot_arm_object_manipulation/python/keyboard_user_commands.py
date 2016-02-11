#!/usr/bin/env python

import rospy

import Tkinter as tk
from turtlebot_arm_object_manipulation.msg import *

from actionlib import *


USER_COMMANDS = {
    's': "start",
    'r': "reset",
    'f': "fold",
    'q': "quit"
}

def on_key_press(event):
    # Clean text and reinsert the header
    text.delete('1.0', 'end')
    text.insert('end', text_header)

    try:
        rospy.loginfo("Key pressed: %s -> command: %s", event.char, USER_COMMANDS[event.char])
        
        # Show the valid command about to execute
        text.insert('end', "\n\n'%s' command selected" % USER_COMMANDS[event.char])
    
        # Creates a goal to send to the action server.
        goal = UserCommandGoal(command=USER_COMMANDS[event.char])
    
        # Sends the goal to the action server.
        client.send_goal(goal)

        # TODO: Maybe I should call this in a separated thread... just for information
        #     # Waits for the server to finish performing the action.
        #     client.wait_for_result()
        #  
        #     # Prints out the result of executing the action
        #     rospy.loginfo("SM result: %s", client.get_result())  # probably empty... not implemented
    except KeyError:
        # Invalid command; nothing to do
        text.insert('end', "\n\n'%s' is not a valid command" % event.char)



if __name__ == '__main__':
    try:
        rospy.init_node('keyboard_user_commands')

        # Creates the SimpleActionClient, passing the type of the action
        # (UserCommandAction) to the constructor.
        client = SimpleActionClient('user_commands_action_server', UserCommandAction)
    
        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        # Create a simple GUI to show available commands and capture keyboard inputs
        window = tk.Tk()
        window.geometry(rospy.get_param('~window_geometry', '300x200'))
        window.wm_title(rospy.get_param('~window_caption', 'User input'))

        text = tk.Text(window, background='black', foreground='white',
                       font=(rospy.get_param('~text_font', 'Comic Sans MS'), rospy.get_param('~font_size', 12)))
        text_header = rospy.get_param('~shown_text', 'Press command key')
        text.insert('end', text_header)
        text.pack()

        window.bind('<KeyPress>', on_key_press)
        window.mainloop()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
