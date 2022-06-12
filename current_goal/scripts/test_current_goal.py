#!/usr/bin/env python

PKG='current_goal'

import rospy
import rosunit
import unittest
from std_msgs.msg import String, Bool

class TestCurrentGoal(unittest.TestCase):

    def test_present_goal(self):
        def callback(msg):
            goals = ['Captain_America', 'Hulk', 'Iron_Man', '']
            if(msg.data in goals):
                self.assertTrue(True)

        rospy.Subscriber('/current_goal', String, callback)

    def test_next_goals(self):
        self.next_goals_ok = False
        self.goals = ['Captain_America', 'Hulk', 'Iron_Man', '']

        def callback(msg):
            if(len(self.goals) == 0):
                self.next_goals_ok = True
            else:
                if(msg.data in self.goals):
                    try:
                        self.goals.remove(msg.data)
                    except ValueError:
                        pass

        rospy.Subscriber('/current_goal', String, callback)

        while(len(self.goals) != 0):
            try:
                rate = rospy.Rate(.5)
                pub = rospy.Publisher('/goal_success', Bool, queue_size=1)
                pub.publish(Bool(True))
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

        self.assertTrue(self.next_goals_ok)

if __name__ == '__main__':
    rospy.init_node('test_current_goal')
    rosunit.unitrun(PKG, 'test_current_goal', TestCurrentGoal)
