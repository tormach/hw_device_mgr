import rospy
import hal
# import service messages from the ROS node
from hal_402_device_mgr.srv import srv_robot_state
from hal_402_drive import drive_402 as drive_402


class hal_402_mgr(object):

    def __init__(self):
        self.compname = 'hal_402_mgr'
        self.drives = dict()
        self.prev_robot_state = 'unknown'
        self.curr_robot_state = 'unknown'

        # create ROS node
        rospy.init_node(self.compname)
        rospy.loginfo("%s: Node started" % self.compname)

        # create HAL userland component
        self.halcomp = hal.component(self.compname)
        rospy.loginfo("%s: HAL component created" % self.compname)

        # create drives which create pins
        self.create_drives()
        self.halcomp.ready()

        self.create_service()
        self.create_publisher()

    def create_drives(self):
        for i in range(0, 6):
            # create 6 drives, later do this from ROS parameters
            drivename = "drive_%s" % (i + 1)
            self.drives[drivename] = drive_402(drivename, self)
            rospy.loginfo("%s: %s created" % (self.compname, drivename))

    def create_publisher(self):
        # todo, read from ROS param server
        self.update_rate = 1  # Hz
        self.rate = rospy.Rate(self.update_rate)
        # create publishers for topics and send out a test message
        for key, drive in self.drives.items():
            drive.create_topics()
            drive.test_publisher()

    def cb_robot_state_service(self, req):
        # the value of the requested state is in req.req_state
        return self.curr_robot_state

    def cb_test_service_cb(self, req):
        print(req.req_state)  # show abcd in the terminsal
        rospy.loginfo("%s: cb_test_service" % self.compname)
        # the response that's returned is the return value of the callback
        return "test service return string"

    def create_service(self):
        # $ rosservice call /hal_402_drives_mgr abcd
        # will call the function callback, that will receive the
        # service message as an argument
        # testin routine:
        # self.service = rospy.Service('hal_402_drives_mgr',
        #                             srv_robot_state,
        #                             self.cb_test_service)
        self.service = rospy.Service('hal_402_drives_mgr',
                                     srv_robot_state,
                                     self.cb_robot_state_service)
        rospy.loginfo("%s: service %s created" %
                      (self.compname,
                       self.service.resolved_name))

    def inspect_hal_pins(self):
        for key, drive in self.drives.items():
            drive.read_halpins()
            drive.calculate_status_word()
            drive.calculate_state()
            print('{}: {} status_word: {:#010b} status: {}'.format(
                                                   self.compname,
                                                   drive.drive_name,
                                                   drive.curr_status_word,
                                                   drive.curr_state))

    def publish_states(self):
        for key, drive in self.drives.items():
            drive.publish_state()

    def run(self):
        while not rospy.is_shutdown():
            self.inspect_hal_pins()
            self.publish_states()
            self.rate.sleep()
