import rospy
import time
from machinekit import hal
# import service messages from the ROS node
from hal_402_device_mgr.srv import srv_robot_state
from hal_402_drive import drive_402 as drive_402
from hal_402_drive import state_machine_402 as state_402


class hal_402_mgr(object):

    def __init__(self):
        self.compname = 'hal_402_mgr'
        self.drives = dict()
        self.prev_robot_state = 'unknown'
        self.curr_robot_state = 'unknown'
        # very simple transitions for now
        # unknown -> stopped
        # error -> stopped
        # stopped -> error
        # stopped -> started
        # started -> error
        self.states = {
            'unknown':  [state_402.path_to_switch_on_disabled, 'SWITCH ON DISABLED'],
            'started':  [state_402.path_to_operation_enabled, 'OPERATION ENABLED'],
            'stopped':  [state_402.path_to_switch_on_disabled, 'SWITCH ON DISABLED'],
            'error':    [state_402.path_to_switch_on_disabled, 'SWITCH ON DISABLED']
        }

        # create ROS node
        rospy.init_node(self.compname)
        rospy.loginfo("%s: Node started" % self.compname)

        # create HAL userland component
        self.halcomp = hal.Component(self.compname)
        rospy.loginfo("%s: HAL component created" % self.compname)

        # create drives which create pins
        self.create_drives()
        self.halcomp.ready()

        # check if we're running real hardware, and properly
        # - set up drives
        # - connect pins and signals
        self.check_for_real_hardware_setup()

        self.create_service()
        self.create_publisher()

    def has_parameters(self, list_of_parameters):
        has_parameters = True
        for parameter in list_of_parameters:
            has_parameters = rospy.has_param(parameter)
            if (has_parameters is False):
                # exit this list at first missing parameter
                break
        return has_parameters

    def check_for_real_hardware_setup(self):
        # check for existence of parameters
        if self.has_parameters(['/sim', '/sim_mode']):
            # parameters exist, get values
            sim = rospy.get_param('/sim')
            sim_mode = rospy.get_param('/sim_mode')
            if (sim or sim_mode):
                self.sim_set_drivestates('SWITCH ON DISABLED')
                self.sim_set_drive_sim(True)
                rospy.loginfo("%s: no hardware setup detected, default to \
                              simulation mode" % self.compname)
            else:
                rospy.loginfo("%s: hardware setup detected" % self.compname)
                # check for parameter existence
                if self.has_parameters(['/hal_402_device_mgr/slaves/name',
                                        '/hal_402_device_mgr/slaves/instances',
                                        '/hal_402_device_mgr/slaves/wait_timeout',
                                        '/hal_402_device_mgr/slaves/wait_on_pinname']):
                    # get parameters
                    slaves_name = rospy.get_param(
                        '/hal_402_device_mgr/slaves/name')
                    slaves_instances_param = rospy.get_param(
                        '/hal_402_device_mgr/slaves/instances')
                    wait_on_pinname = rospy.get_param(
                        '/hal_402_device_mgr/slaves/wait_on_pinname')
                    last_nr = slaves_instances_param[-1]
                    timeout = rospy.get_param(
                        '/hal_402_device_mgr/slaves/wait_timeout')
                    pin_name = slaves_name + '.%s.%s' % (last_nr, wait_on_pinname)
                    while (not (pin_name in hal.pins) and (timeout > 0)):
                        time.sleep(1)
                        timeout -= 1
                    if timeout <= 0:
                        # a timeout occured
                        rospy.logerr("%s: waiting on slave pins timeout" %
                                     self.compname)
                    else:
                        # we've recognized a pin within the timeout
                        self.connect_pins_and_signals()
                else:
                    rospy.logerr("%s: no correct /hal_402_device_mgr/slaves params" %
                                 self.compname)
        else:
            rospy.logerr("%s: no /sim or /sim_mode parameters" % self.compname)

    def connect_pins_and_signals(self):
        # check for parameters
        # for each drive, connect pins to pins
        # for each drive, connect existing signals to pins
        rospy.loginfo("%s: trying to connect pins" % self.compname)

    def create_drives(self):
        # check if parameters exist:
        if self.has_parameters(['/hal_402_device_mgr/drives/name',
                                '/hal_402_device_mgr/drives/instances']):
            name = rospy.get_param('/hal_402_device_mgr/drives/name')
            instances = rospy.get_param('/hal_402_device_mgr/drives/instances')
            for n in instances:
                # create n drives from ROS parameters
                drivename = name + "_%s" % n
                self.drives[drivename] = drive_402(drivename, self)
                rospy.loginfo("%s: %s created" % (self.compname, drivename))
        else:
            rospy.logerr("%s: no correct /hal_402_device_mgr/drives params" %
                         self.compname)

    def create_publisher(self):
        # todo, read from ROS param server
        has_update_rate = rospy.has_param('/hal_402_device_mgr/update_rate')
        print(has_update_rate)
        if (has_update_rate):
            self.update_rate = rospy.get_param('/hal_402_device_mgr/update_rate')
            self.rate = rospy.Rate(self.update_rate)
            # create publishers for topics and send out a test message
            for key, drive in self.drives.items():
                drive.create_topics()
                if (drive.sim is True):
                    drive.test_publisher()
        else:
            rospy.logerr("%s: no /hal_402_device_mgr/update_rate param found" %
                         self.compname)

    def all_equal_status(self, status):
        # check if all the drives have the same status
        for key, drive in self.drives.items():
            if not (drive.curr_state == status):
                return False
        return True

    def sim_set_drive_sim(self, status):
        # set simulation attribute to mimic hanging statusword
        for key, drive in self.drives.items():
            drive.sim = True

    def sim_set_drivestates(self, status):
        for key, drive in self.drives.items():
            drive.sim_set_input_status_pins(status)
        # give HAL at least 1 cycle to process
        time.sleep(0.002)
        self.inspect_hal_pins()

    def cb_robot_state_service(self, req):
        # The service callback
        # the value of the requested state is in req.req_state (string)
        # the return value is the service response (string)
        # check the requested state for validity
        if req.req_state not in self.states:
            rospy.loginfo("%s: request for state failed, %s not a valid state" %
                          (self.compname,
                           req.req_state))
            return "request for state %s failed, state not known" % req.req_state
        # pick a transition table for the requested state
        tr_table = self.states[req.req_state][0]
        all_target_states = self.states[req.req_state][1]
        # set "active" transition table for the drive
        for key, drive in self.drives.items():
            drive.set_transition_table(tr_table)

        # the drive returns success if all the drives are :
        # OPERATION ENABLED or SWITCH ON DISABLED or max_attempts
        i = 0
        max_attempts = len(state_402.states_402)
        print(all_target_states)
        print(not self.all_equal_status(all_target_states))
        while ((not self.all_equal_status(all_target_states))
               or (i < (max_attempts + 1))):
            for key, drive in self.drives.items():
                # traverse states for all drives (parallel)
                # ignore drives which state already is at the target state
                if not (drive.curr_state == all_target_states):
                    drive.next_transition()
            # give HAL time to make at least 1 cycle
            time.sleep(0.002)
            self.inspect_hal_pins()
            self.publish_states()
            i += 1

        # when states are successfully reached, update overall state
        # when all statuses have been reached, success
        # otherwise num_states has overflowed
        if self.all_equal_status(all_target_states):
            self.curr_robot_state = req.req_state
        else:
            self.curr_robot_state = 'error'
        return self.curr_robot_state

    def cb_test_service_cb(self, req):
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
            # if drive.status_word_changed():
            #     print('{}: {} status_word: {:#010b} status: {}'.format(
            #                                         self.compname,
            #                                         drive.drive_name,
            #                                         drive.curr_status_word,
            #                                         drive.curr_state))
            # else:
            #     do nothing cause nothing has changed
            #     pass

    def publish_states(self):
        for key, drive in self.drives.items():
            drive.publish_state()

    def run(self):
        while not rospy.is_shutdown():
            self.inspect_hal_pins()
            self.publish_states()
            self.rate.sleep()
