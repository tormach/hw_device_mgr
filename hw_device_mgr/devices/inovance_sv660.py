from ..ethercat.device import EtherCATDevice, EtherCATSimDevice
from ..ethercat.config import EtherCATConfig
from ..cia_402.device import CiA402Device, CiA402SimDevice
from ..errors.device import ErrorDevice
from functools import lru_cache
import time


class InovanceSV660Config(EtherCATConfig):
    """Inovance SV660 servo drive config."""

    # Device params non-volatile setting in 200E-02h:
    # 0:  params not updated
    # 1:  2000h series changed from serial or EtherCAT saved
    # 2:  6000h series changed from EtherCAT (only) saved
    # 3:  2000h and 6000h series changed from EtherCAT (only) saved

    def get_device_params_nv(self):
        return self.config.upload("200E-02h") == 3

    def set_device_params_nv(self, nv=True, dry_run=False):
        curr_setting = self.get_device_params_nv()
        if curr_setting != nv:
            self.logger.info(
                f"Setting params {'non' if nv else ''}volatile mode"
            )
            self.config.download(
                "200E-02h", (0, 3)[nv], force=True, dry_run=dry_run
            )
        else:
            self.logger.info(
                f"Params already {'non' if nv else ''}volatile mode"
            )

    def clear_fault(self):
        err_code = self.upload("203Fh")
        self.logger.info(f"Resetting device fault; code: {err_code}")
        time.sleep(0.5)  # This may or may not be needed
        self.download("200D-02h", 1)
        time.sleep(0.5)  # This may or may not be needed
        err_code = self.upload("203Fh")
        self.logger.info(f"  After fault reset, code: {err_code}")

    def soft_reset(self):
        self.logger.info("Performing soft reset on drive")
        time.sleep(0.5)  # This may or may not be needed
        self.download("200D-01h", 1)
        time.sleep(2.0)  # Give the drive time to react

    def reset_error_and_restart(self):
        self.logger.info(f"Queueing fault reset and rebooting drive")
        self.enqueue_command(self.clear_fault)
        self.enqueue_command(self.soft_reset)


class InovanceSV660(EtherCATDevice, CiA402Device, ErrorDevice):
    """Inovance SV660 servo drives."""

    vendor_id = 0x00100000
    product_code = 0x000C010D
    xml_description_package = "hw_device_mgr.devices.device_xml"
    xml_description_fname = "SV660_EOE_1Axis_V9.12.xml"
    device_error_package = "hw_device_mgr.devices.device_err"
    device_error_yaml = "inovance_sv660n.yaml"
    config_class = InovanceSV660Config
    have_sto = True

    feedback_out_data_types = dict(
        home_found="bit",
        error_code="uint16",  # Override uint32
    )
    feedback_out_overlap = {"error_code"}  # Override uint32

    feedback_out_defaults = dict(
        home_found=False,
    )

    @classmethod
    @lru_cache
    def to_error_code_data_type(cls, value):
        # Ignore least significant word, "manufacturer external fault code"
        dt = cls.data_type_class.by_shared_name("uint16")
        return dt(value >> 16)

    def get_error_code(self):
        error_code = self.interface("feedback_in").get("error_code")
        return self.to_error_code_data_type(error_code)

    def get_feedback(self):
        fb_out = super().get_feedback()
        sw = self.interface("feedback_in").get("status_word")
        if self.test_sw_bit(sw, "MANUFACTURER_SPECIFIC_3"):  # "Home found"
            fb_out.update(home_found=True)
        return fb_out

    def set_command(self, **kwargs):
        cmd_out = super().set_command(**kwargs)
        cmd_in = self._interfaces["command_in"]
        fb_in = self._interfaces["feedback_in"]
        if cmd_in.rising_edge("reset_fault"):
            if fb_in.get("error_code") == 0x07310731:
                self.config.reset_error_and_restart()
        return cmd_out


class SimInovanceSV660(InovanceSV660, EtherCATSimDevice, CiA402SimDevice):
    def set_sim_feedback(self):
        # Simulate home_found feedback
        sfb = super().set_sim_feedback()
        sw, old_sw = sfb.changed("status_word", return_vals=True)
        # In MODE_HM, OPERATION_MODE_SPECIFIC_1 is "homing attained"
        homing_attained = False
        if sfb.get("control_mode_fb") == self.MODE_HM:
            homing_attained = self.test_sw_bit(sw, "OPERATION_MODE_SPECIFIC_1")
        # SV660N:  MANUFACTURER_SPECIFIC_3 (bit 15) is "Home found"
        old_home_found = self.test_sw_bit(old_sw, "MANUFACTURER_SPECIFIC_3")
        home_found = old_home_found or homing_attained
        if home_found:
            sw = self.add_status_word_flags(sw, MANUFACTURER_SPECIFIC_3=True)
            sfb.update(status_word=sw)
        return sfb
